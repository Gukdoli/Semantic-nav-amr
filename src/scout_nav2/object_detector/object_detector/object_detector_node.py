"""object_detector_node: RGB-D -> open-vocabulary detection -> 3D map points.

Pipeline (SPEC 2.2):
  color+depth (ApproximateTimeSynchronizer) ----> single-slot frame buffer
  worker thread (throttled to inference_rate_hz):
      YOLO-World detect -> per-box median depth -> deproject (camera_info K)
      -> tf2 transform image frame -> map -> publish DetectedObject3DArray

Heavy inference never runs in the subscriber callback (CLAUDE rule): the
callback only stashes the latest synced frame; a background worker does the
model + projection work.
"""

from __future__ import annotations

import threading
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

import message_filters
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import CameraInfo, Image

import tf2_ros
from tf2_geometry_msgs import do_transform_point  # noqa: F401 (registers Point)

from semantic_nav_msgs.msg import DetectedObject3D, DetectedObject3DArray

from object_detector.detector import YoloeDetector
from object_detector.projection import deproject, median_depth


class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__("object_detector_node")

        # --- Parameters (declared + yaml managed; no hardcoding) ---
        self.declare_parameter("target_classes", ["fire extinguisher", "chair"])
        # Optional richer detection prompts, parallel to target_classes. Fed to
        # YOLOE for detection while results are stored under target_classes (the
        # canonical label). Empty -> use target_classes as the prompts.
        self.declare_parameter("detection_prompts", [""])
        self.declare_parameter("model_path", "yoloe-11s-seg.pt")
        self.declare_parameter("min_confidence", 0.5)
        self.declare_parameter("slop", 0.1)
        self.declare_parameter("inference_rate_hz", 5.0)
        self.declare_parameter("depth_patch_ratio", 0.2)
        self.declare_parameter("target_frame", "map")
        # Inference device: "" -> auto (cuda:0 if available else cpu), or force
        # e.g. "cuda:0" / "cpu".
        self.declare_parameter("device", "")
        # Inference resolution (larger -> better small-object recall, slower).
        self.declare_parameter("imgsz", 640)
        # Publish an annotated RGB image (raw detections drawn) for debugging.
        self.declare_parameter("publish_debug_image", True)

        self.target_classes = list(
            self.get_parameter("target_classes").get_parameter_value().string_array_value
        )
        prompts = list(
            self.get_parameter("detection_prompts")
            .get_parameter_value()
            .string_array_value
        )
        # Treat empty / placeholder-only lists as "no override".
        prompts = [p for p in prompts if p]
        self.detection_prompts = (
            prompts if len(prompts) == len(self.target_classes) else None
        )
        self.model_path = self.get_parameter("model_path").value
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.slop = float(self.get_parameter("slop").value)
        self.inference_rate_hz = float(self.get_parameter("inference_rate_hz").value)
        self.depth_patch_ratio = float(self.get_parameter("depth_patch_ratio").value)
        self.target_frame = self.get_parameter("target_frame").value
        self.device = self.get_parameter("device").value
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.publish_debug_image = bool(
            self.get_parameter("publish_debug_image").value
        )

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Latest cached intrinsics (camera_info is not part of the sync; it is
        # near-constant so we just keep the most recent one).
        self._camera_info: Optional[CameraInfo] = None

        # Single-slot frame buffer: the worker only ever processes the newest
        # synced (rgb, depth) pair, dropping anything it couldn't keep up with.
        self._frame_lock = threading.Lock()
        self._latest_frame = None  # tuple(color_msg, depth_msg)
        self._stop = threading.Event()

        self.pub = self.create_publisher(
            DetectedObject3DArray, "/semantic_nav/detections", 10
        )
        self.debug_pub = (
            self.create_publisher(Image, "/semantic_nav/debug_image", 10)
            if self.publish_debug_image
            else None
        )

        self.create_subscription(
            CameraInfo, "/camera/color/camera_info", self._on_camera_info, 10
        )
        color_sub = message_filters.Subscriber(self, Image, "/camera/color/image_raw")
        depth_sub = message_filters.Subscriber(self, Image, "/camera/depth/image_raw")
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], queue_size=10, slop=self.slop
        )
        self._sync.registerCallback(self._on_frame)

        # Detector is constructed lazily on the worker thread so the node still
        # starts (and logs a clear install hint) when ultralytics is missing.
        self._detector = None
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self.get_logger().info(
            f"object_detector_node up. classes={self.target_classes} "
            f"model={self.model_path} rate={self.inference_rate_hz}Hz"
        )

    # --- Callbacks (lightweight only) ---
    def _on_camera_info(self, msg: CameraInfo):
        self._camera_info = msg

    def _on_frame(self, color_msg: Image, depth_msg: Image):
        with self._frame_lock:
            self._latest_frame = (color_msg, depth_msg)

    # --- Worker thread: heavy work lives here ---
    def _ensure_detector(self) -> bool:
        if self._detector is not None:
            return True
        try:
            self._detector = YoloeDetector(
                self.model_path,
                self.target_classes,
                self.device,
                self.imgsz,
                prompts=self.detection_prompts,
            )
            self.get_logger().info(
                f"YOLOE detector loaded (device={self._detector.device}, "
                f"imgsz={self.imgsz}, classes={self.target_classes}, "
                f"prompts={self.detection_prompts or self.target_classes})."
            )
            return True
        except ImportError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=30.0)
            return False
        except Exception as exc:  # pragma: no cover - model/runtime issues
            self.get_logger().error(
                f"Failed to load detector: {exc}", throttle_duration_sec=30.0
            )
            return False

    def _worker_loop(self):
        period = 1.0 / self.inference_rate_hz if self.inference_rate_hz > 0 else 0.2
        while not self._stop.wait(period):
            if self._camera_info is None:
                continue
            if not self._ensure_detector():
                continue
            with self._frame_lock:
                frame = self._latest_frame
                self._latest_frame = None
            if frame is None:
                continue
            try:
                self._process(*frame)
            except Exception as exc:  # pragma: no cover - keep worker alive
                self.get_logger().error(
                    f"frame processing error: {exc}", throttle_duration_sec=10.0
                )

    def _process(self, color_msg: Image, depth_msg: Image):
        rgb = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="rgb8")
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        depth = np.asarray(depth, dtype=np.float32)

        k = self._camera_info.k
        detections = self._detector.detect(rgb)

        out = DetectedObject3DArray()
        for det in detections:
            # Per-detection outcome, recorded for the debug overlay so a dropped
            # detection is visibly distinguishable from a published one.
            if det.confidence < self.min_confidence:
                self._draw_debug(rgb, det, "low-conf")
                continue
            point_cam = self._bbox_to_camera_point(det.bbox, depth, k)
            if point_cam is None:
                self._draw_debug(rgb, det, "no-depth")
                continue
            point_map = self._to_target_frame(point_cam, color_msg.header)
            if point_map is None:
                self._draw_debug(rgb, det, "no-tf")
                continue
            obj = DetectedObject3D()
            obj.header.stamp = color_msg.header.stamp
            obj.header.frame_id = self.target_frame
            obj.label = det.label
            obj.confidence = float(det.confidence)
            obj.position = point_map
            out.objects.append(obj)
            self._draw_debug(rgb, det, "ok")

        # DetectedObject3DArray has no header (SPEC 2.1); each DetectedObject3D
        # carries its own header (frame_id=map, image timestamp).
        self.pub.publish(out)

        if self.debug_pub is not None:
            msg = self.bridge.cv2_to_imgmsg(rgb, encoding="rgb8")
            msg.header = color_msg.header
            self.debug_pub.publish(msg)

    # Outcome -> box colour (RGB, since the debug image is published as rgb8).
    _DEBUG_COLORS = {
        "ok": (0, 255, 0),        # published
        "low-conf": (255, 180, 0),  # below min_confidence
        "no-depth": (255, 0, 0),   # no valid depth in bbox
        "no-tf": (255, 0, 255),    # TF lookup failed
    }

    def _draw_debug(self, rgb, det, status):
        """Annotate `rgb` in place with one detection box + label + status."""
        if self.debug_pub is None:
            return
        x1, y1, x2, y2 = (int(v) for v in det.bbox)
        color = self._DEBUG_COLORS.get(status, (200, 200, 200))
        cv2.rectangle(rgb, (x1, y1), (x2, y2), color, 2)
        cv2.putText(
            rgb,
            f"{det.label} {det.confidence:.2f} [{status}]",
            (x1, max(0, y1 - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            1,
            cv2.LINE_AA,
        )

    def _bbox_to_camera_point(self, bbox, depth, k):
        x1, y1, x2, y2 = bbox
        h, w = depth.shape[:2]
        cx = 0.5 * (x1 + x2)
        cy = 0.5 * (y1 + y2)
        # Central patch of the bbox, sized by depth_patch_ratio.
        half_w = max(1.0, self.depth_patch_ratio * (x2 - x1) * 0.5)
        half_h = max(1.0, self.depth_patch_ratio * (y2 - y1) * 0.5)
        u0 = max(0, int(cx - half_w))
        u1 = min(w, int(cx + half_w) + 1)
        v0 = max(0, int(cy - half_h))
        v1 = min(h, int(cy + half_h) + 1)
        if u1 <= u0 or v1 <= v0:
            return None
        patch = depth[v0:v1, u0:u1].reshape(-1)
        z = median_depth(patch.tolist())
        if z is None:
            return None
        x, y, z = deproject(k, cx, cy, z)
        return (x, y, z)

    def _to_target_frame(self, point_cam, source_header):
        """Transform a camera-frame point into target_frame using tf2.

        Source frame is the image's header.frame_id (camera_color_optical_frame),
        never a hardcoded camera_link. Lookup uses the image timestamp.
        """
        ps = PointStamped()
        ps.header.frame_id = source_header.frame_id
        ps.header.stamp = source_header.stamp
        ps.point.x, ps.point.y, ps.point.z = point_cam
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_header.frame_id,
                rclpy.time.Time.from_msg(source_header.stamp),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            self.get_logger().warn(
                f"TF {source_header.frame_id}->{self.target_frame} failed: {exc}",
                throttle_duration_sec=5.0,
            )
            return None
        transformed = do_transform_point(ps, tf)
        return Point(x=transformed.point.x, y=transformed.point.y, z=transformed.point.z)

    def destroy_node(self):
        self._stop.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
