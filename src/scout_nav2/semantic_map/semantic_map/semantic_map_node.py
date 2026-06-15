"""semantic_map_node: accumulate detections, serve lookups, publish markers.

Subscribes /semantic_nav/detections, merges into an ObjectStore, answers
/semantic_nav/find_object (FindObject -> all confirmed instances of a label),
and publishes RViz markers on /semantic_nav/object_markers.

Confirmation: an object must be observed >= `min_observations` times before it
counts as "confirmed". Confirmed objects show as solid green markers and are
returned by find_object; unconfirmed ones (e.g. one-off false positives, more
likely now that min_confidence is low) show as faint grey markers and are
excluded from lookups.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from semantic_nav_msgs.msg import DetectedObject3D, DetectedObject3DArray
from semantic_nav_msgs.srv import FindObject

from semantic_map.object_store import ObjectStore


class SemanticMapNode(Node):
    def __init__(self):
        super().__init__("semantic_map_node")

        self.declare_parameter("merge_distance", 0.5)
        self.declare_parameter("ema_alpha", 0.3)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("marker_publish_rate", 2.0)
        self.declare_parameter("marker_scale", 0.3)
        self.declare_parameter("text_height", 0.4)
        # Min times an object must be observed before it is confirmed (returned
        # by find_object + shown solid). Acts as a noise filter for the low
        # min_confidence used to boost recall.
        self.declare_parameter("min_observations", 3)

        self.frame_id = self.get_parameter("frame_id").value
        self.marker_scale = float(self.get_parameter("marker_scale").value)
        self.text_height = float(self.get_parameter("text_height").value)
        self.min_observations = int(self.get_parameter("min_observations").value)
        rate = float(self.get_parameter("marker_publish_rate").value)

        self.store = ObjectStore(
            merge_distance=float(self.get_parameter("merge_distance").value),
            ema_alpha=float(self.get_parameter("ema_alpha").value),
        )

        self.create_subscription(
            DetectedObject3DArray,
            "/semantic_nav/detections",
            self._on_detections,
            10,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, "/semantic_nav/object_markers", 10
        )
        self.create_service(
            FindObject, "/semantic_nav/find_object", self._on_find_object
        )
        self.create_timer(1.0 / rate if rate > 0 else 0.5, self._publish_markers)

        self.get_logger().info("semantic_map_node up.")

    def _on_detections(self, msg: DetectedObject3DArray):
        for obj in msg.objects:
            stamp = obj.header.stamp.sec + obj.header.stamp.nanosec * 1e-9
            self.store.update(
                obj.label,
                (obj.position.x, obj.position.y, obj.position.z),
                obj.confidence,
                stamp,
            )

    def _on_find_object(self, request, response):
        # Return all confirmed instances of the label (empty -> not found).
        for obj in self.store.find(request.label, self.min_observations):
            match = DetectedObject3D()
            match.header.frame_id = self.frame_id
            match.header.stamp = self._to_time_msg(obj.last_seen)
            match.label = obj.label
            match.confidence = float(obj.confidence)
            match.position = Point(x=obj.x, y=obj.y, z=obj.z)
            response.matches.append(match)
        return response

    def _publish_markers(self):
        array = MarkerArray()
        # Wipe stale markers each cycle so removed objects don't linger.
        clear = Marker()
        clear.header.frame_id = self.frame_id
        clear.action = Marker.DELETEALL
        array.markers.append(clear)

        for obj_id, obj in self.store.items():
            confirmed = obj.count >= self.min_observations
            array.markers.append(self._sphere_marker(obj_id, obj, confirmed))
            array.markers.append(self._text_marker(obj_id, obj, confirmed))
        self.marker_pub.publish(array)

    def _sphere_marker(self, obj_id: int, obj, confirmed: bool) -> Marker:
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        # Separate namespaces so confirmed/unconfirmed can be toggled in RViz.
        m.ns = "objects" if confirmed else "unconfirmed"
        m.id = obj_id
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position = Point(x=obj.x, y=obj.y, z=obj.z)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = self.marker_scale
        if confirmed:
            m.color = ColorRGBA(r=0.1, g=0.8, b=0.2, a=0.9)  # solid green
        else:
            m.color = ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.35)  # faint grey
        return m

    def _text_marker(self, obj_id: int, obj, confirmed: bool) -> Marker:
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "labels" if confirmed else "unconfirmed_labels"
        m.id = obj_id
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position = Point(x=obj.x, y=obj.y, z=obj.z + self.marker_scale)
        m.pose.orientation.w = 1.0
        m.scale.z = self.text_height
        if confirmed:
            m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            m.text = f"{obj.label} ({obj.confidence:.2f})"
        else:
            # Show progress toward confirmation for debugging.
            m.color = ColorRGBA(r=0.7, g=0.7, b=0.7, a=0.6)
            m.text = f"{obj.label}? ({obj.count}/{self.min_observations})"
        return m

    @staticmethod
    def _to_time_msg(seconds: float) -> TimeMsg:
        t = TimeMsg()
        t.sec = int(seconds)
        t.nanosec = int((seconds - int(seconds)) * 1e9)
        return t


def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapNode()
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
