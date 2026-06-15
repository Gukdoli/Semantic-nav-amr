"""goal_commander_node: natural-language command -> Nav2 navigation goal (M4).

Pipeline (SPEC 2.4):
  NavigateToObject service request (free-text command)
    -> command_parser.parse (keyword match)            -> target label
    -> /semantic_nav/find_object (FindObject)           -> confirmed matches (map)
    -> pick nearest to the robot (tf2 map<-robot pose)
    -> approach.compute_approach_pose (offset + facing)
    -> NavigateToPose action (send goal, await accept/reject)
    -> respond accepted = "Nav2 accepted the goal" (ASYNC: not arrival)

The response is asynchronous: `accepted` means Nav2 accepted the goal, and the
message carries the selected instance and its distance. Arrival/stop is observed
in RViz, not in the service response (a future status topic is out of M4 scope).

Costmap occupancy validation is delegated to Nav2: we send a plain offset pose
and, if Nav2 rejects the goal, surface that as accepted=false (the 8-direction
candidate search of SPEC 2.4.4 is deferred to M5).

Threading: the navigate callback blocks on two nested waits (the find_object
client call, then the Nav2 goal accept/reject). On a single-threaded executor
those would deadlock, so the node runs on a MultiThreadedExecutor with every
endpoint (service server, find_object client, action client) in one
ReentrantCallbackGroup.
"""

from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from semantic_nav_msgs.srv import FindObject, NavigateToObject

from language_goal.approach import compute_approach_pose, nearest
from language_goal.command_parser import DEFAULT_LABEL_SYNONYMS, parse


class GoalCommanderNode(Node):
    def __init__(self):
        super().__init__("goal_commander_node")

        self.declare_parameter("approach_distance", 0.7)
        self.declare_parameter("robot_base_frame", "mobile_robot_base_link")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("nav_action_name", "navigate_to_pose")
        self.declare_parameter("find_object_service", "/semantic_nav/find_object")
        self.declare_parameter("tf_timeout", 0.5)
        self.declare_parameter("nav_server_wait_sec", 5.0)
        self.declare_parameter("find_object_wait_sec", 5.0)
        # Flat list of labels this command pipeline understands (M4: single
        # class). Per-label synonyms come from DEFAULT_LABEL_SYNONYMS; a proper
        # per-label synonym param structure is M5.
        self.declare_parameter("target_labels", ["fire extinguisher"])

        self.approach_distance = float(self.get_parameter("approach_distance").value)
        self.robot_base_frame = self.get_parameter("robot_base_frame").value
        self.target_frame = self.get_parameter("target_frame").value
        self.nav_action_name = self.get_parameter("nav_action_name").value
        self.find_object_service = self.get_parameter("find_object_service").value
        self.tf_timeout = float(self.get_parameter("tf_timeout").value)
        self.nav_server_wait_sec = float(self.get_parameter("nav_server_wait_sec").value)
        self.find_object_wait_sec = float(
            self.get_parameter("find_object_wait_sec").value
        )
        target_labels = list(self.get_parameter("target_labels").value)

        # Restrict the synonym map to the configured labels (default-injected).
        self.label_synonyms = {
            label: DEFAULT_LABEL_SYNONYMS.get(label, [label])
            for label in target_labels
        }

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # One reentrant group so the navigate callback can wait on the
        # find_object client and the action client without deadlocking.
        self.cb_group = ReentrantCallbackGroup()

        self.find_client = self.create_client(
            FindObject, self.find_object_service, callback_group=self.cb_group
        )
        self.nav_client = ActionClient(
            self, NavigateToPose, self.nav_action_name, callback_group=self.cb_group
        )
        self.create_service(
            NavigateToObject,
            "/semantic_nav/navigate_to_object",
            self._on_navigate,
            callback_group=self.cb_group,
        )

        self.get_logger().info("goal_commander_node up.")

    # ------------------------------------------------------------------ helpers
    @staticmethod
    def _wait_for_future(future, timeout_sec: float):
        """Poll a future to completion (other executor threads resolve it)."""
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if future.done():
                return future.result()
            time.sleep(0.02)
        return None

    def _robot_xy(self) -> Optional[tuple]:
        """Robot (x, y) in the target (map) frame via tf2, or None on failure."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.robot_base_frame,
                rclpy.time.Time(),  # latest
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            self.get_logger().warn(
                f"robot pose TF {self.target_frame}<-{self.robot_base_frame} "
                f"failed: {exc}"
            )
            return None
        t = tf.transform.translation
        return (t.x, t.y)

    def _make_goal_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.target_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        # Yaw-only quaternion (z up).
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    # ----------------------------------------------------------------- callback
    def _on_navigate(self, request, response):
        # 1. Parse the command.
        parsed = parse(request.command, self.label_synonyms)
        if parsed is None:
            response.accepted = False
            response.message = (
                f"명령을 이해하지 못했습니다: '{request.command}' "
                f"(아는 라벨: {list(self.label_synonyms)})"
            )
            self.get_logger().info(response.message)
            return response
        label = parsed.target_label

        # 2. Query the semantic map for confirmed instances.
        if not self.find_client.wait_for_service(timeout_sec=self.find_object_wait_sec):
            response.accepted = False
            response.message = (
                f"find_object 서비스({self.find_object_service})를 사용할 수 없습니다."
            )
            self.get_logger().warn(response.message)
            return response
        req = FindObject.Request()
        req.label = label
        future = self.find_client.call_async(req)
        result = self._wait_for_future(future, self.find_object_wait_sec)
        if result is None:
            response.accepted = False
            response.message = "find_object 응답 시간 초과."
            self.get_logger().warn(response.message)
            return response
        matches = result.matches
        if not matches:
            response.accepted = False
            response.message = f"'{label}' 객체를 찾지 못했습니다 (확정된 인스턴스 없음)."
            self.get_logger().info(response.message)
            return response

        # 3. Robot pose (for nearest selection + approach direction).
        robot_xy = self._robot_xy()
        if robot_xy is None:
            response.accepted = False
            response.message = "로봇 위치(TF) 조회에 실패했습니다."
            return response

        # 4. Pick nearest instance.
        points = [(m.position.x, m.position.y) for m in matches]
        idx, dist = nearest(points, robot_xy)
        chosen = matches[idx]
        if len(matches) > 1:
            select_msg = (
                f"'{label}' {len(matches)}개 발견, 가까운 것으로 이동 (거리 {dist:.1f}m)."
            )
        else:
            select_msg = f"'{label}' 1개로 이동 (거리 {dist:.1f}m)."

        # 5. Approach pose + Nav2 goal.
        ax, ay, yaw = compute_approach_pose(
            (chosen.position.x, chosen.position.y), robot_xy, self.approach_distance
        )
        goal_pose = self._make_goal_pose(ax, ay, yaw)

        if not self.nav_client.wait_for_server(timeout_sec=self.nav_server_wait_sec):
            response.accepted = False
            response.message = (
                f"Nav2 액션 서버({self.nav_action_name})를 사용할 수 없습니다. "
                f"({select_msg})"
            )
            self.get_logger().warn(response.message)
            return response

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        send_future = self.nav_client.send_goal_async(goal)
        goal_handle = self._wait_for_future(send_future, self.nav_server_wait_sec)
        if goal_handle is None:
            response.accepted = False
            response.message = f"Nav2 목표 전송 시간 초과. ({select_msg})"
            self.get_logger().warn(response.message)
            return response
        if not goal_handle.accepted:
            response.accepted = False
            response.message = f"Nav2가 목표를 거부했습니다. ({select_msg})"
            self.get_logger().warn(response.message)
            return response

        # Log the eventual result for debugging (out of the response's scope).
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

        response.accepted = True
        response.message = (
            f"{select_msg} 목표 ({ax:.2f}, {ay:.2f}) 수락됨."
        )
        self.get_logger().info(response.message)
        return response

    def _on_nav_result(self, future):
        try:
            status = future.result().status
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Nav2 결과 콜백 오류: {exc}")
            return
        self.get_logger().info(f"Nav2 navigation finished, status={status}.")


def main(args=None):
    rclpy.init(args=args)
    node = GoalCommanderNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
