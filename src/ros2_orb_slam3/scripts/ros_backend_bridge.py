#!/usr/bin/env python3
"""Bridge ORB-SLAM3 ROS topics to the slamdoteetee backend HTTP API."""

from __future__ import annotations

import json
import math
import os
import sys
import urllib.error
import urllib.request

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String


def yaw_deg_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return (math.degrees(math.atan2(siny_cosp, cosy_cosp)) + 360.0) % 360.0


def tracking_to_slam_status(status: str) -> str:
    normalized = status.strip().upper()
    if normalized == "OK":
        return "tracking"
    if normalized == "RECENTLY_LOST":
        return "recently_loss"
    if normalized == "LOST":
        return "tracking_loss"
    return "tracking"


class RosBackendBridge(Node):
    def __init__(self) -> None:
        super().__init__("ros_backend_bridge")
        self.backend_url = os.environ.get("ROBOT_API_BASE_URL", "http://127.0.0.1:8080").rstrip("/")
        self.publish_ms = float(os.environ.get("POSE_PUBLISH_MS", "200")) / 1000.0

        self.pose_payload: dict | None = None
        self.slam_payload: dict | None = None
        self.pipeline_payload: dict = {
            "use_imu": True,
            "use_drift_correction": True,
            "drift_risk_score": 0.0,
            "drift_predicted_error": 0.0,
        }
        self.last_compute_ms = 0.0
        self.last_tracking = "UNKNOWN"

        self.create_subscription(PoseStamped, "/orbslam3/pose", self._on_pose, 10)
        self.create_subscription(String, "/orbslam3/tracking_status", self._on_tracking, 10)
        self.create_subscription(Float64, "/orbslam3/compute_time_ms", self._on_compute, 10)
        self.create_subscription(Float64, "/orbslam3/drift_risk_score", self._on_drift_risk, 10)
        self.create_subscription(Float64, "/orbslam3/drift_predicted_error", self._on_drift_error, 10)
        self.create_subscription(Bool, "/orbslam3/imu_enabled", self._on_imu_enabled, 10)
        self.create_subscription(Bool, "/orbslam3/drift_correction_enabled", self._on_drift_enabled, 10)
        self.create_timer(self.publish_ms, self._publish_to_backend)

        self.get_logger().info(f"Bridging /orbslam3/* -> {self.backend_url}")

    def _on_pose(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        self.pose_payload = {
            "topic": "/odom",
            "ros_enabled": True,
            "status": "live",
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "z": msg.pose.position.z,
            "yaw_deg": yaw_deg_from_quaternion(q.x, q.y, q.z, q.w),
            "frame_id": msg.header.frame_id or "map",
            "timestamp": int(msg.header.stamp.sec * 1000 + msg.header.stamp.nanosec / 1_000_000),
        }

    def _on_tracking(self, msg: String) -> None:
        self.last_tracking = msg.data

    def _on_compute(self, msg: Float64) -> None:
        if self.last_tracking.strip().upper() == "OK":
            self.last_compute_ms = float(msg.data)

    def _on_drift_risk(self, msg: Float64) -> None:
        self.pipeline_payload["drift_risk_score"] = round(float(msg.data), 4)

    def _on_drift_error(self, msg: Float64) -> None:
        self.pipeline_payload["drift_predicted_error"] = round(float(msg.data), 4)

    def _on_imu_enabled(self, msg: Bool) -> None:
        self.pipeline_payload["use_imu"] = bool(msg.data)

    def _on_drift_enabled(self, msg: Bool) -> None:
        self.pipeline_payload["use_drift_correction"] = bool(msg.data)

    def _build_slam_payload(self) -> dict:
        slam_status = tracking_to_slam_status(self.last_tracking)
        tracking_ok = self.last_tracking.strip().upper() == "OK"
        latency_ms = round(self.last_compute_ms, 1) if tracking_ok else 0.0
        fps = 0.0
        if tracking_ok and self.last_compute_ms > 0.0:
            fps = round(1000.0 / self.last_compute_ms, 1)
        return {
            "topic": "/slam/status",
            "slam_status": slam_status,
            "tracking_loss": slam_status == "tracking_loss",
            "recently_loss": slam_status == "recently_loss",
            "latency_ms": latency_ms,
            "fps": fps,
            "timestamp": int(self.get_clock().now().nanoseconds / 1_000_000),
        }

    def _post_json(self, path: str, payload: dict) -> None:
        body = json.dumps(payload).encode("utf-8")
        request = urllib.request.Request(
            f"{self.backend_url}{path}",
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=1.5) as response:
                response.read()
        except urllib.error.URLError as exc:
            self.get_logger().warning(f"Backend ingest failed ({path}): {exc}")

    def _publish_to_backend(self) -> None:
        tracking = self.last_tracking.strip().upper()
        if self.pose_payload is not None and tracking in ("OK", "RECENTLY_LOST"):
            pose = dict(self.pose_payload)
            pose["status"] = "live" if tracking == "OK" else "stale"
            self._post_json("/api/ingest/pose", pose)
        elif self.pose_payload is not None and tracking == "LOST":
            pose = dict(self.pose_payload)
            pose["status"] = "stale"
            self._post_json("/api/ingest/pose", pose)
        self.slam_payload = self._build_slam_payload()
        self._post_json("/api/ingest/slam", self.slam_payload)
        self._post_json("/api/ingest/pipeline", dict(self.pipeline_payload))


def main() -> None:
    rclpy.init(args=sys.argv)
    node = RosBackendBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
