import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


def test_filter_stays_alive_while_waiting_for_cloud_transform(tmp_path, monkeypatch):
    executable = (
        Path(get_package_prefix("vlm_nav"))
        / "lib"
        / "vlm_nav"
        / "obstacle_cloud_filter"
    )
    topic = f"/vlm_nav/test/filter_input_{os.getpid()}"
    ros_log_dir = tmp_path / "ros_logs"
    ros_log_dir.mkdir()
    monkeypatch.setenv("ROS_LOG_DIR", str(ros_log_dir))
    environment = os.environ.copy()
    process = subprocess.Popen(
        [
            str(executable),
            "--ros-args",
            "-r",
            "__node:=obstacle_cloud_filter_runtime_test",
            "-p",
            f"input_topic:={topic}",
            "-p",
            "output_topic:=/vlm_nav/test/filter_output",
            "-p",
            "target_frame:=base_link",
        ],
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )

    rclpy.init()
    publisher_node = rclpy.create_node("obstacle_cloud_filter_runtime_test_publisher")
    publisher = publisher_node.create_publisher(
        PointCloud2, topic, qos_profile_sensor_data
    )
    try:
        discovery_deadline = time.monotonic() + 5.0
        while publisher.get_subscription_count() == 0:
            if process.poll() is not None or time.monotonic() >= discovery_deadline:
                output = process.communicate(timeout=1.0)[0]
                raise AssertionError(
                    "filter subscriber was not discovered before process exit/timeout:\n"
                    + output
                )
            rclpy.spin_once(publisher_node, timeout_sec=0.05)

        cloud = PointCloud2()
        cloud.header.frame_id = "body"
        cloud.header.stamp = publisher_node.get_clock().now().to_msg()
        publisher.publish(cloud)

        survival_deadline = time.monotonic() + 1.0
        while process.poll() is None and time.monotonic() < survival_deadline:
            rclpy.spin_once(publisher_node, timeout_sec=0.05)

        if process.poll() is not None:
            output = process.communicate(timeout=1.0)[0]
            raise AssertionError(
                f"filter exited after receiving a cloud (code {process.returncode}):\n"
                + output
            )
    finally:
        publisher_node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            process.communicate(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.communicate(timeout=5.0)
