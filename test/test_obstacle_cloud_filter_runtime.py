import os
from pathlib import Path
import signal
import subprocess
import time
import struct

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


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


def test_confirmation_waits_three_cycles_in_fixed_frame_and_keeps_clearing(tmp_path, monkeypatch):
    executable = Path(get_package_prefix('vlm_nav')) / 'lib/vlm_nav/obstacle_cloud_filter'
    topic = f'/vlm_nav/test/confirmation_{os.getpid()}'
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    process = subprocess.Popen([
        str(executable), '--ros-args', '-r', '__node:=confirmation_runtime_test',
        '-p', f'input_topic:={topic}/input', '-p', f'output_topic:={topic}/clearing',
        '-p', f'confirmed_topic:={topic}/confirmed', '-p', 'confirmation_frame:=odom',
        '-p', 'confirmation_cycles:=3', '-p', 'confirmation_period:=0.8',
    ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('confirmation_runtime_test_driver')
    publisher = node.create_publisher(PointCloud2, topic+'/input', qos_profile_sensor_data)
    broadcaster = TransformBroadcaster(node)
    received = {'clearing': {}, 'confirmed': {}}
    subscriptions = []
    for key in received:
        subscriptions.append(node.create_subscription(PointCloud2, topic+'/'+key,
            lambda m, k=key: received[k].update({m.header.stamp.sec*10**9+m.header.stamp.nanosec: m}),
            qos_profile_sensor_data))

    def wait_for(predicate):
        deadline = time.monotonic()+5
        while not predicate():
            assert process.poll() is None, 'filter exited unexpectedly'
            assert time.monotonic() < deadline, 'timed out waiting for point-cloud output'
            rclpy.spin_once(node, timeout_sec=.02)

    try:
        wait_for(lambda: publisher.get_subscription_count() > 0)
        # Preload timestamped robot motion so the filter can use exact-time odom TF.
        frames = []
        for i, seconds in enumerate([1., 1.4, 1.8, 2.2, 2.6, 3., 3.39, 3.4, 5.8]):
            ns = round(seconds*1e9)
            tf = TransformStamped()
            tf.header.frame_id = 'odom'; tf.child_frame_id = 'base_link'
            tf.header.stamp.sec, tf.header.stamp.nanosec = divmod(ns, 10**9)
            tf.transform.translation.x = i*.1
            tf.transform.rotation.w = 1.
            frames.append((ns, tf, 2.01-i*.1))
        for _ in range(5):
            broadcaster.sendTransform([tf for _, tf, _ in frames])
            rclpy.spin_once(node, timeout_sec=.05)
        for i, (ns, tf, x) in enumerate(frames):
            cloud = PointCloud2()
            cloud.header.frame_id = 'base_link'; cloud.header.stamp = tf.header.stamp
            cloud.height = cloud.width = 1
            cloud.fields = [PointField(name=n, offset=j*4, datatype=PointField.FLOAT32, count=1)
                            for j, n in enumerate(('x', 'y', 'z'))]
            cloud.point_step = cloud.row_step = 12
            cloud.data = struct.pack('<fff', x, .01, .3 if i % 2 else 1.1)
            publisher.publish(cloud)
            wait_for(lambda: ns in received['clearing'] and ns in received['confirmed'])
            assert received['clearing'][ns].width == 1
            confirmed = received['confirmed'][ns]
            assert confirmed.width == (1 if i == 7 else 0)
            if confirmed.width:
                assert confirmed.header.frame_id == 'odom'
                offset = next(f.offset for f in confirmed.fields if f.name == 'x')
                assert abs(struct.unpack_from('<f', bytes(confirmed.data), offset)[0]-2.01) < 1e-5
        # Normal TF delivery lag must queue the scan, not reset the confirmation history.
        late_ns = 7*10**9
        cloud.header.stamp.sec, cloud.header.stamp.nanosec = 7, 0
        publisher.publish(cloud)
        until = time.monotonic()+.15
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.02)
        assert late_ns not in received['clearing']
        tf.header.stamp = cloud.header.stamp
        broadcaster.sendTransform(tf)
        wait_for(lambda: late_ns in received['clearing'] and late_ns in received['confirmed'])
        assert received['clearing'][late_ns].width == 1
    finally:
        node.destroy_node(); rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            process.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.communicate(timeout=5)
