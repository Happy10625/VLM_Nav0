#!/usr/bin/env python3
"""Probe one live ROS RGB frame and benchmark the configured Qwen VLM."""

import argparse
import base64
import statistics
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from vlm_nav.vlm_client import OpenAICompatibleVLMClient
from vlm_nav.exploration import render_frontier_map, scan_montage
from vlm_nav.models import FrontierCandidate


MAX_BASE64_BYTES = 10 * 1024 * 1024
MAX_ASPECT_RATIO = 200.0


def receive_frame(topic: str, wait_seconds: float):
    node = rclpy.create_node("qwen_vlm_latency_probe")
    received = []
    subscription = node.create_subscription(
        Image,
        topic,
        lambda message: received.append(message) if not received else None,
        qos_profile_sensor_data,
    )
    deadline = time.monotonic() + wait_seconds
    while not received and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not received:
        node.destroy_subscription(subscription)
        node.destroy_node()
        raise RuntimeError(
            f"No RGB frame received from {topic!r} within {wait_seconds:.1f}s"
        )
    message = received[0]
    rgb = CvBridge().imgmsg_to_cv2(message, desired_encoding="rgb8")
    node.destroy_subscription(subscription)
    node.destroy_node()
    return message, rgb


def encoded_size(rgb, jpeg_quality: int):
    ok, encoded = cv2.imencode(
        ".jpg",
        cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_RGB2BGR),
        [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality],
    )
    if not ok:
        raise RuntimeError("JPEG encoding failed")
    return encoded.nbytes, len(base64.b64encode(encoded.tobytes()))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/camera/color/image_raw")
    parser.add_argument("--target", default="chair")
    parser.add_argument("--samples", type=int, default=3)
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument(
        "--max-latency",
        type=float,
        default=None,
        help=(
            "fail when any successful VLM response takes longer than this many "
            "seconds; omitted means report only"
        ),
    )
    parser.add_argument("--wait-for-frame", type=float, default=10.0)
    parser.add_argument("--jpeg-quality", type=int, default=85)
    parser.add_argument(
        "--mode",
        choices=("target", "frontier", "both"),
        default="target",
        help="benchmark one-image target inference, two-image frontier inference, or both",
    )
    args = parser.parse_args()
    if args.samples < 1:
        parser.error("--samples must be at least 1")
    if args.max_latency is not None and args.max_latency <= 0.0:
        parser.error("--max-latency must be greater than 0")

    rclpy.init()
    try:
        message, rgb = receive_frame(args.topic, args.wait_for_frame)
    finally:
        rclpy.shutdown()

    jpeg_bytes, base64_bytes = encoded_size(rgb, args.jpeg_quality)
    aspect_ratio = max(message.width / message.height, message.height / message.width)
    compatible = (
        message.width > 10
        and message.height > 10
        and aspect_ratio <= MAX_ASPECT_RATIO
        and base64_bytes <= MAX_BASE64_BYTES
    )
    print(
        "frame: "
        f"{message.width}x{message.height} {message.encoding}, "
        f"raw={len(message.data)} B, jpeg={jpeg_bytes} B, "
        f"base64={base64_bytes} B"
    )
    print(f"qwen_image_constraints: {'PASS' if compatible else 'FAIL'}")
    if not compatible:
        raise RuntimeError("The camera image does not satisfy Qwen image limits")

    client = OpenAICompatibleVLMClient(
        timeout_s=args.timeout,
        image_detail="low",
        jpeg_quality=args.jpeg_quality,
    )
    print(f"model: {client.model}")
    benchmark_cases = []
    if args.mode in ("target", "both"):
        benchmark_cases.append(
            ("target", lambda: client.infer(rgb, args.target))
        )
    if args.mode in ("frontier", "both"):
        montage = scan_montage([rgb] * 8, [index * 0.785398 for index in range(8)])
        grid = np.zeros((140, 140), dtype=np.int16)
        grid[:15, :] = -1
        grid[:, :15] = -1
        grid[-15:, :] = -1
        grid[:, -15:] = -1
        candidates = (
            FrontierCandidate(1, 2.2, 0.0, 0.0, 2.2, 18),
            FrontierCandidate(2, 0.0, 2.2, 1.5708, 2.2, 24),
            FrontierCandidate(3, -2.2, 0.0, 3.1416, 2.2, 16),
        )
        map_image = render_frontier_map(
            grid, (-3.5, -3.5), 0.05, (0.0, 0.0, 0.0),
            (0.0, 0.0), 3.0, candidates,
        )
        benchmark_cases.append(
            (
                "frontier",
                lambda: client.infer_frontier(
                    montage, map_image, candidates, args.target
                ),
            )
        )

    total_failures = 0
    total_latency_violations = 0
    for case_name, invoke in benchmark_cases:
        elapsed_samples = []
        failures = 0
        latency_violations = 0
        for index in range(1, args.samples + 1):
            started = time.perf_counter()
            try:
                result = invoke()
            except Exception as error:
                elapsed = time.perf_counter() - started
                failures += 1
                print(
                    f"{case_name} {index}/{args.samples}: FAIL {elapsed:.3f}s "
                    f"{type(error).__name__}: {error}"
                )
            else:
                elapsed = time.perf_counter() - started
                elapsed_samples.append(elapsed)
                within_limit = (
                    args.max_latency is None or elapsed <= args.max_latency
                )
                if not within_limit:
                    latency_violations += 1
                detail = (
                    f"selected={result.selected_frontier_id}"
                    if case_name == "frontier"
                    else f"visible={result.target_visible}"
                )
                print(
                    f"{case_name} {index}/{args.samples}: "
                    f"{'OK' if within_limit else 'SLOW'} {elapsed:.3f}s "
                    f"{detail} confidence={result.confidence:.3f}"
                )
        if elapsed_samples:
            print(
                f"{case_name}_latency_success: "
                f"min={min(elapsed_samples):.3f}s "
                f"mean={statistics.fmean(elapsed_samples):.3f}s "
                f"median={statistics.median(elapsed_samples):.3f}s "
                f"max={max(elapsed_samples):.3f}s"
            )
        print(
            f"{case_name}_summary: success={len(elapsed_samples)} "
            f"failure={failures} latency_violation={latency_violations} "
            f"total={args.samples}"
        )
        if args.max_latency is not None:
            print(
                f"{case_name}_latency_threshold: "
                f"max_allowed={args.max_latency:.3f}s "
                f"result={'PASS' if latency_violations == 0 else 'FAIL'}"
            )
        total_failures += failures
        total_latency_violations += latency_violations
    if total_failures or total_latency_violations:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
