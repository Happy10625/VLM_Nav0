"""Remove sparse XY returns before they reach SLAM and Nav2 costmaps."""

import json
import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String


def filter_sparse_obstacle_points(
    points,
    resolution,
    footprint_length,
    footprint_width,
    min_obstacle_points,
    min_height=-math.inf,
    max_height=math.inf,
    max_range=math.inf,
):
    """Return a mask that rejects obstacle returns with fewer than N neighbours.

    Neighbours are counted as distinct XY cells in a body-aligned rectangle the
    size of the robot footprint. Points outside the configured obstacle height
    and range are passed through unchanged because downstream consumers apply
    their own height/range limits.
    """
    xyz = np.asarray(points, dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] < 3:
        raise ValueError("points must be an Nx3 array")
    resolution = float(resolution)
    length = float(footprint_length)
    width = float(footprint_width)
    threshold = int(min_obstacle_points)
    if resolution <= 0.0:
        raise ValueError("filter resolution must be positive")
    if length <= 0.0 or width <= 0.0:
        raise ValueError("footprint dimensions must be positive")
    if threshold < 1:
        raise ValueError("minimum obstacle-point count must be at least one")
    if float(max_height) < float(min_height):
        raise ValueError("maximum obstacle height cannot be below minimum height")
    if float(max_range) <= 0.0:
        raise ValueError("maximum filter range must be positive")

    keep = np.ones(xyz.shape[0], dtype=bool)
    finite = np.all(np.isfinite(xyz[:, :3]), axis=1)
    candidates = (
        finite
        & (xyz[:, 2] >= float(min_height))
        & (xyz[:, 2] <= float(max_height))
        & (np.hypot(xyz[:, 0], xyz[:, 1]) <= float(max_range))
    )
    if threshold <= 1 or not np.any(candidates):
        return keep, 0

    candidate_indices = np.flatnonzero(candidates)
    candidate_xy = xyz[candidates, :2]
    cell_xy = np.floor(candidate_xy / resolution).astype(np.int64)
    minimum = cell_xy.min(axis=0)
    local_xy = cell_xy - minimum
    grid_width = int(local_xy[:, 0].max()) + 1
    grid_height = int(local_xy[:, 1].max()) + 1
    occupied = np.zeros((grid_height, grid_width), dtype=np.float32)
    occupied[local_xy[:, 1], local_xy[:, 0]] = 1.0

    # Cell centres must remain inside the physical half extents; rounding up
    # would silently make the noise window larger than the robot body.
    half_x = max(0, int(math.floor(0.5 * length / resolution + 1e-9)))
    half_y = max(0, int(math.floor(0.5 * width / resolution + 1e-9)))
    counts = cv2.boxFilter(
        occupied,
        ddepth=-1,
        ksize=(2 * half_x + 1, 2 * half_y + 1),
        normalize=False,
        borderType=cv2.BORDER_CONSTANT,
    )
    local_counts = counts[local_xy[:, 1], local_xy[:, 0]]
    rejected_indices = candidate_indices[local_counts < float(threshold)]
    keep[rejected_indices] = False
    return keep, int(rejected_indices.size)


class SparseObstacleFilter(Node):
    """Publish the point cloud used by both SLAM and Nav2 after denoising."""

    def __init__(self):
        super().__init__("sparse_obstacle_filter")
        defaults = {
            "input_topic": "/cloud_registered_body",
            "output_topic": "/vlm_nav/filtered_obstacle_cloud",
            "enabled": True,
            "filter_resolution": 0.05,
            "footprint_length": 0.72,
            "footprint_width": 0.50,
            "min_obstacle_points": 3,
            "min_obstacle_height": 0.05,
            "max_obstacle_height": 1.50,
            "max_filter_range": 20.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.publisher = self.create_publisher(
            PointCloud2,
            str(self.get_parameter("output_topic").value),
            qos_profile_sensor_data,
        )
        self.status_publisher = self.create_publisher(
            String, "~/status", qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("input_topic").value),
            self.on_cloud,
            qos_profile_sensor_data,
        )
        self.received_clouds = 0
        self.last_input_points = 0
        self.last_removed_points = 0

    def on_cloud(self, message):
        self.received_clouds += 1
        try:
            records = point_cloud2.read_points(message, skip_nans=False)
            names = records.dtype.names or ()
            if not all(name in names for name in ("x", "y", "z")):
                raise ValueError("PointCloud2 must contain x, y, and z fields")
            xyz = np.column_stack((records["x"], records["y"], records["z"]))
            self.last_input_points = int(records.size)
            if bool(self.get_parameter("enabled").value):
                keep, removed = filter_sparse_obstacle_points(
                    xyz,
                    self.get_parameter("filter_resolution").value,
                    self.get_parameter("footprint_length").value,
                    self.get_parameter("footprint_width").value,
                    self.get_parameter("min_obstacle_points").value,
                    self.get_parameter("min_obstacle_height").value,
                    self.get_parameter("max_obstacle_height").value,
                    self.get_parameter("max_filter_range").value,
                )
            else:
                keep = np.ones(records.size, dtype=bool)
                removed = 0
        except (AssertionError, ValueError, TypeError) as error:
            self.get_logger().error(f"Cannot filter obstacle cloud: {error}")
            return

        selected = records[keep]
        output = PointCloud2()
        output.header = message.header
        output.height = 1
        output.width = int(selected.size)
        output.fields = message.fields
        output.is_bigendian = message.is_bigendian
        output.point_step = message.point_step
        output.row_step = output.point_step * output.width
        output.data = selected.tobytes()
        output.is_dense = message.is_dense
        self.last_removed_points = int(removed)
        self.publisher.publish(output)

        status = String()
        status.data = json.dumps(
            {
                "received_clouds": self.received_clouds,
                "input_points": self.last_input_points,
                "removed_points": self.last_removed_points,
                "output_points": self.last_input_points - self.last_removed_points,
                "footprint_length": float(
                    self.get_parameter("footprint_length").value
                ),
                "footprint_width": float(
                    self.get_parameter("footprint_width").value
                ),
                "min_obstacle_points": int(
                    self.get_parameter("min_obstacle_points").value
                ),
            },
            ensure_ascii=False,
        )
        self.status_publisher.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = SparseObstacleFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
