"""Filter isolated lethal costmap cells for Nav2 behavior collision checks."""

import copy
import json
import math

import cv2
import numpy as np
import rclpy
from nav2_msgs.msg import Costmap
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


LETHAL_OBSTACLE = 254
FREE_SPACE = 0


def filter_sparse_lethal_cells(
    costs,
    resolution,
    neighborhood_radius,
    min_occupied_cells,
    lethal_cost=LETHAL_OBSTACLE,
):
    """Remove lethal cells whose circular neighborhood contains too few lethal cells.

    The returned array is a copy. Unknown cells and non-lethal inflation costs are
    preserved. The second return value is the number of lethal cells removed.
    """
    grid = np.asarray(costs, dtype=np.uint8)
    if grid.ndim != 2:
        raise ValueError("cost grid must be two-dimensional")
    resolution = float(resolution)
    radius = float(neighborhood_radius)
    threshold = int(min_occupied_cells)
    if resolution <= 0.0:
        raise ValueError("costmap resolution must be positive")
    if radius < 0.0:
        raise ValueError("neighborhood radius cannot be negative")
    if threshold < 1:
        raise ValueError("minimum occupied-cell count must be at least one")

    filtered = grid.copy()
    lethal = grid == int(lethal_cost)
    if threshold <= 1 or not np.any(lethal):
        return filtered, 0

    radius_cells = max(0, int(math.ceil(radius / resolution)))
    offsets = np.arange(-radius_cells, radius_cells + 1, dtype=np.float32)
    yy, xx = np.meshgrid(offsets, offsets, indexing="ij")
    metric_distance_squared = (xx * resolution) ** 2 + (yy * resolution) ** 2
    kernel = (metric_distance_squared <= radius * radius + 1e-9).astype(np.float32)
    if not np.any(kernel):
        kernel[radius_cells, radius_cells] = 1.0

    neighbor_counts = cv2.filter2D(
        lethal.astype(np.float32),
        ddepth=-1,
        kernel=kernel,
        borderType=cv2.BORDER_CONSTANT,
    )
    sparse = lethal & (neighbor_counts < float(threshold))
    removed = int(np.count_nonzero(sparse))
    filtered[sparse] = FREE_SPACE
    return filtered, removed


class SparseCostmapFilter(Node):
    """Publish a sparse-noise-tolerant copy of the local Nav2 costmap."""

    def __init__(self):
        super().__init__("sparse_costmap_filter")
        defaults = {
            "input_topic": "/local_costmap/costmap_raw",
            "output_topic": "/vlm_nav/behavior_costmap_raw",
            "enabled": True,
            "neighborhood_radius": 0.20,
            "min_occupied_cells": 3,
            "lethal_cost": LETHAL_OBSTACLE,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        output_topic = str(self.get_parameter("output_topic").value)
        input_topic = str(self.get_parameter("input_topic").value)
        self.filtered_pub = self.create_publisher(Costmap, output_topic, qos)
        self.status_pub = self.create_publisher(String, "~/status", qos)
        self.create_subscription(Costmap, input_topic, self.on_costmap, qos)

        self.received_maps = 0
        self.filtered_maps = 0
        self.last_lethal_cells = 0
        self.last_removed_cells = 0
        self.get_logger().info(
            "Sparse costmap filter: "
            f"{input_topic} -> {output_topic}, "
            f"radius={float(self.get_parameter('neighborhood_radius').value):.2f}m, "
            f"minimum_cells={int(self.get_parameter('min_occupied_cells').value)}"
        )

    def publish_status(self, resolution):
        message = String()
        message.data = json.dumps(
            {
                "received_maps": self.received_maps,
                "filtered_maps": self.filtered_maps,
                "resolution": float(resolution),
                "neighborhood_radius": float(
                    self.get_parameter("neighborhood_radius").value
                ),
                "min_occupied_cells": int(
                    self.get_parameter("min_occupied_cells").value
                ),
                "lethal_cells_before": self.last_lethal_cells,
                "lethal_cells_removed": self.last_removed_cells,
                "lethal_cells_after": (
                    self.last_lethal_cells - self.last_removed_cells
                ),
            },
            ensure_ascii=False,
        )
        self.status_pub.publish(message)

    def on_costmap(self, message):
        width = int(message.metadata.size_x)
        height = int(message.metadata.size_y)
        expected = width * height
        self.received_maps += 1
        if width <= 0 or height <= 0 or len(message.data) != expected:
            self.get_logger().error(
                "Ignoring malformed raw costmap: "
                f"{width}x{height}, data={len(message.data)}"
            )
            return

        grid = np.asarray(message.data, dtype=np.uint8).reshape(height, width)
        lethal_cost = int(self.get_parameter("lethal_cost").value)
        self.last_lethal_cells = int(np.count_nonzero(grid == lethal_cost))
        if bool(self.get_parameter("enabled").value):
            try:
                filtered, removed = filter_sparse_lethal_cells(
                    grid,
                    float(message.metadata.resolution),
                    float(self.get_parameter("neighborhood_radius").value),
                    int(self.get_parameter("min_occupied_cells").value),
                    lethal_cost,
                )
            except ValueError as error:
                self.get_logger().error(f"Cannot filter raw costmap: {error}")
                return
        else:
            filtered = grid.copy()
            removed = 0

        output = copy.deepcopy(message)
        output.data = filtered.reshape(-1).tolist()
        self.last_removed_cells = int(removed)
        self.filtered_maps += 1
        self.filtered_pub.publish(output)
        self.publish_status(message.metadata.resolution)


def main(args=None):
    rclpy.init(args=args)
    node = SparseCostmapFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
