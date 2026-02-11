#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, PointCloud2


class LidarMonitor(Node):
    def __init__(self):
        super().__init__('lidar_monitor')

        # Parameters so you can override at runtime:
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cloud_topic', '/points')
        self.declare_parameter('print_every_sec', 1.0)   # throttle prints

        self.scan_topic = self.get_parameter('scan_topic').value
        self.cloud_topic = self.get_parameter('cloud_topic').value
        self.print_every_sec = float(self.get_parameter('print_every_sec').value)

        self._last_print_scan = 0.0
        self._last_print_cloud = 0.0

        # Subscribe to both; it's fine if one topic doesn't exist.
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.on_scan,
            qos_profile_sensor_data
        )

        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.cloud_topic,
            self.on_cloud,
            qos_profile_sensor_data
        )

        self.get_logger().info(
            f"Listening for LaserScan on '{self.scan_topic}' and PointCloud2 on '{self.cloud_topic}'."
        )
        self.get_logger().info("Tip: override topics with --ros-args -p scan_topic:=/your_scan_topic")

    def on_scan(self, msg: LaserScan):
        now = time.time()
        if (now - self._last_print_scan) < self.print_every_sec:
            return
        self._last_print_scan = now

        # Basic stats (avoid printing full ranges array)
        ranges = msg.ranges
        valid = [r for r in ranges if (not math.isinf(r)) and (not math.isnan(r)) and r > 0.0]
        n_total = len(ranges)
        n_valid = len(valid)

        if n_valid > 0:
            r_min = min(valid)
            r_max = max(valid)
            r_avg = sum(valid) / n_valid
        else:
            r_min = r_max = r_avg = float('nan')

        hz = 0.0
        if msg.scan_time > 0.0:
            hz = 1.0 / msg.scan_time

        self.get_logger().info(
            f"[LaserScan] frame={msg.header.frame_id or '(none)'} "
            f"points={n_total} valid={n_valid} "
            f"range(m): min={r_min:.2f} avg={r_avg:.2f} max={r_max:.2f} "
            f"angle(deg): [{math.degrees(msg.angle_min):.1f}, {math.degrees(msg.angle_max):.1f}] "
            f"scan_time={msg.scan_time:.3f}s (~{hz:.1f} Hz)"
        )

        # Optional: show first few samples (useful if you want to see changing values)
        preview = []
        for r in ranges[:10]:
            if math.isinf(r):
                preview.append("inf")
            elif math.isnan(r):
                preview.append("nan")
            else:
                preview.append(f"{r:.2f}")
        self.get_logger().info(f"[LaserScan] first10 ranges: {', '.join(preview)}")

    def on_cloud(self, msg: PointCloud2):
        now = time.time()
        if (now - self._last_print_cloud) < self.print_every_sec:
            return
        self._last_print_cloud = now

        # Number of points is width * height for organized cloud; many lidars publish height=1
        n_points = int(msg.width) * int(msg.height)

        self.get_logger().info(
            f"[PointCloud2] frame={msg.header.frame_id or '(none)'} "
            f"width={msg.width} height={msg.height} points={n_points} "
            f"point_step={msg.point_step} row_step={msg.row_step} "
            f"fields={[f.name for f in msg.fields]}"
        )


def main():
    rclpy.init()
    node = LidarMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
