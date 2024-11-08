import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

class OccupancyGrid:
    def __init__(self):
        self.grid_dimensions = (64, 64)

    def grid_update(self, scan_msg):
        # update the occupancy grid with the new scan data
        ranges = np.array(scan_msg.ranges)
        for i, r in enumerate(ranges):
            x = r * np.cos(scan_msg.angle_min + scan_msg.angle_increment * i)
            y = r * np.sin(scan_msg.angle_min + scan_msg.angle_increment * i)

class OccupancyGridNode(Node):
    def __init__(self):
        super().__init__('occupancy_grid')
