import rclpy
from rclpy import Node
import numpy as np
from sensor_msgs import LaserScan

class Config:
    def __init__():
        # Disparity Extender config
        self.safety_bubble_diameter = 0.6 #[m]
        self.range_cutoff = 5 #[m]
        self.disparity_trigger = 0.6 #[m]
        # ROS2 Config
        self.publish_rate = 100 #[Hz]
        self.drive_topic = "/drive"
        self.ranges_topic = "/scan"

class DisparityExtender:
    def __init__:
        self.load_config()
        self.ranges = LaserScan()

    def load_config(self):
        c = Config()
        self.diparity_trigger = c.disparity_trigger
        self.safety_bubble_diameter = c.safety_bubble_diameter
        self.range_cutoff = c.range_cutoff

    def cutoff_range(self):
        
        self.ranges_topic(<=self.cutoff_range)

    def 


class DisparityExtenderNode(Node):
    def __init__():
        super.__init__()