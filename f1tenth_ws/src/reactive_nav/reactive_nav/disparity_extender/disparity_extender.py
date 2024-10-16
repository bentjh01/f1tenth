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
        self.mean_front_range = 0 #[m]
        self.angle_increment = 3.142/1024 #[rad]

    def load_config(self):
        c = Config()
        self.diparity_trigger = c.disparity_trigger
        self.safety_bubble_diameter = c.safety_bubble_diameter
        self.range_cutoff = c.range_cutoff

    def cutoff_range(self):
        if self.do_range_cutoff:
            self.ranges[self.ranges > self.range_cutoff] = self.range_cutoff
    
    def get_index_count(self, diameter, range, angle_increment):
        arc = range * angle_increment
        return int(diameter/arc)
        
    def update(self, scan_msg):
        self.ranges = scan_msg.ranges

        # mean front distance
        front_index = int(len(self.ranges)//2)
        front_range = self.ranges[front_index]
        index_count = self.get_index_count(self.safety_bubble_diameter, front_range, self.angle_increment)
        mean_front_range = np.mean(self.ranges[front_index-index_count//2:front_index+index_count//2])

        # Min Filter
        unmodified_ranges = self.ranges.copy()
        for i, r in enumerate(self.ranges):
            index_count = self.get_index_count(self.safety_bubble_diameter, r, self.angle_increment)
            radius_count = int(index_count//2)
            range_count = self.ranges.shape[0]
            if (radius_count <= i < (range_count-radius_count)):
                min_range = np.min(unmodified_ranges[i-radius_count:i+radius_count])
            elif (i < radius_count):
                min_range = np.min(unmodified_ranges[:i+radius_count])
            else:
                min_range = np.min(unmodified_ranges[i-radius_count:])
            self.ranges[i] = min_range

        speed = mean_front_range/range_cutoff * self.max_speed



class DisparityExtenderNode(Node):
    def __init__():
        super.__init__()