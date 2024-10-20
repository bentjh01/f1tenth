import rclpy
from rclpy import Node
import numpy as np
from sensor_msgs import LaserScan
from ackermann_msgs import AckermannDriveStamped

class Config:
    def __init__(self):
        # Disparity Extender config
        self.linear_x_proportional_gain = 1.0
        self.angular_z_proportional_gain = 1.0
        self.safety_bubble_diameter = 0.6 #[m]
        self.range_cutoff = 5 #[m]
        # ROS2 Config
        self.publish_rate = 100 #[Hz]
        self.drive_topic = "/drive"
        self.scan_topic = "/scan"

class MinimumFilter:
    def __init__(self):
        config = Config()
        self.safety_bubble_diameter = config.safety_bubble_diameter
        self.range_cutoff = config.range_cutoff
        self.linear_x_proportional_gain = config.linear_x_proportional_gain
        self.angular_z_proportional_gain = config.angular_z_proportional_gain
        self.mean_front_range = 0 #[m]

    def update(self, ranges, angle_increment):
        # mean front distance
        front_index = int(ranges.shape[0]//2)
        front_range = ranges[front_index]
        arc = front_range * angle_increment
        index_count = int(self.safety_bubble_diameter//arc//2)
        mean_front_range = np.mean(self.ranges[front_index-index_count:front_index+index_count])

        # Min Filter
        unmodified_ranges = self.ranges.copy()
        for i, r in enumerate(self.ranges):
            arc = r * angle_increment
            radius_count = int(self.safety_bubble_diameter//arc//2)
            range_count = self.ranges.shape[0]
            if (radius_count <= i < (range_count-radius_count)):
                min_range = np.min(unmodified_ranges[i-radius_count:i+radius_count])
            elif (i < radius_count):
                min_range = np.min(unmodified_ranges[:i+radius_count])
            else:
                min_range = np.min(unmodified_ranges[i-radius_count:])
            self.ranges[i] = min_range
        
        # select max range
        max_index = np.argmax(self.ranges)
        goal_bearing = self.angle_increment * (max_index - self.ranges.shape[0]//2)

        steering = goal_bearing
        speed = min(mean_front_range/self.range_cutoff,1) * self.max_speed
        return speed, steering

class MinimumFilterNode(Node):
    def __init__(self):
        super.__init__("minimum_filter")
        config = Config()

        self.minimum_filter = MinimumFilter()

        self.cmd_vel_publisher = self.create_publisher(AckermannDriveStamped, config.drive_topic, 10)
        self.cmd_vel_timer = self.create_timer(1.0/config.publish_rate, self.timer_callback)
        self.drive_msg = AckermannDriveStamped()

        self.scan_subscriber = self.create_subscriber(LaserScan, config.scan_topic, self.scan_callback, 10)
        self.scan_subscriber

    def timer_callback(self):
        self.cmd_vel_publisher.publish(self.drive_msg)

    def scan_callback(self, msg):
        drive_data = self.minimum_filter.update(msg.ranges, msg.angle_increment)
        self.drive_msg.speed = drive_data[0]
        self.drive_msg.steering = drive_data[1]

def main(args):
    rclpy.init(args = args)
    node = MinimumFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
