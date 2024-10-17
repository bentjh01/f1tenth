import rclpy
from rclpy import Node
import numpy as np
from sensor_msgs import LaserScan
from ackermann_msgs import AckermannDriveStamped
from geometry_msgs import Twist

class Config:
    def __init__(self):
        # Disparity Extender config
        self.safety_bubble_diameter = 0.6 #[m]
        self.range_cutoff = 5 #[m]
        self.disparity_threshold = 0.6 #[m]
        self.speed_proportional_gain = 1
        self.steering_proportional_gain = 1
        self.max_speed = 10 #[m/s]
        # ROS2 Config
        self.publish_rate = 100 #[Hz]
        self.drive_topic = "/drive"
        self.ranges_topic = "/scan"

class DisparityExtender:
    def __init__(self):
        c = Config()
        self.safety_bubble_diameter = c.safety_bubble_diameter
        self.range_cutoff = c.range_cutoff
        self.disparity_threshold = c.disparity_threshold
        self.max_speed = c.max_speed
        self.speed_proportional_gain = c.speed_proportional_gain
        self.steering_proportional_gain = c.steering_proportional_gain

    def update(self, ranges, angle_increment):
        # mean front distance
        front_index = int(len(ranges)//2)
        front_range = ranges[front_index]
        arc = front_range * angle_increment
        index_count = int(self.safety_bubble_diameter/arc/2)
        mean_front_range = np.mean(ranges[front_index-index_count:front_index+index_count+1])

        marked_indexes = []
        # find disparity
        for i in range(1, ranges.shape[0]):
            if abs(ranges[i] - ranges[i-1]) > self.disparity_threshold:
                if ranges[i] < ranges[i-1]:
                    r = ranges[i]
                    arc = angle_increment * r
                else:
                    i -= i
                    r = ranges[i-1]
                arc = angle_increment * r
                radius_count = int(self.safety_bubble_diameter/arc/2)
                lower_bound = i-radius_count 
                upper_bound = i+radius_count+1
                marked_point = [lower_bound, upper_bound, r]
                marked_indexes.append(marked_point)
        
        ### MARK MINIMUM ###
        minimum_range = np.min(ranges)
        minimum_index = np.argmin(ranges)
        arc = angle_increment * minimum_range
        radius_count = int(self.safety_bubble_diameter/arc/2)
        lower_bound = minimum_index-radius_count 
        upper_bound = minimum_index+radius_count+1
        marked_point = [lower_bound, upper_bound, r]
        marked_indexes.append(marked_point)
        
        ### APPLY MARKS ###
        marked_indexes.sort(reverse=True, key=lambda x: x[2])
        for mark_point in marked_indexes:
            ranges[mark_point[0]:mark_point[1]] = mark_point[2]

        # select max range
        max_index = np.argmax(ranges)
        goal_bearing = angle_increment * (max_index - ranges.shape[0]//2)

        steering = self.steering_proportional_gain * goal_bearing
        speed = min(mean_front_range/self.range_cutoff,1) * self.max_speed * self.speed_proportional_gain

        return speed, steering


class DisparityExtenderNode(Node):
    def __init__(self):
        super().__init__('disparity_extender')

        c = Config()
        self.publish_rate = c.publish_rate
        self.drive_topic = c.drive_topic
        self.laser_topic = c.ranges_topic

        self.de = DisparityExtender()

        self.scan_subscriber = self.create_subscription(LaserScan, self.laser_topic, self.scan_callback, 10)
        self.scan_subscriber

        self.drive_publisher = self.create_publisher(Twist, self.drive_topic, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        self.drive_msg = AckermannDriveStamped()

    def get_time(self):
        """
        Returns the current time in seconds
        """
        return self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9

    def scan_callback(self, msg):
        drive_output = self.de.update(msg.ranges, msg.angle_increment)
        self.drive_msg.drive.speed = drive_output[0]
        self.drive_msg.drive.steering_angle = drive_output[1]

    def timer_callback(self):
        self.drive_publisher.publish(self.drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DisparityExtenderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()