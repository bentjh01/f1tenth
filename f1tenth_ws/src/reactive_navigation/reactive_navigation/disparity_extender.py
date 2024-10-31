import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist

config = {
    "safety_bubble_diameter": 0.8, # [m]
    "range_cutoff": 5, # [m]
    "disparity_threshold": 0.6, # [m]
    "linear_x_proportional_gain": 1,
    "angular_z_proportional_gain": 0.6,
    "max_speed": 8.0, # [m/s]
    "publish_rate": 100, # [Hz]
    "drive_topic": "/drive",
    "ranges_topic": "/scan"
}

class DisparityExtender:
    def __init__(self):
        self.safety_bubble_diameter = config["safety_bubble_diameter"]
        self.range_cutoff = config["range_cutoff"]
        self.disparity_threshold = config["disparity_threshold"]
        self.max_speed = config["max_speed"]
        self.linear_x_proportional_gain = config["linear_x_proportional_gain"]
        self.angular_z_proportional_gain = config["angular_z_proportional_gain"]

    def update(self, ranges, angle_increment):
        # mean front distance
        ranges = np.array(ranges)
        front_index = int(ranges.shape[0]//2)
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
                else:
                    i -= 1
                    r = ranges[i]
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
        marked_point = [lower_bound, upper_bound, 0]
        marked_indexes.append(marked_point)
        
        ### APPLY MARKS ###
        marked_indexes.sort(reverse=True, key=lambda x: x[2])
        for mark_point in marked_indexes:
            ranges[mark_point[0]:mark_point[1]] = mark_point[2]

        # select max range
        max_index = np.argmax(ranges)
        goal_bearing = angle_increment * (max_index - ranges.shape[0]//2)

        angular_z = float(self.angular_z_proportional_gain * goal_bearing)
        linear_x = float(min(mean_front_range/self.range_cutoff * self.linear_x_proportional_gain,1) * self.max_speed)

        return linear_x, angular_z


class DisparityExtenderNode(Node):
    def __init__(self):
        super().__init__('disparity_extender')

        self.publish_rate = config["publish_rate"]
        self.drive_topic = config["drive_topic"]
        self.laser_topic = config["ranges_topic"]

        self.de = DisparityExtender()

        self.scan_subscriber = self.create_subscription(LaserScan, self.laser_topic, self.scan_callback, 10)
        self.scan_subscriber

        self.drive_publisher = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
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