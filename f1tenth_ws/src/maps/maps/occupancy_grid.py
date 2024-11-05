import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist

config = {
    "publish_rate": 100, # [Hz]
    "ranges_topic": "/scan"
}

class OccupancyGrid:
    def __init__(self):
        self.map = None

    def update(self, ranges, angle_increment):
        # mean front distance
        ranges = np.array(ranges)

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