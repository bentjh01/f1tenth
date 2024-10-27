import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
import numpy as np

# Bicycle model (https://www.researchgate.net/publication/354261242_Simulation_Performance_Evaluation_of_Pure_Pursuit_Stanley_LQR_MPC_Controller_for_Autonomous_Vehicles)
# steering_angle = np.arctan(wheel_base / turning_radius)
# turning_radius = wheel_base / np.tan(steering_angle)

config = {"twist_topic": "cmd_vel", 
          "ackermann_topic": "drive", 
          "wheel_base": 0.33,
          "header_frame_id": "base_link"}

class Twist2Ackermann(Node):
    def __init__(self):
        super().__init__('twist2ackermann')
        self.sub = self.create_subscription(Twist, config["twist_topic"], self.twist_callback, 10)
        self.pub = self.create_publisher(AckermannDriveStamped, config["ackermann_topic"], 10)
        self.get_logger().info('Twist2Ackermann node has been started.')

    def twist_callback(self, msg):
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = self.get_clock().now().to_msg()
        ack_msg.header.frame_id = config["header_frame_id"]
        if msg.angular.z == 0:
            ack_msg.drive.steering_angle = 0.0
        else:
            ack_msg.drive.steering_angle = np.arctan(config["wheel_base"] / msg.angular.z)
        ack_msg.drive.speed = msg.linear.x
        self.pub.publish(ack_msg)

def main():
    rclpy.init()
    twist_to_ackermann = Twist2Ackermann()
    rclpy.spin(twist_to_ackermann)
    twist_to_ackermann.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
