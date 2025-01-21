#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np 
from std_msgs.msg import Float32MultiArray


class SimpleController(Node):
    def __init__(self):
        super().__init__("simple_controller")

        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.17)

        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info("Using wheel_radius %f" % self.wheel_radius)
        self.get_logger().info("Using wheel_separation %f" % self.wheel_separation)

        self.wheel_cmd_vel_pub_ = self.create_publisher(Float32MultiArray, "wheel_vel" , 10)
        self.wheel_remapped_cmd_vel_sub = self.create_subscription(Twist, "remapped_cmd_vel", self.velCallback, 10)

        self.speed_conversion = np.array([[self.wheel_radius/2 , self.wheel_radius/2], 
                                          [self.wheel_radius/self.wheel_separation , -self.wheel_radius/self.wheel_separation ]])
        

    def velCallback(self, msg):
        robot_speed = np.array([[msg.linear.x],
                                [msg.angular.z]])
        
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion), robot_speed)

        self.get_logger().info("Wheel_speed_left %f" % wheel_speed[1,0])
        self.get_logger().info("Wheel_speed_right %f" % wheel_speed[0,0])

        wheel_speed_msg = Float32MultiArray()
        wheel_speed_msg.data = [wheel_speed[1, 0], wheel_speed[0, 0]]

        print("Left wheel", wheel_speed_msg.data[0], "Right wheel", wheel_speed_msg.data[1])

        self.wheel_cmd_vel_pub_.publish(wheel_speed_msg)


def main():
    rclpy.init()
    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
