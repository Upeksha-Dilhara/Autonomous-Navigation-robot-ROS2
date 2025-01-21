import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterType
from rclpy.parameter import Parameter

import numpy as np


class VelocitySmoother(Node):

    def __init__(self):
        super().__init__('velocity_smoother')
        self.smoothing_frequency = self.declare_parameter('smoothing_frequency', 20.0).value
        self.feedback = self.declare_parameter('feedback', 'OPEN_LOOP').value
        self.scale_velocities = self.declare_parameter('scale_velocities', False).value
        self.max_velocity = self.declare_parameter('max_velocity', [0.50, 0.0, 2.5]).value
        self.min_velocity = self.declare_parameter('min_velocity', [-0.50, 0.0, -2.5]).value
        self.max_accel = self.declare_parameter('max_accel', [2.5, 0.0, 3.2]).value
        self.max_decel = self.declare_parameter('max_decel', [-2.5, 0.0, -3.2]).value
        self.odom_topic = self.declare_parameter('odom_topic', 'odom').value
        self.odom_duration = self.declare_parameter('odom_duration', 0.1).value
        self.deadband_velocity = self.declare_parameter('deadband_velocity', [0.0, 0.0, 0.0]).value
        self.velocity_timeout = self.declare_parameter('velocity_timeout', 1.0).value

        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.input_command_callback, QoSProfile(depth=1))
        self.smoothed_cmd_pub = self.create_publisher(
            Twist, 'cmd_vel_smoothed', QoSProfile(depth=1))

        self.timer = self.create_timer(1.0 / self.smoothing_frequency, self.smoother_timer)

        self.last_command_time = self.get_clock().now()
        self.last_command = Twist()
        self.stopped = False

    def input_command_callback(self, msg):
        # Check for NaN or Inf
        if np.any(np.isnan([msg.linear.x, msg.linear.y, msg.angular.z])) or \
                np.any(np.isinf([msg.linear.x, msg.linear.y, msg.angular.z])):
            self.get_logger().error("Velocity message contains NaNs or Infs! Ignoring as invalid!")
            return
        self.last_command = msg
        self.last_command_time = self.get_clock().now()

    def smoother_timer(self):
        if self.get_clock().now() - self.last_command_time > rclpy.duration.Duration(seconds=self.velocity_timeout):
            if self.last_command.linear.x == 0 and self.last_command.linear.y == 0 and \
                    self.last_command.angular.z == 0 or self.stopped:
                self.stopped = True
                return
            self.last_command = Twist()

        self.stopped = False

        current = self.last_command

        # Apply absolute velocity restrictions
        current.linear.x = np.clip(current.linear.x, self.min_velocity[0], self.max_velocity[0])
        current.linear.y = np.clip(current.linear.y, self.min_velocity[1], self.max_velocity[1])
        current.angular.z = np.clip(current.angular.z, self.min_velocity[2], self.max_velocity[2])

        eta = 1.0

        if self.scale_velocities:
            # Find eta constraint
            eta = self.find_eta_constraint(
                current.linear.x, self.last_command.linear.x, self.max_accel[0], self.max_decel[0])
            eta = max(eta, self.find_eta_constraint(
                current.linear.y, self.last_command.linear.y, self.max_accel[1], self.max_decel[1]))
            eta = max(eta, self.find_eta_constraint(
                current.angular.z, self.last_command.angular.z, self.max_accel[2], self.max_decel[2]))

        # Apply constraints
        cmd_vel = Twist()
        cmd_vel.linear.x = self.apply_constraints(
            current.linear.x, self.last_command.linear.x, self.max_accel[0], self.max_decel[0], eta)
        cmd_vel.linear.y = self.apply_constraints(
            current.linear.y, self.last_command.linear.y, self.max_accel[1], self.max_decel[1], eta)
        cmd_vel.angular.z = self.apply_constraints(
            current.angular.z, self.last_command.angular.z, self.max_accel[2], self.max_decel[2], eta)

        # Apply deadband restrictions
        cmd_vel.linear.x = 0.0 if abs(cmd_vel.linear.x) < self.deadband_velocity[0] else cmd_vel.linear.x
        cmd_vel.linear.y = 0.0 if abs(cmd_vel.linear.y) < self.deadband_velocity[1] else cmd_vel.linear.y
        cmd_vel.angular.z = 0.0 if abs(cmd_vel.angular.z) < self.deadband_velocity[2] else cmd_vel.angular.z

        self.smoothed_cmd_pub.publish(cmd_vel)


    def find_eta_constraint(self, v_curr, v_cmd, accel, decel):
        dv = v_cmd - v_curr
        if abs(v_cmd) >= abs(v_curr) and v_curr * v_cmd >= 0.0:
            v_component_max = accel / self.smoothing_frequency
            v_component_min = -accel / self.smoothing_frequency
        else:
            v_component_max = -decel / self.smoothing_frequency
            v_component_min = decel / self.smoothing_frequency

        if dv > v_component_max:
            return v_component_max / dv
        if dv < v_component_min:
            return v_component_min / dv
        return -1.0

    def apply_constraints(self, v_curr, v_cmd, accel, decel, eta):
        dv = v_cmd - v_curr
        if abs(v_cmd) >= abs(v_curr) and v_curr * v_cmd >= 0.0:
            v_component_max = accel / self.smoothing_frequency
            v_component_min = -accel / self.smoothing_frequency
        else:
            v_component_max = -decel / self.smoothing_frequency
            v_component_min = decel / self.smoothing_frequency

        return v_curr + np.clip(eta * dv, v_component_min, v_component_max)


def main(args=None):
    rclpy.init(args=args)
    velocity_smoother = VelocitySmoother()
    rclpy.spin(velocity_smoother)
    velocity_smoother.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
