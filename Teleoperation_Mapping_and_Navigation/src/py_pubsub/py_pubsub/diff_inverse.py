import rclpy
from rclpy.node import Node
from py_pubsub.roboclaw import Roboclaw
from time import sleep
from geometry_msgs.msg import Twist,TransformStamped
from std_msgs.msg import Float32MultiArray
#from ros2_ws.src.py_pubsub.py_pubsub import roboclaw
import numpy as np
from rclpy.time import Time
from rclpy.constants import S_TO_NS
import math
from tf_transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class DifferentialInverse(Node):

    def __init__(self):
        super().__init__('diff_inverse')
        self.subscription = self.create_subscription(Twist,'cmd_vel_smoothed',self.diff_inverse_callback,10)
        address = 0x80
        roboclaw = Roboclaw("/dev/ttyS0", 38400)
        #roboclaw = Roboclaw("/dev/ttyAMA0", 38400)
        roboclaw.Open()
        
        self.address = address
        self.roboclaw = roboclaw

        p_M1 =  1.97657 #1.56562;
        i_M1 =  0.42826 #0.29137;
        d_M1 = 0.0;
        qpps_M1 = 6937 #10500;

        p_M2 = 1.95074 #1.53399;
        i_M2 = 0.41543;
        d_M2 = 0.0;
        qpps_M2 =  7312 #10687;
    
        self.p_M1 = p_M1
        self.i_M1 = i_M1
        self.d_M1 = d_M1
        self.qpps_M1 = qpps_M1

        self.p_M2 = p_M2
        self.i_M2 = i_M2
        self.d_M2 = d_M2
        self.qpps_M2 = qpps_M2

        #roboclaw.SetM1VelocityPID(address,p_M1,i_M1,d_M1,qpps_M1)
        #roboclaw.SetM2VelocityPID(address,p_M2,i_M2,d_M2,qpps_M2)

        # en_start1 = self.roboclaw.ReadEncM1(self.address)[1]
        # diff1 = 0
        # while diff1 < 17000:
        #     self.roboclaw.ForwardM1(self.address,100)
        #     self.roboclaw.ForwardM2(self.address,100)
        #     diff1 = self.roboclaw.ReadEncM1(self.address)[1] - en_start1

        # self.roboclaw.ForwardM1(self.address,0)   
        # self.roboclaw.ForwardM2(self.address,0) 

        ################  M1 is right motor 

        self.declare_parameter("wheel_radius", 0.034)
        self.declare_parameter("wheel_separation", 0.22)

        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.speed_conversion_ = np.array([[self.wheel_radius/2, self.wheel_radius/2],
                                           [self.wheel_radius/self.wheel_separation, -self.wheel_radius/self.wheel_separation]])

        self.left_wheel_prev_pos = self.roboclaw.ReadEncM2(self.address)[1]
        self.right_wheel_prev_pos = self.roboclaw.ReadEncM1(self.address)[1]
        self.prev_time = self.get_clock().now()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.encoder_count_per_revolute = 17000.0

        self.odom_pub = self.create_publisher(Odometry, "wheel_odom", 20)
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_footprint"
        self.odom_msg.pose.pose.orientation.x = 0.0
        self.odom_msg.pose.pose.orientation.y = 0.0
        self.odom_msg.pose.pose.orientation.z = 0.0
        self.odom_msg.pose.pose.orientation.w = 1.0

        # self.broadcaster = TransformBroadcaster(self)
        # self.transform_stamped = TransformStamped()
        # self.transform_stamped.header.frame_id = "odom"
        # self.transform_stamped.child_frame_id = "base_footprint"



    def diff_inverse_callback(self, msg):

        ########## Code for finding expected speed of the wheels  #################
        robot_speed = np.array([[msg.linear.x],
                                [msg.angular.z]])
        
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed) 

        right_wheel_velocity_expe = wheel_speed[0]
        left_wheel_velocity_expe = wheel_speed[1]

        # print("wheel_right: ", right_wheel_velocity_expe,  "wheel_left: ", left_wheel_velocity_expe)

        ########### Code for giving the expected speed to motors using pwm ###############

        # if (right_wheel_velocity_expe > 1.4 ) and (left_wheel_velocity_expe > 1.4):    #moving forweard
        #     self.roboclaw.ForwardM1(self.address,55)
        #     self.roboclaw.ForwardM2(self.address,55)

        # elif (right_wheel_velocity_expe < -1.4) and (left_wheel_velocity_expe < -1.4):  #move Backward
        #     self.roboclaw.BackwardM1(self.address,55)
        #     self.roboclaw.BackwardM2(self.address,55)

        # elif (1.1 < right_wheel_velocity_expe < 1.4 ) and (-1.4 < left_wheel_velocity_expe < -1.1):  #turn left
        #     self.roboclaw.BackwardM2(self.address,45)
        #     self.roboclaw.ForwardM1(self.address,45)

        # elif (-1.4 < right_wheel_velocity_expe < -1.1 ) and (1.1 < left_wheel_velocity_expe < 1.4):  #turn right
        #     self.roboclaw.BackwardM1(self.address,45)
        #     self.roboclaw.ForwardM2(self.address,45)

        # else :
        #     self.roboclaw.ForwardM1(self.address,0)     #stop
        #     self.roboclaw.ForwardM2(self.address,0)

        def custom_map(input):

            input_min = 0
            input_max = 12.0
            output_min = 100 #35
            output_max = 100

            # return int((input - input_min) * (output_max - output_min) / (input_max - input_min) + output_min)
            if input > 0:
                return output_min + int(((output_max-output_min) / (input_max-input_min)) * input )
            elif input < 0: 
                return - output_min + int(((output_max-output_min) / (input_max-input_min)) * input ) 
            else: 
                return 0 
        
        speed_m1 = abs(self.roboclaw.ReadSpeedM1(self.address)[1])
        speed_m2 = abs(self.roboclaw.ReadSpeedM2(self.address)[1])

        # Calculate error (difference in speeds)
        # error = speed_m1 - speed_m2

        # # Compute PID output
        # pid_output = int(error * 0.054)

        # print(pid_output)

        right_pwm = custom_map(right_wheel_velocity_expe) 
        left_pwm = custom_map(left_wheel_velocity_expe) 

        # print("right: " , right_pwm, " left: ", left_pwm)
        
        if right_pwm > 0 and left_pwm > 0:
            self.roboclaw.ForwardM1(self.address,abs(right_pwm))
            self.roboclaw.ForwardM2(self.address,abs(left_pwm))

        elif right_pwm < 0 and left_pwm < 0:
            self.roboclaw.BackwardM1(self.address,abs(right_pwm))
            self.roboclaw.BackwardM2(self.address,abs(left_pwm))

        elif left_pwm > 0 and right_pwm < 0:
            self.roboclaw.ForwardM2(self.address,abs(left_pwm))
            self.roboclaw.BackwardM1(self.address,abs(right_pwm))

        elif left_pwm < 0 and right_pwm > 0:
            self.roboclaw.BackwardM2(self.address,abs(left_pwm))
            self.roboclaw.ForwardM1(self.address,abs(right_pwm) )

        else:
            self.roboclaw.ForwardM1(self.address,0)    
            self.roboclaw.ForwardM2(self.address,0)


        ########## Code for finding real speeds   ################
        
        right_wheel_velocity_real = (self.roboclaw.ReadSpeedM1(self.address)[1] / self.encoder_count_per_revolute ) * (2*math.pi)
        left_wheel_velocity_real  = (self.roboclaw.ReadSpeedM2(self.address)[1] / self.encoder_count_per_revolute ) * (2*math.pi)

        linear_real = ((self.wheel_radius * right_wheel_velocity_real) + (self.wheel_radius * left_wheel_velocity_real)) / 2
        angular_real = ((self.wheel_radius * right_wheel_velocity_real) - (self.wheel_radius * left_wheel_velocity_real)) / self.wheel_separation

        # print("M2: ", right_wheel_velocity_real, "M1: ", left_wheel_velocity_real)



        ######### Code for finding odometry

        # dp_left = ((self.roboclaw.ReadEncM2(self.address)[1] - self.left_wheel_prev_pos)/self.encoder_count_per_revolute) * (2*math.pi)
        # dp_right = ((self.roboclaw.ReadEncM1(self.address)[1] - self.right_wheel_prev_pos)/self.encoder_count_per_revolute) * (2*math.pi)
        dt = self.get_clock().now() - self.prev_time
        dp_left = left_wheel_velocity_real*(dt.nanoseconds / S_TO_NS)
        dp_right = right_wheel_velocity_real*(dt.nanoseconds / S_TO_NS)
        
        self.left_wheel_prev_pos = self.roboclaw.ReadEncM2(self.address)[1]
        self.right_wheel_prev_pos = self.roboclaw.ReadEncM1(self.address)[1]
        self.prev_time = self.get_clock().now()

        # print(self.roboclaw.ReadEncM2(self.address)[1], self.roboclaw.ReadEncM1(self.address)[1])

        # Calculate the position increment
        d_s = (self.wheel_radius * dp_right + self.wheel_radius * dp_left) / 2
        d_theta = (self.wheel_radius * dp_right - self.wheel_radius * dp_left) / self.wheel_separation
        self.theta += d_theta
        self.x += d_s * math.cos(self.theta)
        self.y += d_s * math.sin(self.theta)

        # print("x: ", self.x, " / ", "y: ", self.y, " / ", "theta: ", self.theta)
        
        # Compose and publish the odom message
        q = quaternion_from_euler(0, 0, self.theta) 
        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.orientation.x = q[0]
        self.odom_msg.pose.pose.orientation.y = q[1]
        self.odom_msg.pose.pose.orientation.z = q[2]
        self.odom_msg.pose.pose.orientation.w = q[3]
        self.odom_msg.twist.twist.linear.x = linear_real
        self.odom_msg.twist.twist.angular.z = angular_real
        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_pub.publish(self.odom_msg)


        # Boradcasting odom to base footprint tf
        # self.transform_stamped.transform.translation.x = self.x
        # self.transform_stamped.transform.translation.y = self.y
        # self.transform_stamped.transform.rotation.x = q[0]
        # self.transform_stamped.transform.rotation.y = q[1]
        # self.transform_stamped.transform.rotation.z = q[2]
        # self.transform_stamped.transform.rotation.w = q[3]
        # self.transform_stamped.header.stamp = self.get_clock().now().to_msg()
        # self.broadcaster.sendTransform(self.transform_stamped)  



def main(args=None):
    rclpy.init(args=args)

    diff_inverse = DifferentialInverse()

    rclpy.spin(diff_inverse)
    diff_inverse.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
