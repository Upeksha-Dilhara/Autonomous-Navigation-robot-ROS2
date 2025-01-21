import rclpy
from rclpy.node import Node
from py_pubsub.roboclaw import Roboclaw
from time import sleep
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
#from ros2_ws.src.py_pubsub.py_pubsub import roboclaw




class Diff_Inverse(Node):

    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.subscription = self.create_subscription(Twist,'cmd_vel',self.cmd_to_pwm_callback,10)
        address = 0x80
        roboclaw = Roboclaw("/dev/ttyS0", 38400)
        #roboclaw = Roboclaw("/dev/ttyAMA0", 38400)
        roboclaw.Open()
        
        self.address = address
        self.roboclaw = roboclaw

        p_M1 = 1.56562;
        i_M1 = 0.29137;
        d_M1 = 0.0;
        qpps_M1 = 10500;

        p_M2 = 1.53399;
        i_M2 = 0.27342;
        d_M2 = 0.0;
        qpps_M2 = 10687;
    
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

        en_start1 = self.roboclaw.ReadEncM1(self.address)[1]
        diff1 = 0
        while diff1 < 17000:
            self.roboclaw.ForwardM1(self.address,50)
            diff1 = self.roboclaw.ReadEncM1(self.address)[1] - en_start1

        self.roboclaw.ForwardM1(self.address,0)

    def cmd_to_pwm_callback(self, msg):

        right_wheel_vel = ( msg.linear.x  + msg.angular.z ) /2
        left_wheel_vel = (  msg.linear.x  - msg.angular.z ) /2

        print(left_wheel_vel  , " / " ,right_wheel_vel)

        print("Encoder_M1", self.roboclaw.ReadEncM1(self.address), " / Encoder_M2", self.roboclaw.ReadEncM2(self.address))

        #if (right_wheel_vel >0):
            #self.roboclaw.ForwardM1(self.address,63)
        #if (right_wheel_vel <0):
            #self.roboclaw.BackwardM1(self.address,63)
        #if (right_wheel_vel >0):
            #self.roboclaw.ForwardM2(self.address,63)
        #if (right_wheel_vel <0):
            #self.roboclaw.BackwardM1(self.address,63)
        

        encoder1 = self.roboclaw.ReadEncM1(self.address)[1]
        
        


        if (right_wheel_vel == 0.25) and (left_wheel_vel == 0.25):    #moving forweard
            self.roboclaw.ForwardM1(self.address,50)
            self.roboclaw.ForwardM2(self.address,50)

        elif (right_wheel_vel == -0.25) and (left_wheel_vel == -0.25):  #move Backward
            self.roboclaw.BackwardM1(self.address,50)
            self.roboclaw.BackwardM2(self.address,50)

        elif (right_wheel_vel == 0.25) and (left_wheel_vel == -0.25):  #turn right
            self.roboclaw.BackwardM2(self.address,30)
            self.roboclaw.ForwardM1(self.address,30)

        elif (right_wheel_vel == -0.25) and (left_wheel_vel == 0.25):  #turn left
            self.roboclaw.BackwardM1(self.address,30)
            self.roboclaw.ForwardM2(self.address,30)

        else :
            self.roboclaw.ForwardM1(self.address,0)     #stop
            self.roboclaw.ForwardM2(self.address,0)

        
        #GPIO.output(self.mr_a, right_wheel_vel > 0)
        #GPIO.output(self.mr_b, right_wheel_vel < 0 )
        #GPIO.output(self.ml_a, left_wheel_vel > 0)
        #GPIO.output(self.ml_b, left_wheel_vel < 0)




def main(args=None):
    rclpy.init(args=args)

    diff_inverse = Diff_Inverse()

    rclpy.spin(diff_inverse)
    diff_inverse.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
