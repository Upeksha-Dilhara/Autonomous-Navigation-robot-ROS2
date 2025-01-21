#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int8, UInt8, Float32, String
from roboclaw_python_library.roboclaw_python.roboclaw_3 import Roboclaw
import time

# class for define motors
class Motors():
    def __init__(self):
        # variables for sensors
        self.ir_read_position = 10
        self.left_distance = 254
        self.right_distance = 254
        self.imu = 0        
        
        self.enableCenterTask = True
        self.cmd_delay = 0.15

        # motor speeds
        self.left_motor_speed = 0 # M1
        self.right_motor_speed = 0  # M2

        self.isdocked = False   # check if docked

        ### ROS ###
        rospy.init_node("motors")
        rospy.Subscriber("/ir", Int8, self.irCallback)
        rospy.Subscriber("/left_distance", UInt8, self.leftDistanceCallback)
        rospy.Subscriber("/right_distance", UInt8, self.rightDistanceCallback)
        rospy.Subscriber("/imu", Float32, self.imuCallback)
        self.velPub = rospy.Publisher("/cmd_vel", String, queue_size=10)
    
    # callback function for IR data
    def irCallback(self, msg):
        self.ir_read_position = msg.data
    
    # callback function for left distance sensor data
    def leftDistanceCallback(self, msg):
        self.left_distance = msg.data
    
    # callback function for right distance data
    def rightDistanceCallback(self, msg):
        self.right_distance = msg.data
    
    # callback function for IMU data
    def imuCallback(self, msg):
        self.imu = msg.data
    
    # drive motors forward
    def forwardDrive(self, speed=25):
        self.left_motor_speed = speed
        self.right_motor_speed = speed
        self.velPub.publish(f"{-speed},{-speed}")
    
    # drive motors backward
    def backwardDrive(self, speed=25):
        self.left_motor_speed = speed
        self.right_motor_speed = speed
        self.velPub.publish(f"{speed},{speed}")
    
    # stop motors
    def stopDrive(self):        
        self.left_motor_speed = 0
        self.right_motor_speed = 0
        self.velPub.publish("0,0")
    
    # set the task according to the IR data
    def IR_Task_Process(self):
        # IR detected
        if (self.ir_read_position!=10):            
            if(self.ir_read_position == 0):
                self.robotTaskStepCenter()
            elif (self.ir_read_position == 1):
                self.setCenter(1)
            elif (self.ir_read_position == 2):
                self.setCenter(2)
            elif (self.ir_read_position == 3):
                self.setCenter(3)
            elif (self.ir_read_position == 4):
                self.setCenter(4)
            # else if(self.ir_read_position == 5){

            # }else if(self.ir_read_position == 6){

            # }else if(self.ir_read_position == 7){

            elif (self.ir_read_position == -1):
                self.setCenter(-1)
            elif (self.ir_read_position == -2):
                self.setCenter(-2)
            elif (self.ir_read_position == -3):
                self.setCenter(-3)
            elif(self.ir_read_position == -4):
                self.setCenter(-4)
            elif(self.ir_read_position == -5):
                self.setCenter(-5)
            # else if(self.ir_read_position == -6){

            # }else if(self.ir_read_position == -7){

            # }
    
    # setting motor speeds and aligning according to sensor data
    def robotTaskStepCenter(self):
        if(self.enableCenterTask):
            if(self.left_distance<255 or self.right_distance<255):              
                while (((self.left_distance != self.right_distance) or ((self.left_distance-self.right_distance) > 20) or ((self.right_distance-self.left_distance) > 20)) and not rospy.is_shutdown()): # not parallel
                    # self.distance_Sensor_sub_task()   # Check sensor data avalability

                    if (self.left_distance > self.right_distance):
                        self.right_motor_speed = 25# roboclaw.ForwardM2(address,25);
                        self.left_motor_speed = 0# roboclaw.ForwardM1(address,0);
                        self.velPub.publish("0,25")
                        print("right back")
                    elif(self.right_distance > self.left_distance):
                        self.right_motor_speed = 0#roboclaw.ForwardM2(address,0);
                        self.left_motor_speed = 25#roboclaw.ForwardM1(address,25);
                        self.velPub.publish("25,0")
                        print("left back")
                    rospy.sleep(self.cmd_delay)
                    
                    # self.velPub.publish(f"{self.left_motor_speed},{self.right_motor_speed}")
                    
                # is parallel
                self.stopDrive()
                print("stop")
                #self.left_motor_speed = 0#roboclaw.ForwardM1(address,0); // stop forward
                #self.right_motor_speed = 0#roboclaw.ForwardM2(address,0); // stop forward
                if (self.left_distance<50 and self.right_distance<50):
                    self.enableCenterTask = False
                    self.isdocked = True
            else:
                self.backwardDrive()
                print("back")
                rospy.sleep(0.01)
                #self.left_motor_speed = 25#roboclaw.ForwardM1(address,25); // go forward
                #self.right_motor_speed = 25#roboclaw.ForwardM2(address,25); // go forward
    
    # rotating
    def rotate(self, side):
        if (side == "L"):
            self.velPub.publish("25,-25")
        elif (side == "R"):
            self.velPub.publish("-25,25")

    # turning left or right
    def turn(self, angle, side):
        start_angle = self.imu

        while not rospy.is_shutdown():
            if (abs(self.imu-start_angle)<angle or (360-(abs(self.imu-start_angle))<angle)):
                print("rotating", abs(self.imu-start_angle))
                self.rotate(side)
                rospy.sleep(self.cmd_delay)
            else:
                print("stop rotating")
                self.stopDrive()
                rospy.sleep(self.cmd_delay)
                break
    
    # drive for some time or encoder count forward or backward
    def freeDrive(self, mode='D', encoderCount=0, direction='B', d_time=0):        
        if (mode == 'D'):
            while not rospy.is_shutdown():
                print("Distance mode", self.left_distance, self.right_distance)
                if (self.left_distance==255 and self.right_distance==255):
                    self.stopDrive()
                    print("stop")
                    rospy.sleep(self.cmd_delay)
                    break
                else:
                    self.forwardDrive()
                    rospy.sleep(self.cmd_delay)
        elif (mode == "T"):
            start_time = time.time()
            while not rospy.is_shutdown():
                if ((time.time()-start_time)<d_time):
                    print("driving")
                    if (direction == "B"):
                        self.backwardDrive()
                        rospy.sleep(self.cmd_delay)
                    else:
                        self.forwardDrive()
                        rospy.sleep(self.cmd_delay)
                else:
                    print("stop")
                    self.stopDrive()
                    rospy.sleep(self.cmd_delay)
                    break

    # move towards the center from another position
    def setCenter(self, ir_position):
        # set the rotating and driving according to IR data
        if (ir_position>0):
            if (ir_position==1):
                clockwise_angle = 90
                counter_clockwise_angle = 60
                forward_driving_time = 2
            elif (ir_position==2 or ir_position==3):
                clockwise_angle = 90
                counter_clockwise_angle = 60
                forward_driving_time = 5
            elif (ir_position==4 or ir_position==5):
                clockwise_angle = 90
                counter_clockwise_angle = 60
                forward_driving_time = 5     

            self.freeDrive()
            rospy.sleep(1)
            self.turn(clockwise_angle, "R")
            rospy.sleep(1)
            self.freeDrive("T", direction="F", d_time=forward_driving_time)
            rospy.sleep(1)
            self.turn(counter_clockwise_angle, "L")
            rospy.sleep(1)   
        else:
            if (ir_position==-1):
                clockwise_angle = 60
                counter_clockwise_angle = 90
                forward_driving_time = 2
            elif (ir_position==-2 or ir_position==-3):
                clockwise_angle = 60
                counter_clockwise_angle = 90
                forward_driving_time = 5
            elif (ir_position==-4 or ir_position==-5):
                clockwise_angle = 60
                counter_clockwise_angle = 90
                forward_driving_time = 5
            self.freeDrive()
            rospy.sleep(1)
            self.turn(counter_clockwise_angle, "L")
            rospy.sleep(1)
            self.freeDrive("T", direction="F", d_time=forward_driving_time)
            rospy.sleep(1)
            self.turn(clockwise_angle, "R")
            rospy.sleep(1)

# initialization
def init():
    motors = Motors()
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        if not motors.isdocked:
            motors.IR_Task_Process()

if __name__ == "__main__":
    try:
        init()
    except rospy.ROSException:
        pass
