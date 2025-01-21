# import logging
# import sys
# import time
# from Adafruit_BNO055 import BNO055

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Imu
# import math
# from tf_transformations import quaternion_from_euler

# class ImuSensor(Node):

#     def __init__(self):
#         super().__init__('imu_sensor')
    
#         self.bno = BNO055.BNO055(rst=18)

#         ########### initailizing imu library ##########
#         # Enable verbose debug logging if -v is passed as a parameter.
#         if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':
#             logging.basicConfig(level=logging.DEBUG)

#         # Initialize the BNO055 and stop if something went wrong.
#         if not self.bno.begin():
#             raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

#         # Print system status and self test result.
#         status, self_test, error = self.bno.get_system_status()
#         print('System status: {0}'.format(status))
#         print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
#         # Print out an error if system status is in error mode.
#         if status == 0x01:
#             print('System error: {0}'.format(error))
#             print('See datasheet section 4.3.59 for the meaning.')

#         # Print BNO055 software revision and other diagnostic data.
#         sw, bl, accel, mag, gyro = self.bno.get_revision()
#         print('Software version:   {0}'.format(sw))
#         print('Bootloader version: {0}'.format(bl))
#         print('Accelerometer ID:   0x{0:02X}'.format(accel))
#         print('Magnetometer ID:    0x{0:02X}'.format(mag))
#         print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))

#         print('Reading BNO055 data, press Ctrl-C to quit...')

#         self.imu_pub = self.create_publisher(Imu, "imu/data_raw", 10)
#         self.imu_msg = Imu()

#         self.publishing_loop()

#     def publishing_loop(self):

#         while True:
#             # Read the Euler angles for heading, roll, pitch (all in degrees).
#             heading, roll, pitch = self.bno.read_euler()
#             # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
#             sys, gyro, accel, mag = self.bno.get_calibration_status()
#             # Print everything out.
#             # print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
#             #     heading, roll, pitch, sys, gyro, accel, mag))
#             # Other values you can optionally read:
#             # Orientation as a quaternion:
#             #x,y,z,w = bno.read_quaterion()
#             # Sensor temperature in degrees Celsius:
#             #temp_c = bno.read_temp()
#             # Magnetometer data (in micro-Teslas):
#             #x,y,z = bno.read_magnetometer()
#             # Gyroscope data (in degrees per second):
#             #x,y,z = bno.read_gyroscope()
#             # Accelerometer data (in meters per second squared):
#             #x,y,z = bno.read_accelerometer()
#             # Linear acceleration data (i.e. acceleration from movement, not gravity--
#             # returned in meters per second squared):
#             #x,y,z = bno.read_linear_acceleration()
#             # Gravity acceleration data (i.e. acceleration just from gravity--returned
#             # in meters per second squared):
#             #x,y,z = bno.read_gravity()
#             # Sleep for a second until the next reading.


#             self.imu_msg.header.stamp = self.get_clock().now().to_msg()
#             self.imu_msg.header.frame_id = "imu_link"
#             euler = quaternion_from_euler(0, 0, heading * 0.0174533)
#             self.imu_msg.orientation.x = float(euler[0])
#             self.imu_msg.orientation.y = float(euler[1])
#             self.imu_msg.orientation.z = float(euler[2])
#             self.imu_msg.orientation.w = float(euler[3])
#             self.imu_msg.linear_acceleration.x = float(self.bno.read_linear_acceleration()[0])
#             self.imu_msg.linear_acceleration.y = float(self.bno.read_linear_acceleration()[1])
#             self.imu_msg.linear_acceleration.z = float(self.bno.read_linear_acceleration()[2])
#             self.imu_msg.linear_acceleration_covariance[0] = -1
#             self.imu_msg.angular_velocity.x = float(self.bno.read_gyroscope()[0] * (math.pi / 180))
#             self.imu_msg.angular_velocity.y = float(self.bno.read_gyroscope()[1] * (math.pi / 180))
#             self.imu_msg.angular_velocity.z = float(self.bno.read_gyroscope()[2] * (math.pi / 180))
#             self.imu_msg.angular_velocity_covariance[0] = -1



#             self.imu_pub.publish(self.imu_msg)
            



# def main(args=None):
#     rclpy.init(args=args)

#     imu_sensor = ImuSensor()

#     rclpy.spin(imu_sensor)
#     imu_sensor.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()







import logging
import sys
import time
from Adafruit_BNO055 import BNO055

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from tf2_ros import TransformBroadcaster
from tf2_geometry_msgs import TransformStamped
import math
from tf_transformations import quaternion_from_euler

class ImuSensor(Node):

    def __init__(self):
        super().__init__('imu_sensor')
    
        self.bno = BNO055.BNO055(rst=18)

        ########### initializing imu library ##########
        # Enable verbose debug logging if -v is passed as a parameter.
        if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':
            logging.basicConfig(level=logging.DEBUG)

        # Initialize the BNO055 and stop if something went wrong.
        if not self.bno.begin():
            raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

        # Print system status and self test result.
        status, self_test, error = self.bno.get_system_status()
        self.get_logger().info('System status: {0}'.format(status))
        self.get_logger().info('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
        # Print out an error if system status is in error mode.
        if status == 0x01:
            self.get_logger().info('System error: {0}'.format(error))
            self.get_logger().info('See datasheet section 4.3.59 for the meaning.')

        # Print BNO055 software revision and other diagnostic data.
        sw, bl, accel, mag, gyro = self.bno.get_revision()
        self.get_logger().info('Software version:   {0}'.format(sw))
        self.get_logger().info('Bootloader version: {0}'.format(bl))
        self.get_logger().info('Accelerometer ID:   0x{0:02X}'.format(accel))
        self.get_logger().info('Magnetometer ID:    0x{0:02X}'.format(mag))
        self.get_logger().info('Gyroscope ID:       0x{0:02X}\n'.format(gyro))

        self.get_logger().info('Reading BNO055 data, press Ctrl-C to quit...')

        self.imu_pub = self.create_publisher(Imu, "imu_data", 10)
        self.broadcaster = TransformBroadcaster(self)

        self.publishing_loop()

    def publishing_loop(self):
        while True:
            # Read the Euler angles for heading, roll, pitch (all in degrees).
            heading, roll, pitch = self.bno.read_euler()
            # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
            sys, gyro, accel, mag = self.bno.get_calibration_status()

            # Create a message and fill it with IMU data
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'

            quaternion = quaternion_from_euler(math.radians(roll), math.radians(pitch), math.radians(heading))
            imu_msg.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])

            imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z = self.bno.read_gyroscope()
            imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z = self.bno.read_accelerometer()

            # Publish the message
            self.imu_pub.publish(imu_msg) 
            



def main(args=None):
    rclpy.init(args=args)

    imu_sensor = ImuSensor()

    rclpy.spin(imu_sensor)
    imu_sensor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

