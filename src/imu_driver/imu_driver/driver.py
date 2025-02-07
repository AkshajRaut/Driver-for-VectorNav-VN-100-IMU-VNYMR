#!/usr/bin/env python3

import serial
import glob
import rclpy
from rclpy.node import Node
from imu_messages.msg import IMUmsg
from std_msgs.msg import Header
import numpy as np
from datetime import datetime
import math

class IMUDriver(Node):
    def __init__(self):
        super().__init__('imu_driver')
        self.get_logger().info("Driver started")
        self.declare_parameter('port','/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        port_pattern = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        available_ports = glob.glob(port_pattern)
        
        if not available_ports:
            self.get_logger().error(f"No serial ports found for pattern: {port_pattern}")
            rclpy.shutdown()
            return

        self.port = None
        for port in available_ports:
            if 'ttyUSB' in port or 'ttyACM' in port or 'pts' in port: 
                try:
                    self.port = serial.Serial(port, baud, timeout=3)
                    self.port.write(b"$VNWRG,07,40*59")
                    self.get_logger().info(f"Using Serial port: {port}")
                    break
                except (serial.SerialException, OSError):
                    self.get_logger().warning(f"Failed to open port: {port}")

        if self.port is None:
            self.get_logger().error("No valid serial ports available.")
            rclpy.shutdown()
            return

        self.publisher = self.create_publisher(IMUmsg, 'imu', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)


    def timer_callback(self):
        line = self.port.readline().decode().strip()
        if line.startswith('$VNYMR'):
            data = line.split(',')
            
            yaw = float(data[1])
            yaw_rad = yaw * (math.pi/180)
            pitch = float(data[2])
            pitch_rad = pitch * (math.pi/180)
            roll = float(data[3])
            roll_rad = roll * (math.pi/180)
            mag_X = float(data[4])/10000
            mag_Y = float(data[5])/10000
            mag_Z = float(data[6])/10000
            acc_X = float(data[7])
            acc_Y = float(data[8])
            acc_Z = float(data[9])
            gyro_X = float(data[10])
            gyro_Y = float(data[11])
            gyro_Z = float(data[12][0:9])

            qy = np.cos(roll_rad/2) * np.sin(pitch_rad/2) * np.cos(yaw_rad/2) + np.sin(roll_rad/2) * np.cos(pitch_rad/2) * np.sin(yaw_rad/2)
            qx = np.sin(roll_rad/2) * np.cos(pitch_rad/2) * np.cos(yaw_rad/2) - np.cos(roll_rad/2) * np.sin(pitch_rad/2) * np.sin(yaw_rad/2)
            qz = np.cos(roll_rad/2) * np.cos(pitch_rad/2) * np.sin(yaw_rad/2) - np.sin(roll_rad/2) * np.sin(pitch_rad/2) * np.cos(yaw_rad/2)
            qw = np.cos(roll_rad/2) * np.cos(pitch_rad/2) * np.cos(yaw_rad/2) + np.sin(roll_rad/2) * np.sin(pitch_rad/2) * np.sin(yaw_rad/2)
            
            current_datetime = datetime.now()
            seconds = current_datetime.hour*3600 + current_datetime.minute*60 + current_datetime.second
            nseconds = seconds + current_datetime.microsecond*1000
     
            msg = IMUmsg()
            msg.header.stamp.sec = seconds
            msg.header.stamp.nanosec = nseconds
            msg.header.frame_id = 'IMU1_Frame'
            msg.imu.orientation.x = qx
            msg.imu.orientation.y = qy
            msg.imu.orientation.z = qz
            msg.imu.orientation.w = qw
            msg.imu.linear_acceleration.x = acc_X
            msg.imu.linear_acceleration.y = acc_Y
            msg.imu.linear_acceleration.z = acc_Z
            msg.imu.angular_velocity.x = gyro_X
            msg.imu.angular_velocity.y = gyro_Y
            msg.imu.angular_velocity.z = gyro_Z
            msg.mag_field.magnetic_field.x = mag_X
            msg.mag_field.magnetic_field.y = mag_Y
            msg.mag_field.magnetic_field.z = mag_Z

            self.publisher.publish(msg)
            self.get_logger().info("Published IMU Data "+str(msg))

def main(args=None):
    rclpy.init(args=args)
    node = IMUDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.port.close()
        node.destroy_node()
        rclpy.shutdown()
        

if __name__ == '__main__':
    main()
