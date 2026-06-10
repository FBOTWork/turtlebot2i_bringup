#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import serial  # type: ignore
import time

from sensor_msgs.msg import Imu


class IMUNode:

    def __init__(self):

        rospy.init_node('imu_node')

        port = rospy.get_param('~port', '/dev/arduino')
        baud = rospy.get_param('~baud', 115200)
        self.frame_id = rospy.get_param('~frame_id', 'imu_link')

        self.ser = serial.Serial(port, baud, timeout=1)

        time.sleep(2.0)
        self.ser.flushInput()

        self.pub_imu = rospy.Publisher('/imu/data_raw', Imu, queue_size=10)

        rospy.loginfo("IMU node started")


    def run(self):

        rate = rospy.Rate(50)

        while not rospy.is_shutdown():

            try:
                line = self.ser.readline().strip()

                if not line:
                    continue

                self.process_line(line)

            except Exception as e:
                rospy.logwarn("Serial error: %s", str(e))

            rate.sleep()


    def process_line(self, line):

        try:
            if "IMU:" not in line:
                return

            line = line.split("IMU:")[-1]
            line = line.replace(";", "")
            line = line.replace(" ", "")
            line = line.strip()

            parts = line.split(',')

            if len(parts) != 10:
                rospy.logwarn_throttle(5.0, "Invalid IMU line: %s", line)
                return

            try:
                data = [float(x) for x in parts]
            except ValueError:
                rospy.logwarn_throttle(5.0, "Invalid numeric value in IMU line: %s", line)
                return

            msg = Imu()

            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.frame_id

            msg.orientation.x = data[0]
            msg.orientation.y = data[1]
            msg.orientation.z = data[2]
            msg.orientation.w = data[3]

            msg.orientation_covariance = [
                0.05, 0.0, 0.0,
                0.0, 0.05, 0.0,
                0.0, 0.0, 0.30
            ]

            msg.angular_velocity.x = data[4]
            msg.angular_velocity.y = data[5]
            msg.angular_velocity.z = data[6]

            msg.angular_velocity_covariance = [
                0.02, 0.0, 0.0,
                0.0, 0.02, 0.0,
                0.0, 0.0, 0.02
            ]

            msg.linear_acceleration.x = data[7]
            msg.linear_acceleration.y = data[8]
            msg.linear_acceleration.z = data[9]

            msg.linear_acceleration_covariance = [
                0.04, 0.0, 0.0,
                0.0, 0.04, 0.0,
                0.0, 0.0, 0.04
            ]

            self.pub_imu.publish(msg)

        except Exception as e:
            rospy.logwarn_throttle(5.0, "Parsing error: %s | line: %s", str(e), line)


if __name__ == '__main__':

    node = IMUNode()
    node.run()