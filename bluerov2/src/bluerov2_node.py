# !/usr/bin/env python3 
# Implemented to convert bluerov's IMU, camera, and status information into ros msgs 
# The BlueRov class inherits from Bridge

from __future__ import division

import json
import math
import re

import numpy as np
import rospy
import numpy
# import sys
import time
import utm
import socket
# sys.path.append("/home/antonio/catkin_ws/src/bluerov2")
from utils.bluerov2_bridge_mavlink import Bridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist,Accel
from bluerov2.msg import State

armed = True
depth_hold = True
translation_limit = 266.5
rotation_limit = 266.5
max_vel = 1.5
max_omega = 1.5
vel_to_cmd = translation_limit / max_vel
omega_to_cmd = rotation_limit / max_omega


def shutdown():
    global bluerov
    bluerov.arm_throttle(False)


class BlueRov(Bridge):
    def __init__(self, device='udpin:192.168.2.1:14550', baudrate=115200, source_system=1, source_component=1):
        """
        Inherited from Bridge from Mavlink to ROS
        Args:
            device (str, optional): mavproxy device description
            baudrate (int, optional): Serial baudrate
        """
        super(BlueRov, self).__init__(device, baudrate, source_system, source_component)
        self.ROV_name = '/BlueRov2'
        self.model_base_link = '/base_link'
        self.control_actions = [0, 0, 0, 0]
        self.pub_topics = {
            '/battery':
                [
                    self._create_battery_msg,
                    BatteryState,
                    10
                ],
            '/state':
                [
                    self._create_rov_state,
                    State,
                    10
                ],
            '/odometry':
                [
                    self._create_odometry_msg,
                    Odometry,
                    10
                ],

        }
        self.sub_topics = {
            # '/cmd_vel':
            #    [self._control_callback,
            #     Twist,
            #     1]
            '/BlueRov2/cmd_vel':
                [self._control_callback,
                 Accel,
                 10]

        }

        self.mavlink_msg_available = {}
        for topic, pubs in self.pub_topics.items():
            self.mavlink_msg_available[topic] = 0  # Control of transmission frequency
            pubs.append(rospy.Publisher(self.ROV_name + topic, pubs[1],
                                        queue_size=pubs[2]))  # add object publisher into each pub_topic
        for topic, subs in self.sub_topics.items():
            callback, msg_type, queue_size = subs
            rospy.Subscriber(topic, msg_type, callback, queue_size=queue_size)

    def _create_header(self, msg):
        """
        Create ROS message header to comunicate timestamp data
        Args:
            msg (ROS message): ROS message with header
        """
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.model_base_link

    def _create_odometry_msg(self, topic):
        """
        Create odometry message from ROV information

        Raises:
            Exception: No data to create the message
        """

        # Check if data is available
        if 'GLOBAL_POSITION_INT' not in self.get_data():
            raise Exception('no GLOBAL_POSITION_INT data')

        if 'ATTITUDE' not in self.get_data():
            raise Exception('no ATTITUDE data')

        msg = Odometry()

        self._create_header(msg)

        # https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED
        # local_position_data = self.get_data()['GLOBAL_POSITION_INT']
        local_position_data = self.get_data()['LOCAL_POSITION_NED']

        # xyz_data = [local_position_data[i] for i in ['lat', 'lon', 'alt']]
        xyz_data = [local_position_data[i] for i in ['x', 'y', 'z']]
        vxyz_data = [local_position_data[i] for i in ['vx', 'vy', 'vz']]

        # https://mavlink.io/en/messages/common.html#ATTITUDE
        attitude_data = self.get_data()['ATTITUDE']
        orientation = [attitude_data[i] for i in ['roll', 'pitch', 'yaw']]
        orientation_speed = [attitude_data[i] for i in ['rollspeed', 'pitchspeed', 'yawspeed']]
        # x, y = utm.from_latlon(xyz_data[0], xyz_data[1])
        msg.pose.pose.position.x = xyz_data[0]  # lat
        msg.pose.pose.position.y = xyz_data[1]  # lon
        msg.pose.pose.position.z = xyz_data[2]  # m
        msg.twist.twist.linear.x = vxyz_data[0] / 100  # m/s -> cm/s
        msg.twist.twist.linear.y = vxyz_data[1] / 100  # m/s -> cm/s
        msg.twist.twist.linear.z = vxyz_data[2] / 100  # m/s -> cm/s

        # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_Angles_to_Quaternion_Conversion
        cy = math.cos(orientation[2] * 0.5)
        sy = math.sin(orientation[2] * 0.5)
        cr = math.cos(orientation[0] * 0.5)
        sr = math.sin(orientation[0] * 0.5)
        cp = math.cos(orientation[1] * 0.5)
        sp = math.sin(orientation[1] * 0.5)
        # Orientation [rad] [-pi,pi] || Orientation Speed [rad/s]
        msg.pose.pose.orientation.w = cy * cr * cp + sy * sr * sp
        msg.pose.pose.orientation.x = cy * sr * cp - sy * cr * sp
        msg.pose.pose.orientation.y = cy * cr * sp + sy * sr * cp
        msg.pose.pose.orientation.z = sy * cr * cp - cy * sr * sp
        msg.twist.twist.angular.x = orientation_speed[0]
        msg.twist.twist.angular.y = orientation_speed[1]
        msg.twist.twist.angular.z = orientation_speed[2]

        self.pub_topics[topic][3].publish(msg)

    def _create_battery_msg(self, topic):
        """ Create battery message from ROV data

        Raises:
            Exception: No data available
        """

        # Check if data is available
        if 'SYS_STATUS' not in self.get_data():
            raise Exception('no SYS_STATUS data')

        if 'BATTERY_STATUS' not in self.get_data():
            raise Exception('no BATTERY_STATUS data')

        bat = BatteryState()
        self._create_header(bat)

        # http://docs.ros.org/jade/api/sensor_msgs/html/msg/BatteryState.html
        bat.voltage = self.get_data()['SYS_STATUS']['voltage_battery'] / 1000
        bat.current = self.get_data()['SYS_STATUS']['current_battery'] / 100
        bat.percentage = self.get_data()['BATTERY_STATUS']['battery_remaining'] / 100
        self.pub_topics[topic][3].publish(bat)

    def _create_rov_state(self, topic):
        """
        Create ROV state message from ROV data

        Raises:
            Exception: No data available
        """

        # Check if data is available
        if 'SERVO_OUTPUT_RAW' not in self.get_data():
            raise Exception('no SERVO_OUTPUT_RAW data')

        if 'HEARTBEAT' not in self.get_data():
            raise Exception('no HEARTBEAT data')

        servo_output_raw_msg = self.get_data()['SERVO_OUTPUT_RAW']
        servo_output_raw = [servo_output_raw_msg['servo{}_raw'.format(i + 1)] for i in range(8)]
        motor_throttle = [servo_output_raw[i] - 1500 for i in range(6)]
        # 1100 -> -1 and 2000 -> 1
        for throttle in motor_throttle:
            if throttle < 0:
                throttle = throttle / 400
            else:
                throttle = throttle / 500

        light_on = (servo_output_raw[6] - 1100) / 8
        # need to check
        camera_angle = servo_output_raw[7] - 1500

        # Create angle from pwm
        camera_angle = -45 * camera_angle / 400

        base_mode = self.get_data()['HEARTBEAT']['base_mode']
        custom_mode = self.get_data()['HEARTBEAT']['custom_mode']

        mode, arm = self.decode_mode(base_mode, custom_mode)
        data = State()
        data.arm = arm
        data.rc1 = motor_throttle[0]
        data.rc2 = motor_throttle[1]
        data.rc3 = motor_throttle[2]
        data.rc4 = motor_throttle[3]
        data.rc5 = motor_throttle[4]
        data.rc6 = motor_throttle[5]
        data.light = light_on
        data.camera = camera_angle
        data.mode = mode

        self.pub_topics[topic][3].publish(data)

    '''
    def _create_sensor_ft_msg(self, topic):
        """
        Create force state message from ROV data

        Raises:
            Exception: No data available
        """
        [fx, fy, fz, mx, my, mz] = self.get_data_force_torque_sensor()
        print([fx, fy, fz, mx, my, mz])
        msg = FTSensor()
        msg.f_x = fx
        msg.f_y = fy
        msg.f_z = fz
        msg.m_x = mx
        msg.m_y = my
        msg.m_z = mz
        self.pub_topics[topic][3].publish(msg)
    '''
    def _control_callback(self, msg):
        self.control_actions = [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z]

    def send_control_actions(self):
        # print(self.control_actions)
        # vel_x, vel_y, vel_z = self.control_actions[0], self.control_actions[1],self.control_actions[2]
        # omega_z = self.control_actions[3]
        x = np.interp(self.control_actions[0], [-100, 100], [1100, 1900])
        y = np.interp(self.control_actions[1], [-100, 100], [1100, 1900])
        z = np.interp(self.control_actions[2], [-100, 100], [1100, 1900])
        yaw = np.interp(self.control_actions[3], [-100, 100], [1100, 1900])
        # vel_x = max(-max_vel, min(max_vel, vel_x))
        # vel_y = max(-max_vel, min(max_vel, vel_y))
        # vel_z = max(-max_vel,min(max_vel, vel_z))
        # omega_z = max(-max_omega, min(max_omega, omega_z))
        # print(vel_x, vel_y, omega_z)
        # x = 1500 + int(round(vel_to_cmd * vel_x))
        # y = 1500 + int(round(vel_to_cmd * vel_y))
        # z = 1500 + int(round(vel_to_cmd * vel_z))
        # yaw = 1500 + int(round(omega_to_cmd * omega_z))
        # from [1100,1900]
        print(x, y, z, yaw)
        self.set_cmd_vel(x, y, z, yaw)

    def publish(self):
        """
        Publish the data in ROS topics
        """
        # self.wait_conn()
        self.update()
        # self.print_data()
        for topic, pubs in self.pub_topics.items():
            try:
                if time.time() - self.mavlink_msg_available[topic] > 1:
                    pubs[0](topic)
            except Exception as e:
                self.mavlink_msg_available[topic] = time.time()
                print(e)


def main():
    global bluerov

    rospy.init_node('bluerov_node', log_level=rospy.DEBUG)
    rate = rospy.Rate(20)
    try:
        while not rospy.is_shutdown():
            try:
                rospy.loginfo("Section -1 ")
                # bluerov = BlueRov(device='udp:192.168.2.1:14550', source_system=1, source_component=1)
                bluerov = BlueRov(device='udp:localhost:14450', source_system=1, source_component=1)
            except socket.error:
                rospy.logerr(
                    'Failed to make mavlink connection to device {}'.format(device)
                )
                rate.sleep()
            else:
                break

        if rospy.is_shutdown():
            sys.exit(-1)

        while not rospy.is_shutdown():
            rospy.loginfo("Section 1, MANUAL MODE, ARM FALSE ")
            bluerov.set_mode('MANUAL')
            bluerov.arm_throttle(False)
            mode, arm = bluerov.get_mode()
            if mode == 'MANUAL' and not arm:
                break
            rate.sleep()

        if armed:
            bluerov.arm_throttle(True)
        else:
            bluerov.arm_throttle(False)

        if depth_hold:
            bluerov.set_mode('GUIDED')
        else:
            bluerov.set_mode('MANUAL')

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            bluerov.wait_conn()
            # bluerov.update()
            bluerov.publish()
            bluerov.send_control_actions()
            # print(bluerov.control_actions)
            rospy.loginfo("Section 2, MANUAL MODE, ARM TRUE ")
            rate.sleep()

        rospy.on_shutdown(shutdown)
        rospy.loginfo("Section 3, MANUAL MODE, ARM FALSE ")
        while not rospy.is_shutdown():
            bluerov.set_mode('MANUAL')
            bluerov.arm_throttle(False)
            mode, arm = bluerov.get_mode()
            if mode == 'MANUAL' and not arm:
                break
            rate.sleep()

    except rospy.ROSInterruptException as error:
        print(" An Exception occurred during the control loop. Terminating gracefully.")
        print('pubs error with ROS: ', error)
        exit(1)


if __name__ == "__main__":
    main()
