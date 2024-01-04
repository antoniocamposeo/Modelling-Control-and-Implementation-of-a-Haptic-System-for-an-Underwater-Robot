#!/usr/bin/python3

import rospy
import numpy as np
import time
# import sys

# sys.path.append("/home/antonio/catkin_ws/src/bluerov2")

from geometry_msgs.msg import Twist,Accel
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState

# Message into folder of package
from bluerov2.msg import State

from tf.transformations import euler_from_quaternion
from bluerov2.src.bluerov2_bridge_mavlink import Bridge


class BlueRovControl:
    def __init__(self):
        self.ROV_name = '/BlueRov2_control'  
        self.model_base_link = '/base_link' 
        self.control_actions = [0, 0, 0, 0]
        self.move_cmd = Twist()
        self.ROV_data = {'Battery': {'Voltage': None,
                                     'Current': None,
                                     '%': None},
                         'State': {'Arm': None,
                                   'rc1': None, 'rc2': None, 'rc3': None, 'rc4': None, 'rc5': None, 'rc6': None,
                                   'Light': None,
                                   'Camera': None,
                                   'Mode': None},
                         'Odometry': {'Position': [0, 0, 0],
                                      'Orientation': [0, 0, 0],
                                      'Velocity': [0, 0, 0],
                                      'Angular_velocity': [0, 0, 0]}}
        self.pub_topics = {'/cmd_vel':
                               [self.create_control_msg,
                                Twist,
                                1
                                ]}
        self.sub_topics = {
            '/BlueRov2/battery':
                [
                    self._battery_callback,
                    BatteryState,
                    1
                ],
            '/BlueRov2/state':
                [
                    self._state_callback,
                    State,
                    1
                ],
            '/BlueRov2/odometry':
                [
                    self._odometry_callback,
                    Odometry,
                    1
                ],
            '/keyboard_teleop':
                [
                    self._command_teleop_callback,
                    Twist,
                    1
                 ]
        }
        for topic, pubs in self.pub_topics.items():
            pubs.append(rospy.Publisher(topic, pubs[1],
                                        queue_size=pubs[2]))  # add object publisher into each pub_topic
        for topic, subs in self.sub_topics.items():
            callback, msg_type, queue_size = subs
            rospy.Subscriber(topic, msg_type, callback, queue_size=queue_size)

    def _battery_callback(self, msg):
        self.ROV_data['Battery']['Voltage'] = msg.voltage
        self.ROV_data['Battery']['Current'] = msg.current
        self.ROV_data['Battery']['%'] = msg.percentage

    def _state_callback(self, msg):
        self.ROV_data['State']['Arm'] = msg.arm
        self.ROV_data['State']['rc1'] = msg.rc1
        self.ROV_data['State']['rc2'] = msg.rc2
        self.ROV_data['State']['rc3'] = msg.rc3
        self.ROV_data['State']['rc4'] = msg.rc4
        self.ROV_data['State']['rc5'] = msg.rc5
        self.ROV_data['State']['rc6'] = msg.rc6
        self.ROV_data['State']['Light'] = msg.light
        self.ROV_data['State']['Camera'] = msg.camera
        self.ROV_data['State']['Mode'] = msg.mode

    def _odometry_callback(self, msg):
        self.ROV_data['Odometry']['Position'] = [msg.pose.pose.position.x,
                                                 msg.pose.pose.position.y,
                                                 msg.pose.pose.position.z
                                                 ]
        orientation = np.array(euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]))
        self.ROV_data['Odometry']['Orientation'] = [orientation[0], orientation[1], orientation[2]]

        self.ROV_data['Odometry']['Velocity'] = [msg.twist.twist.linear.x,
                                                 msg.twist.twist.linear.y,
                                                 msg.twist.twist.linear.z
                                                 ]

        self.ROV_data['Odometry']['Angular_velocity'] = [msg.twist.twist.angular.x,
                                                         msg.twist.twist.angular.y,
                                                         msg.twist.twist.angular.z
                                                         ]

    def _command_teleop_callback(self,msg):
        self.control_actions[0] = msg.linear.x
        self.control_actions[1] = msg.linear.y
        self.control_actions[2] = msg.linear.z
        self.control_actions[3] = msg.angular.z

    def create_control_msg(self, topic):
        self.move_cmd.linear.x = self.control_actions[0]
        self.move_cmd.linear.y = self.control_actions[1]
        self.move_cmd.linear.z = self.control_actions[2]
        self.move_cmd.angular.z = self.control_actions[3]
        self.pub_topics[topic][3].publish(self.move_cmd)

def main():
    # device='udpin:0.0.0.0:14550'
    # device='udpin:192.168.2.1:14550'
    rospy.init_node("bluerov2_controller")
    BR2_control = BlueRovControl()
    rate = rospy.Rate(20)
    try:
        while not rospy.is_shutdown():
            print(BR2_control.control_actions)
            BR2_control.create_control_msg('/cmd_vel')
            rate.sleep()
    except Exception as e:
        print(" An Exception occurred during the control loop. Terminating gracefully.")
        print(" Exception: ", e)


if __name__ == "__main__":
    main()
