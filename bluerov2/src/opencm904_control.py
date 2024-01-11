import rospy
import numpy as np
import time
import sys

sys.path.append("/home/uclm/catkin_ws/src/bluerov2")

from src.opencm904_node import OpenCM
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from bluerov2.msg import FTSensor
from geometry_msgs.msg import Twist

class OpenCM_Control(OpenCM):
    def __init__(self):
        super(OpenCM, self).__init__()
        self.name_node = 'opencm904_control'
        self.motors_desired_position = [0, 0]
        self.teleop_value = [0.0, 0.0]
        self.step_angle = 0.088
        self.reset_status = None
        self.MAX_STEP = 4095
        self.MIN_STEP = 0
        self.MAX_ANGLE = 360
        self.MIN_ANGLE = 0
        self.pub_topics = {
            '/motor_position':
                [
                    self._create_desired_motor_pos_state,
                    Int32MultiArray,
                    10
                ]
        }
        self.sub_topics = {
            '/opencm904/motor_actual_position':
                [self._motor_opencm_callback,
                 Float32MultiArray,
                 2],
            '/opencm904/sensor_ft_data':
                [self._data_sensor_conv_callback,
                 FTSensor,
                 2],
            '/opencm904/control':
                [self._motor_opencm_teleop_callback,
                 Twist,
                 2],
            '/opencm904/board_status_reset':
                [self._status_board_callback,
                 Int32MultiArray,
                 2]

        }
        self.msg_available = {}
        for topic, pubs in self.pub_topics.items():
            self.msg_available[topic] = 0  # Control of transmission frequency

            pubs.append(rospy.Publisher(topic, pubs[1],
                                        queue_size=pubs[2]))  # add object publisher into each pub_topic
        for topic, subs in self.sub_topics.items():
            callback, msg_type, queue_size = subs
            rospy.Subscriber(topic, msg_type, callback, queue_size=queue_size)

    def _create_desired_motor_pos_state(self, topic):
        """
        Create and Publish data of motor desired position 
        Set the reset status of the OpenCM9.04 
        """
        msg = Int32MultiArray()
        self.motors_desired_position = [self.convert_to_val(self.teleop_value[0]),
                                        self.convert_to_val(self.teleop_value[1])]
        if self.reset_status == 0:
            msg.data = [self.motors_desired_position[0], self.motors_desired_position[1], 0]
        elif self.reset_status == 1:
            msg.data = [self.motors_desired_position[0], self.motors_desired_position[1], 1]
        self.pub_topics[topic][3].publish(msg)

    def _motor_opencm_callback(self, msg):
        """
        Callback function to store motor actual position
        """
        self.motors_actual_position = msg.data

    def _status_board_callback(self, msg):
        """
        Callback function to store the status of the board
        """
        self.reset_status_board = msg.data[1]
        self.reset_status = msg.data[0]

    def _data_sensor_conv_callback(self, msg):
        """
        Callback function to store sensor data
        """
        self.sensor_ft_data = [msg.f_x, msg.f_y, msg.f_z, msg.m_x, msg.m_y, msg.m_z]

    def _motor_opencm_teleop_callback(self, msg):
        """
        Callback function to store control via keyboard 
        """
        self.teleop_value = [msg.angular.z, msg.angular.x]

    def control_algorithm(self):
        """
        Function to be implemented, use for the control action 
        """
        None

    def publish(self):
        for topic, pubs in self.pub_topics.items():
            try:
                if time.time() - self.msg_available[topic] > 1:
                    pubs[0](topic)
            except Exception as e:
                self.msg_available[topic] = time.time()

                print(" An Exception occurred during the publish. Terminating gracefully.")
                print(" Exception: ", e)

    def main(self):
        rospy.init_node(self.name_node)
        rate = rospy.Rate(50)
        try:
            while not rospy.is_shutdown():
                self.publish()
                rate.sleep()
        except Exception as e:
            print(" An Exception occurred during the control loop. Terminating gracefully.")
            print(" Exception: ", e)


if __name__ == "__main__":
    open_cm_control = OpenCM_Control()
    open_cm_control.main()
