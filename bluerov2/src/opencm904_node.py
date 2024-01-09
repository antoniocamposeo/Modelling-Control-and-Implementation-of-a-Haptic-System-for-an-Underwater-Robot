import rospy
import numpy as np
import time

# Message into folder of package
from bluerov2.msg import FTSensor
from std_msgs.msg import String, Int32MultiArray
from std_msgs.msg import Float32MultiArray


class OpenCM:
    def __init__(self):
        self.name_node = "opencm904"
        self.pub_topics = {
            '/motor_actual_position_raw':
                [
                    self._create_actual_motor_pos_msg,
                    Int32MultiArray,
                    1
                ],
            '/sensor_ft_data':
                [
                    self._create_sensor_msg,
                    FTSensor,
                    1
                ],
            '/board_status_reset':
                [
                    self._create_msg_status,
                    Int32MultiArray,
                    1
                ]
        }
        self.sub_topics = {
            '/data_opencm':
                [self._data_sensor_callback,
                 String,
                 1]
        }
        self.step_angle = 4093 / 360
        self.step_value = 360 / 4093
        self.MAX_STEP = 4093
        self.MIN_STEP = 0
        self.MAX_ANGLE = 360
        self.MIN_ANGLE = 0
        self.data = ""
        self.k = 0
        self.offset_bool = True
        self.offset = None
        self.motors_actual_position = [0.0,  # upper motor id : 1
                                       0.0]  # lower motor id : 2
        self.motors_desired_position = [0.0,  # upper motor id : 1
                                        0.0]  # lower motor id : 2
        self.reset_status = 0
        self.reset_status_board = 0
        self.sensor_ft_data_raw = np.array([0, 0, 0, 0, 0, 0])
        self.sensor_ft_data_conv = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.calib_mat = np.matrix([[-0.1, 0, 0, 3.1, 0.2, -3.4],
                                    [0, -3.7, -0.2, 1.8, 0, 1.9],
                                    [5.66667, 0, 6, 0, 6, 0],
                                    [0, 0, 0.0735632, 0, -0.0758621, 0],
                                    [-0.0804598, 0, 0.0436782, 0, 0.0436782, 0],
                                    [0, -0.0436782, 0, -0.0436782, 0.00229885, -0.0436782]])
        self.msg_available = {}
        for topic, pubs in self.pub_topics.items():
            self.msg_available[topic] = 0  # Control of transmission frequency
            pubs.append(rospy.Publisher(self.name_node + topic, pubs[1],
                                        queue_size=pubs[2]))  # add object publisher into each pub_topic
        for topic, subs in self.sub_topics.items():
            callback, msg_type, queue_size = subs
            rospy.Subscriber(topic, msg_type, callback, queue_size=queue_size)

    def _create_actual_motor_pos_msg(self, topic):
        msg = Int32MultiArray()
        msg.data = self.motors_actual_position
        self.pub_topics[topic][3].publish(msg)

    def _create_desired_motor_pos_state(self, topic):
        None

    def _create_msg_status(self, topic):
        msg = Int32MultiArray()
        msg.data = [self.reset_status, self.reset_status_board]
        self.pub_topics[topic][3].publish(msg)

    def _create_sensor_msg(self, topic):
        msg = FTSensor()
        msg.f_x = self.sensor_ft_data_conv[0]
        msg.f_y = self.sensor_ft_data_conv[1]
        msg.f_z = self.sensor_ft_data_conv[2]
        msg.m_x = self.sensor_ft_data_conv[3]
        msg.m_y = self.sensor_ft_data_conv[4]
        msg.m_z = self.sensor_ft_data_conv[5]
        self.pub_topics[topic][3].publish(msg)

    # def _motor_callback(self, msg):
    #     self.motors_actual_position = [self.convert_to_deg(x) for x in msg.data]

    def _data_sensor_callback(self, msg):
        if msg.data is not None:
            self.reset_status = 0
            self.data = msg.data  # String made of 36+/n char
            data_temp = self.data
            for i in range(0, 6):
                temp = data_temp[0:6]
                self.sensor_ft_data_raw[i] = float(temp)
                data_temp = data_temp[6:]

            self.motors_actual_position = [int(data_temp[0:4]) - 1000, int(data_temp[4:8]) - 1000]
            self.reset_status_board = int(data_temp[-1])
            self.data_sensor_conversion()
        elif msg.data is None:
            print("No MSG Available -- resetting the board")
            self.reset_status = 1

    def convert_to_val(self, angle):
        value = angle / self.step_angle
        if value >= self.MAX_STEP:
            value = self.MAX_STEP
        elif value < self.MIN_STEP:
            value = self.MIN_STEP
        return int(value)

    def convert_to_deg(self, value):
        angle = value * self.step_angle
        if angle >= self.MAX_ANGLE:
            angle = self.MAX_ANGLE
        elif angle < self.MIN_ANGLE:
            angle = self.MIN_ANGLE
        return angle

    def data_sensor_conversion(self):

        self.sensor_ft_data_raw = (self.sensor_ft_data_raw / 10000) - 25
        self.sensor_ft_data_conv = np.transpose(np.matmul(self.calib_mat, np.transpose(self.sensor_ft_data_raw)))

        if self.offset_bool is True:
            self.offset = self.sensor_ft_data_conv
            self.sensor_ft_data_conv = np.subtract(self.sensor_ft_data_conv, self.offset)
            print(self.offset)
            self.offset_bool = False
        else:
            self.sensor_ft_data_conv = np.subtract(self.sensor_ft_data_conv, self.offset)

    def publish(self):
        """
        Publish the data in ROS topics
        """
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
        rate = rospy.Rate(77)
        try:
            while not rospy.is_shutdown():
                self.publish()
                rate.sleep() # Test with rospy.spin()
        except Exception as e:
            print(" An Exception occurred during the control loop. Terminating gracefully.")
            print(" Exception: ", e)


if __name__ == "__main__":
    open_cm = OpenCM()
    open_cm.main()
