#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Int32MultiArray
"""
- Sinusoidal signals (I attach the Matlab program to create the references).
- 5 steps of amplitude 30 deg.
- 5 logarithmic chirps like the one from the other day.
"""
# array = np.loadtxt('/csv_files/sin_5.csv')
# array = np.loadtxt('/csv_files/sin_1.csv')
array = np.loadtxt('../catkin_ws/src/bluerov2/csv_files/logaritmic_chirp_3deg.csv')

array = array.T

step_angle = 4093 / 360  # 11.377777 convert from step to angle
step_value = 360 / 4093  # 0.087890625
reset_status_board = None
reset_status = None


def convert_to_deg(value):
    angle = round(value / step_angle, 2)
    # print(angle)
    if angle > 360.0:
        angle = 360.0
    elif angle < 0.0:
        angle = 0.0
    return angle


def convert_to_val(angle):
    value = round(angle / step_value, 2)
    # print(value)
    if value > 4093:
        value = 4093
    elif value < 0:
        value = 0
    return int(value)


def _status_callback(msg):
    global reset_status,reset_status_board
    reset_status_board = msg.data[1]
    reset_status = msg.data[0]


def main():
    pub = rospy.Publisher('motor_position', Int32MultiArray, queue_size=10)
    rospy.Subscriber('opencm904/board_status_reset', Int32MultiArray, _status_callback, queue_size=5)
    rospy.init_node('identification', anonymous=True)
    rate = rospy.Rate(77)  # 10hz
    msg_position = Int32MultiArray()
    start_position_motor_1 = 180
    finish_position_motor_1 = 210

    # msg_position.data = [0]
    t = 0
    i = 0
    while not rospy.is_shutdown():
        if t <= 500:
            print("waiting")
            msg_position.data = [convert_to_val(start_position_motor_1), convert_to_val(180),0]
        elif t > 500:
            #  if i < len(array) - 1:
            if i < len(array) - 1:
                msg_position.data = [convert_to_val(array[i]), convert_to_val(180),0]
                i = i + 1
            #  elif i >= len(array) - 1:
            elif i >= len(array) - 1:
                print("Stop Identification!!!")
                msg_position.data = [convert_to_val(start_position_motor_1), convert_to_val(180),0]
        print(f"t:{t},i:{i},data_val:{convert_to_val(array[i])},data_arr:{array[i]},status:{reset_status},status_board:{reset_status_board}")
        t = t + 1
        pub.publish(msg_position)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
