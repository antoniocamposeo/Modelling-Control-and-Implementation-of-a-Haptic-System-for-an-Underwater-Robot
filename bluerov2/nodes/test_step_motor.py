#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np



from std_msgs.msg import Int32MultiArray

step_angle = 4095 / 360  # 11.377777 convert from step to angle
step_value = 360 / 4095  # 0.087890625


def convert_to_deg(value):
    angle = round(value / step_angle, 2)
    if angle > 360.0:
        angle = 360.0
    elif angle < 0.0:
        angle = 0.0
    return angle


def convert_to_val(angle):
    value = round(angle / step_value, 2)
    if value > 4095:
        value = 4095
    elif value < 0:
        value = 0
    return int(value)


def talker():
    pub = rospy.Publisher('motor_position', Int32MultiArray, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(99)  # 10hz
    msg_position = Int32MultiArray()
    data_to_send = np.arange(180, 230,6)
    # msg_position.data = [0]
    i = 0
    k = 0
    t = 0
    h = 0
    while not rospy.is_shutdown():

        '''if 90 <= i < 270 and k == 0:
            msg_position.data = [convert_to_val(i), convert_to_val(180)]
            i = i + 1
            print(1)
        elif i == 270 and k == 0:
            k = 1
            print(2)
        elif i == 90 and k == 1:
            k = 0
            print(3)
        elif 270 >= i > 90 and k == 1:
            msg_position.data = [convert_to_val(i), convert_to_val(i)]
            i = i - 1
            print(4)
        print(i)'''
        if t <= 1000:
            print("waiting")
            msg_position.data = [convert_to_val(180), convert_to_val(180)]
        elif t > 1000:
            if h >= len(data_to_send):
                print("Stop Identification!!!")
                msg_position.data = [convert_to_val(180), convert_to_val(180)]
                break
            if i < 2000:
                if 0 <= k <= 100:
                    msg_position.data = [convert_to_val(data_to_send[h]), convert_to_val(180)]
                    k = k + 1
                elif k > 100:
                    k = 0
                    h = h + 1
                i = i + 1
            elif i >= 2100 :
                print("Stop Identification!!!")
                msg_position.data = [convert_to_val(180), convert_to_val(180)]

        t = t + 1
        pub.publish(msg_position)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass



#
#
#
#
#
#
