#!/usr/bin/python3
import rospy
import numpy as np
import time
from geometry_msgs.msg import Twist

def main():
    rospy.init_node("controller")

    cmd_topic = '/cmd_vel'
    rate = rospy.Rate(50)
    pub = rospy.Publisher(cmd_topic, Twist, queue_size=10)
    move_cmd = Twist()

    try:
        while not rospy.is_shutdown():
            move_cmd.linear.x = 0.1
            move_cmd.angular.z = 0.2
            print(move_cmd)
            pub.publish(move_cmd)
            time.sleep(0.1)
    except Exception as e:
        print(" An Exception occurred during the control loop. Terminating gracefully.")
        print(" Exception: ", e)

if __name__ == "__main__":
    main()