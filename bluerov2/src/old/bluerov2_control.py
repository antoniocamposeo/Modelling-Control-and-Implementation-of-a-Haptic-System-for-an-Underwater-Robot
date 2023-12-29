#!/usr/bin/env python3
import rospy

from geometry_msgs.msg import Twist
import sys

sys.path.append("/home/antonio/catkin_ws/src/bluerov2")
from src.bridge_mavlink_1 import Bridge

cmd_vel_enabled = True
armed = True
depth_hold = False
x, y, z, yaw = 1, 1, 1, 1
move_cmd = Twist()


def cmd_vel_sub(msg):
    # if not cmd_vel_enabled:
    #    return
    global x, y, z, yaw
    print(msg)
    vel_x, vel_y = msg.linear.x, msg.linear.y
    omega_z = msg.angular.z

    vel_x = max(-max_vel, min(max_vel, vel_x))
    vel_y = max(-max_vel, min(max_vel, vel_y))
    omega_z = max(-max_omega, min(max_omega, omega_z))

    x = 1000 + int(vel_to_cmd * vel_x)
    y = 1000 + int(vel_to_cmd * vel_y)
    z = 65535
    yaw = 1000 + int(omega_to_cmd * omega_z)


def set_control_action(v, w):
    move_cmd.linear.x = v
    move_cmd.angular.z = w




if __name__ == '__main__':
    rospy.init_node('bluerov_control_node')
    r = rospy.Rate(50)
    # joy_sub = rospy.Subscriber('/joy', Joy, joy_callback, queue_size=10)
    cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, cmd_vel_sub)

    translation_limit = rospy.get_param('~translation_limit', 100)
    rotation_limit = rospy.get_param('~rotation_limit', 80)
    max_vel = rospy.get_param('~max_vel', 0.2)
    max_omega = rospy.get_param('~max_omega', 0.15)
    vel_to_cmd = translation_limit / max_vel
    omega_to_cmd = rotation_limit / max_omega

    device = 'udp:192.168.2.1:14550'
    while not rospy.is_shutdown():
        try:
            bridge = Bridge(device)
        except socket.error:
            rospy.logerr(
                'Failed to make mavlink connection to device {}'.format(device)
            )
            rospy.sleep(1.0)
        else:
            break
    if rospy.is_shutdown():
        sys.exit(-1)

    # bridge.wait_conn()


    while not rospy.is_shutdown():
        print(1)
        bridge.set_mode('manual')
        bridge.arm_throttle(False)
        mode, arm = bridge.get_mode()
        if mode == 'MANUAL' and not arm:
            break
        #set_control_action(1,1)
        #cmd_vel_pub.publish(move_cmd)
        #print(bridge.get_cmd_vel())
        print(1)

        rospy.sleep(0.5)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        bridge.update()

        if armed:
            bridge.arm_throttle(True)
        else:
            bridge.arm_throttle(False)

        if depth_hold:
            bridge.set_mode('alt_hold')
        else:
            bridge.set_mode('manual')

        # bridge.set_cmd_vel(x, y, z, yaw)
        print(x, y, z, yaw)
        #set_control_action(1,1)

        #cmd_vel_pub.publish(move_cmd)
        print(2)
        rate.sleep()

    while not rospy.is_shutdown():
        bridge.set_mode('manual')
        mode, arm = bridge.arm_throttle(False)
        if mode == 'MANUAL' and not arm:
            break
        rospy.sleep(0.5)
