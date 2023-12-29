#!/usr/bin/env python3
"""
Implemented mavlink data parsing
Before using it, you need to change the ip and port, the default is bluerov2 .
"""
from pymavlink import mavutil


class MAVBridge(object):
    """
    Input: transfer protocol udp, listening ip, port (local, need to modify Ethernet settings)
    Output: all data transferred through mavlink.
    """

    def __init__(self, device='udpin:192.168.2.1:14550', baudrate=115200):
        """
        establish a connection
        """
        self.conn = mavutil.mavlink_connection(device, baud=baudrate)
        self.conn.wait_heartbeat()
        self.conn.mav.request_data_stream_send(self.conn.target_system, self.conn.target_component,
                                               mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
        self.data = {}

    def get_data(self):
        """ Get mavlink data in dict form
        Returns:
            TYPE: Dict
        """
        return self.data

    def update(self):
        """
        Updated data, stored in the dictionary data
        """
        # Get all messages
        msgs = self.get_all_msgs()
        # Update dict
        for msg in msgs:
            self.data[msg.get_type()] = msg.to_dict()

    def get_all_msgs(self):
        """
        Stores the received mavlink data into the array msgs
        """
        msgs = []
        while True:
            msg = self.conn.recv_match()
            if msg is not None:
                msgs.append(msg)
            else:
                break
        return msgs

    def print_data(self):
        """
        Print data for testing
        """
        print(self.data)

    def set_mode(self, mode):
        """
        Set the flight mode, it's not used here, default is manual.
        See: https://ardupilot.org/copter/docs/flight-modes.html
        """
        mode = mode.upper()
        if mode not in self.conn.mode_mapping():
            print('Unknown mode : {}'.format(mode))
            print('Try:', list(self.conn.mode_mapping().keys()))
            return
        mode_id = self.conn.mode_mapping()[mode]
        self.conn.set_mode(mode_id)

    def decode_mode(self, base_mode, custom_mode):
        """ Parsing Flight Mode and Locked Status from Heartbeat Packets

         Input:
            base_mode (TYPE): System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
            custom_mode (TYPE): A bitfield for use for autopilot-specific flags.

        Output:
            [str, bool]: Flight mode, lock state
        """
        flight_mode = ""

        mode_list = [
            [mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, 'MANUAL'],
            [mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED, 'STABILIZE'],
            [mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED, 'GUIDED'],
            [mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED, 'AUTO'],
            [mavutil.mavlink.MAV_MODE_FLAG_TEST_ENABLED, 'TEST']
        ]

        if base_mode == 0:
            flight_mode = "PreFlight"
        elif base_mode & mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED:
            flight_mode = mavutil.mode_mapping_sub[custom_mode]
        else:
            for mode_value, mode_name in mode_list:
                if base_mode & mode_value:
                    flight_mode = mode_name

        arm = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

        return flight_mode, arm

    def set_guided_mode(self):
        """
        Setting the boot mode
        """
        # https://github.com/ArduPilot/pymavlink/pull/128
        params = [mavutil.mavlink.MAV_MODE_GUIDED, 0, 0, 0, 0, 0, 0]
        self.send_command_long(mavutil.mavlink.MAV_CMD_DO_SET_MODE, params)

    def send_command_long(self, command, params=[0, 0, 0, 0, 0, 0, 0], confirmation=0):
        """ Send long commands

        Parameters:
            command (mavlink command): Command
            params (list, optional): param1, param2, ..., param7
            confirmation (int, optional): Confirmation value
        """
        self.conn.mav.command_long_send(
            self.conn.target_system,  # target system
            self.conn.target_component,  # target component
            command,  # mavlink command
            confirmation,  # confirmation
            params[0],  # params
            params[1],
            params[2],
            params[3],
            params[4],
            params[5],
            params[6]
        )

    def set_position_target_local_ned(self, param=[]):
        """ Send a SET_POSITION_TARGET_LOCAL_NED signal to set the target position
        Parameters.
            param (list, optional): param1, param2, ..., param11
        """
        if len(param) != 11:
            print('SET_POISITION_TARGET_GLOBAL_INT need 11 params')

        # Set mask
        mask = 0b0000000111111111
        for i, value in enumerate(param):
            if value is not None:
                mask -= 1 << i
            else:
                param[i] = 0.0

        self.conn.mav.set_position_target_local_ned_send(
            0,  # system time in milliseconds
            self.conn.target_system,  # target system
            self.conn.target_component,  # target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            mask,  # mask
            param[0], param[1], param[2],  # position x,y,z
            param[3], param[4], param[5],  # velocity x,y,z
            param[6], param[7], param[8],  # accel x,y,z
            param[9], param[10])  # yaw, yaw rate

    def set_attitude_target(self, param=[]):
        """
        Generates a SET_ATTITUDE_TARGET signal to set the target attitude
        Parameters:
            param (list, optional): param1, param2, ..., param7
        """
        if len(param) != 8:
            print('SET_ATTITUDE_TARGET need 8 params')

        # Set mask
        mask = 0b11111111
        for i, value in enumerate(param[4:-1]):
            if value is not None:
                mask -= 1 << i
            else:
                param[i + 3] = 0.0

        if param[7] is not None:
            mask += 1 << 6
        else:
            param[7] = 0.0

        q = param[:4]

        if q != [None, None, None, None]:
            mask += 1 << 7
        else:
            q = [1.0, 0.0, 0.0, 0.0]

        self.conn.mav.set_attitude_target_send(0,  # system time in milliseconds
                                               self.conn.target_system,  # target system
                                               self.conn.target_component,  # target component
                                               mask,  # mask
                                               q,  # quaternion attitude
                                               param[4],  # body roll rate
                                               param[5],  # body pitch rate
                                               param[6],  # body yaw rate
                                               param[7])  # thrust

    def set_servo_pwm(self, id, pwm=1500):
        """ setting the pwm of a servo motor

        Parameters:
            id (int): servo motor id
            pwm (int, optional): pwm range is 1100-2000
        """
        mavutil.mavfile.set_servo(self.conn, id, pwm)

    def set_rc_channel_pwm(self, id, pwm=1500):
        """ Setting the pwm of RC Channel, you can control the movement of
            ROV such as up and down, forward and backward.

        Args:
            id (TYPE): (communications) channel id
            pwm (int, optional):  pwm range is 1100-2000
        """
        rc_channel_values = [65535 for _ in range(8)]  # 8 for mavlink1
        rc_channel_values[id] = pwm
        self.conn.mav.rc_channels_override_send(
            self.conn.target_system,  # target_system
            self.conn.target_component,  # target_component
            *rc_channel_values)  # RC channel list, in microseconds.

    def set_manual_control(self, joy_list=[0] * 4, buttons_list=[0] * 16):
        """ Send manual control signal
        Set a MANUAL_CONTROL message for dealing with more control with ArduSub
        for now it is just to deal with lights under test...
        """
        x, y, z, r = 0, 0, 0, 0  # 32767,32767,32767,32767
        b = 0
        for i in range(len(buttons_list)):
            b = b | (buttons_list[i] << i)
        print("MANUAL_CONTROL_SEND : x : {}, y : {}, z : {}, r : {}, b : {}".format(x, y, z, r, b))
        # https://mavlink.io/en/messages/common.html MANUAL_CONTROL ( #69 )
        self.conn.mav.manual_control_send(
            self.conn.target_system,
            x,
            y,
            z,
            r,
            b)

    def arm_throttle(self, arm_throttle):
        """ Modify lock status
        Args:
            arm_throttle (bool): Arm state
        """
        if arm_throttle:
            self.conn.arducopter_arm()
        else:
            # http://mavlink.org/messages/common#MAV_CMD_COMPONENT_ARM_DISARM
            # param1 (0 to indicate disarm)
            # Reserved (all remaining params)
            self.send_command_long(
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                [0, 0, 0, 0, 0, 0, 0]
            )
