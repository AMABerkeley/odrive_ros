#!/usr/bin/env python
import b
import sys
import time
import rospy
import numpy as np
from odrive_driver import Odrive_Object
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from std_msgs.msg import String

def command_callback(msg):
    # (m0 command, m0 ctr_mode, m1 command, m1 ctrl_mode)
    cmd = msg.data
    if len(cmd) == 4:
        cmd_setpoint = cmd
    else:
        rospy.logerr("Command callback with command message of %i length" % (len(msg),))

def process_cmd_setpoint(od):
    if cmd_setpoint is not None:
        cmd = cmd_setpoint.copy() # make a copy of the command
        # motor 0
        m0_val = cmd[0]
        m0_mode = round(cmd[1])

        m1_val = cmd[2]
        m1_mode = round(cmd[3])

        if m0_mode == CTRL_MODE_CURRENT_CONTROL:
            od.drive_torque_single(m0_val, 0)
        else:
            od.drive_pos_single(m0_val, 0)

        # motor 1
        if m1_mode == CTRL_MODE_CURRENT_CONTROL:
            od.drive_torque_single(m1_val, 1)
        else:
            od.drive_pos_single(m1_val, 1)
        cmd_setpoint = None

def publish_state_msg(odrive):
        pos0 = odrive.driver.axis0.encoder.pos_estimate / float(odrive.cpr) * 2 * np.pi
        pos1 = odrive.driver.axis1.encoder.pos_estimate / float(odrive.cpr) * 2 * np.pi
        cur0 = odrive.driver.axis0.motor.current_control.Iq_measured
        cur1 = odrive.driver.axis1.motor.current_control.Iq_measured

        msg = Float64MultiArray()
        msg.data = [pos0, pos1, cur0, cur1]
        # units of published left and right are in radians
        return msg

if __name__ == '__main__':
    cmd_setpoint = None
    port = None

    rospy.init_node('node1', anonymous=True)
    rospy.logerr("port:")
    rospy.logerr(sys.argv[1])
    port = sys.argv[1]
    od = Odrive_Object(port)
    od.set_trajectory([200000.0, 200000.0, 200000.0, 0.0])
    od.engage()

    rospy.Timer(rospy.Duration(1), od.clear_errors) # dumps errors every 1 seconds

    rospy.Subscriber("/jelly_hardware/odrives/" + str(port) + "/command", Float64MultiArray, command_callback)
    state_pub = rospy.Publisher("/jelly_hardware/odrives/" + str(port) + "/state", Float64MultiArray, queue_size=1)

    r = rospy.Rate(250)
    while not rospy.is_shutdown():
        state_pub.publish(publish_state_msg(od))
        process_cmd_setpoint(od)
        r.sleep()

    od.disengage()
