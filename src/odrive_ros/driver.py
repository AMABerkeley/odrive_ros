#!/usr/bin/env python
import odrive
import sys
from odrive.enums import *
import time
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from std_msgs.msg import String

class odrive_object:
    def __init__(self, port):
        self.cpr = 8192
        while True:
            try:
                self.driver = odrive.find_any(serial_number=port, timeout=30)

                rospy.logerr("connected to odrive: "+ str(port))
                break
            except:
                pass
            rospy.logerr("trying to connect again to port: "+ str(port))

        self.engage()
        rospy.Subscriber("/jelly_hardware/odrives/" + str(port) + "/command", Float64MultiArray, self.command_callback)
        self.state_pub = rospy.Publisher("/jelly_hardware/odrives/" + str(port) + "/state", Float64MultiArray, queue_size=1)
        self.drive_current(0, 0)

    def command_callback(self, msg):
        cmd = msg.data
        if len(cmd) == 2:
            self.drive_pos(cmd[0], cmd[1])
        else:
            self.drive_current(0, 0)

    def engage(self):
        """
        Enter into
        """
        #self.logger.debug("Setting drive mode.")
        for axis in (self.driver.axis0, self.driver.axis1):
            axis.controller.vel_setpoint = 0
            axis.controller.pos_setpoint = 0
            axis.controller.current_setpoint = 0
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            axis.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
        return True

    def drive_pos(self, left, right, trajectory=None):
        # units of left and right are in radians
        try:
            mode = CTRL_MODE_POSITION_CONTROL if trajectory is None else CTRL_MODE_TRAJECTORY_CONTROL


            # convert from radians to counts
            self.driver.axis0.controller.config.control_mode = mode
            self.driver.axis0.controller.pos_setpoint = left * float(self.cpr) / (2 * np.pi)
            self.driver.axis1.controller.config.control_mode = mode
            self.driver.axis1.controller.pos_setpoint = right * float(self.cpr) / (2 * np.pi)

            print("ok")
        except Exception as e:
           raise e

    def publish_position(self):
        pos0 = self.driver.axis0.encoder.pos_estimate / float(self.cpr) * 2 * np.pi
        pos1 = self.driver.axis1.encoder.pos_estimate / float(self.cpr) * 2 * np.pi
        msg = Float64MultiArray()
        msg.data = [pos0, pos1]
        # units of published left and right are in radians
        self.state_pub.publish(msg)



    # def set_trajectory(self, traj_config, traj_values):
        # """
        # Trajectory control value have units related to counts
        # """
#
        # assert len(traj_values) == 4, "Trajectory values not 4 elements long"
        # traj_config.vel_limit = traj_values[0]
        # traj_config.accel_limit = traj_values[1]
        # traj_config.decel_limit = traj_values[2]
        # traj_config.A_per_css = traj_values[3]

    def drive_current(self, left, right):
        try:
            self.driver.axis0.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL
            self.driver.axis0.controller.current_setpoint = left
            self.driver.axis1.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL
            self.driver.axis1.controller.current_setpoint = right
        except Exception as e:
           raise e

if __name__ == '__main__':
    rospy.init_node('node1', anonymous=True)
    port = None
    rospy.logerr("port:")
    rospy.logerr(sys.argv[1])
    port = sys.argv[1]
    od = odrive_object(port)
    # serial = "2069339B304B"
    # od = odrive_object(serial)
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        od.publish_position()
        r.sleep()
