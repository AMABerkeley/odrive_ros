#!/usr/bin/env python
import odrive
import odrive.utils as o_utils
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
        self.port = port

        self.engage()
        rospy.Subscriber("/jelly_hardware/odrives/" + str(port) + "/command", Float64MultiArray, self.command_callback)
        self.state_pub = rospy.Publisher("/jelly_hardware/odrives/" + str(port) + "/state", Float64MultiArray, queue_size=1)
        self.pos_setpoint = None
        #self.drive_mode = CTRL_MODE_TRAJECTORY_CONTROL
        self.drive_mode = CTRL_MODE_POSITION_CONTROL

    def command_callback(self, msg):
        cmd = msg.data
        if len(cmd) == 2:
            self.pos_setpoint = cmd
        else:
            rospy.logerr("Command callback with empty command message - defaulting to 0A current control")
            self.drive_current(0, 0)

    def engage(self):
        """
        Enter into closed loop current control, legs should go limp with some resistance
        TODO: perhaps rename from engage if it makes legs go limp
        """
        #self.logger.debug("Setting drive mode.")
        for axis in (self.driver.axis0, self.driver.axis1):
            axis.controller.vel_setpoint = 0.0
            axis.controller.pos_setpoint = 0.0
            axis.controller.current_setpoint = 0.0
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            axis.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL
        return True

    def disengage(self):
        for axis in (self.driver.axis0, self.driver.axis1):
            axis.requested_state = AXIS_STATE_IDLE


    def drive_pos(self, left, right):
        # units of left and right are in radians
        try:
            # convert from radians to counts
            left_des =  float(left)  * float(self.cpr) / float(2.0 * np.pi)
            right_des = float(right) * float(self.cpr) / float(2.0 * np.pi)
            #rospy.logerr("drive mode: %i" % (self.drive_mode))
            #rospy.logerr("drive_pos(%i, %i)" % (left_des, right_des,))
            #rospy.logerr("traj 0 %s | traj 1 %s"  % (str(self.driver.axis0.trap_traj), str(self.driver.axis1.trap_traj)))
            self.driver.axis0.controller.config.control_mode = self.drive_mode
            self.driver.axis1.controller.config.control_mode = self.drive_mode

            if self.drive_mode ==  CTRL_MODE_TRAJECTORY_CONTROL:
                self.driver.axis0.controller.move_to_pos(left_des)
                self.driver.axis1.controller.move_to_pos(right_des)
            else:
                self.driver.axis0.controller.pos_setpoint = (left_des)
                self.driver.axis1.controller.pos_setpoint = (right_des)


            rospy.logerr("drive_pos: "  + str([left, left_des, right, right_des]))


        except Exception as e:
            rospy.logerr("Exception in driver.drive_pos()")
            raise e

    def set_trajectory(self, traj_values):
        """
        Trajectory control value have units related to counts
        Velocity and acceleration limits are in units of CPR ticks

        default from double_herlea_[no_]brake_amp_traj.json
            vel: ~8*8192 cpr/sec, accel&decel = 2*8192 cpr/sec2
        {"vel_limit": 59000.0, "accel_limit": 16384.0, "decel_limit": 16384.0, "A_per_css": 0.0}
        """

        assert len(traj_values) == 4, "Trajectory values not 4 elements long"
        for axis in (self.driver.axis0, self.driver.axis1):
            axis.trap_traj.config.vel_limit = traj_values[0]
            axis.trap_traj.config.accel_limit = traj_values[1]
            axis.trap_traj.config.decel_limit = traj_values[2]
            axis.trap_traj.config.A_per_css = traj_values[3]

            axis.motor.config.current_lim = 6


    def process_pos_setpoint(self):
        if self.pos_setpoint is not None:
            self.drive_pos(self.pos_setpoint[0], self.pos_setpoint[1])
            self.pos_setpoint = None


    def clear_errors(self, event):
        # Non-critical code for clearing error states. To be run with ros.timer
        if port == "207C37863548" and self.driver.axis0.error:
            rospy.logerr(self.driver.axis0.error)
        o_utils.dump_errors(self.driver, clear=True)

    def publish_state(self):
        pos0 = self.driver.axis0.encoder.pos_estimate / float(self.cpr) * 2 * np.pi
        pos1 = self.driver.axis1.encoder.pos_estimate / float(self.cpr) * 2 * np.pi
        cur0 = self.driver.axis0.motor.current_control.Iq_measured
        cur1 = self.driver.axis1.motor.current_control.Iq_measured
        msg = Float64MultiArray()
        msg.data = [pos0, pos1, cur0, cur1]
        # units of published left and right are in radians
        self.state_pub.publish(msg)


    def drive_current(self, left, right):
        try:
            self.driver.axis0.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL
            self.driver.axis1.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL
            self.driver.axis0.controller.current_setpoint = left
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
    od.set_trajectory([200000.0, 200000.0, 200000.0, 0.0])
    od.engage()
    # od.drive_current(0, 0)
    # serial = "2069339B304B"
    # od = odrive_object(serial)

    rospy.Timer(rospy.Duration(2), od.clear_errors) # dumps errors every 2 seconds

    r = rospy.Rate(250)
    while not rospy.is_shutdown():
        od.publish_state()
        od.process_pos_setpoint()
        r.sleep()

    od.disengage()
