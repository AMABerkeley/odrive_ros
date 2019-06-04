#!/usr/bin/env python
import odrive
import odrive.utils as o_utils
import sys
from odrive.enums import *
import time
import numpy as np

GLOBAL_DRIVE_MODE = CTRL_MODE_POSITION_CONTROL

class Odrive_Object:
    def __init__(self, port):
        self.cpr = 8192
        while True:
            try:
                self.driver = odrive.find_any(serial_number=port, timeout=30)
                break
            except:
                pass
        self.port = port
        self.engage()

        # self.drive_mode = CTRL_MODE_TRAJECTORY_CONTROL
        self.drive_mode = CTRL_MODE_POSITION_CONTROL

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

    def clear_errors(self, event):
        # Non-critical code for clearing error states. To be run with ros.timer
        o_utils.dump_errors(self.driver, clear=True)

    def drive_pos_single(self, val_rad, motor_number, mode=GLOBAL_DRIVE_MODE):
        # units of left and right are in radians
        axis = self.driver.axis0 if motor_number == 0 else self.driver.axis1
        try:
            # convert from radians to counts
            val_count =  float(val_rad)  * float(self.cpr) / float(2.0 * np.pi)
            axis.controller.config.control_mode = mode

            if self.drive_mode ==  CTRL_MODE_TRAJECTORY_CONTROL:
                axis.controller.move_to_pos(val_count)
            else:
                axis.controller.pos_setpoint = val_count
        except Exception as e:
            raise e

    def drive_pos(self, left, right):
        # units of left and right are in radians
        try:
            # convert from radians to counts
            left_des =  float(left)  * float(self.cpr) / float(2.0 * np.pi)
            right_des = float(right) * float(self.cpr) / float(2.0 * np.pi)
            self.driver.axis0.controller.config.control_mode = self.drive_mode
            self.driver.axis1.controller.config.control_mode = self.drive_mode

            if self.drive_mode ==  CTRL_MODE_TRAJECTORY_CONTROL:
                self.driver.axis0.controller.move_to_pos(left_des)
                self.driver.axis1.controller.move_to_pos(right_des)
            else:
                self.driver.axis0.controller.pos_setpoint = (left_des)
                self.driver.axis1.controller.pos_setpoint = (right_des)
        except Exception as e:
            raise e

    def torque_to_current(self, torque):
        return torque * 100.0  / 8.27

    def drive_torque(self, left, right):
        try:
            l_current = self.torque_to_current(left)
            r_current = self.torque_to_current(right)
            self.driver.axis0.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL
            self.driver.axis1.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL
            self.driver.axis0.controller.current_setpoint = l_current
            self.driver.axis1.controller.current_setpoint = r_current
        except Exception as e:
           raise e

    def drive_torque_single(self, torque, motor_number):
        try:
            axis = self.driver.axis0 if motor_number == 0 else self.driver.axis1
            axis.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL
            axis.controller.current_setpoint = self.torque_to_current(torque)
        except Exception as e:
           raise e
