import serial
from odrive.enums import *
from odrive import *
import time
from serial import SerialException
# No longer tested; code moved out here in case it's of use to anyone.

class ODriveInterfaceSerial(object):
    port = None
    
    def __init__(self, logger=None):
        self.logger = logger
        
    def connect(self, port):
        for retry in range(4):
            try:
                self.port = serial.Serial(port)
            except SerialException as e:
	        raise
            time.sleep(5)
            if self.port:
                break
	self.port.timeout = 0.5
        
    def __del__(self):
        self.release()
        
    def setup(self):
        if not self.port:
            return
            
        self.port.write('r vbus_voltage\n')
        voltage = self.port.read(100).strip()
        voltage = voltage if voltage else "unknown"
        
        self.port.write('r vbus_voltage\n')
        voltage = self.port.read(100).strip()
        voltage = voltage if voltage else "unknown"

        self.port.write('w axis1.requested_state 3\n')
        time.sleep(20)

        self.port.write('w axis0.requested_state 3\n')
        time.sleep(20)
        
    # https://github.com/madcowswe/ODrive/blob/1294ddff1dd0619e9f098ce12ca0936670a5b405/tools/odrive/enums.py
    # w axis1.requested_state 3 # calibrate
    # w axis1.requested_state 8 # closed loop control
    # w axis1.controller.config.control_mode 2 # velocity control
    # w axis1.controller.vel_setpoint 500
    # w axis1.requested_state 1
        
    def engage(self):
        if not self.port:
            return
            
        self.port.write('w axis0.requested_state 8\n')
        time.sleep(0.01)
        self.port.write('w axis1.requested_state 8\n')
        time.sleep(0.01)

        self.port.write('w axis0.controller.config.control_mode 2\n')
        time.sleep(0.01)
        self.port.write('w axis1.controller.config.control_mode 2\n')
        time.sleep(0.01)
        
        self.port.write('w axis0.controller.vel_setpoint 0\n')
        time.sleep(0.01)
        self.port.write('w axis1.controller.vel_setpoint 0\n')
        time.sleep(0.01)
        
    def release(self):
        self.port.write('w axis0.requested_state %d\n' % AXIS_STATE_IDLE)
        time.sleep(0.01)
        self.port.write('w axis1.requested_state %d\n' % AXIS_STATE_IDLE)
        time.sleep(0.01)
            
    def drive_vel(self, left_motor_val, right_motor_val):
        if not self.port:
            return
            
        self.port.write('w axis0.controller.vel_setpoint %d\n' % right_motor_val)
        time.sleep(0.01)
        self.port.write('w axis1.controller.vel_setpoint %d\n' % left_motor_val)
        time.sleep(0.01)
