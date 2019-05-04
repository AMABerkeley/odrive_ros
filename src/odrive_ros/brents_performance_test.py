import time
from odrive_interface import ODriveInterfaceAPI
import serial
from serial.serialutil import SerialException

from copy import deepcopy

import sys
import time
import logging
import traceback

import odrive
from odrive.enums import *
from odrive.utils import dump_errors
import pdb;


odrive_id = sys.argv[1]
print("x")
od = ODriveInterfaceAPI()
print("x")
od.connect(odrive_id=odrive_id, timeout=50)
print("x")
od.engage()

# Current control mode
od.driver.axis0.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL
time.sleep(0.1)
od.driver.axis1.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL
time.sleep(0.1)

previous_time = time.time()
exit()
while True:
    od.driver.axis0.controller.current_setpoint = 0.2
    od.driver.axis1.controller.current_setpoint = 0.2
    pos_a = od.driver.axis0.encoder.pos_cpr
    pos_b = od.driver.axis1.encoder.pos_cpr

    current_time = time.time()
    freq = 1.0 / (current_time - previous_time)
    print("Frequency: {:<20} Pos A: {:<20} Pos B: {:<20}".format(freq, pos_a, pos_b))
    previous_time = current_time

