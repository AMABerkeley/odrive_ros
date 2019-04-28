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


ods = []

count = 3

#ids = ["206A3398304B","2069339E304B", "2069339B304B"]
ids = [None,None]

for i in ids:
    od = ODriveInterfaceAPI()
    od.connect(odrive_id=i)
    pdb.set_trace()
    od.engage()
    ods.append(od)


start_time = time.time()
cycle_count = 0
while True:
    cycle_count += 1
    print("Odrive is running at %i hz" % (cycle_count/(time.time() - start_time)))
    for od in ods:
        od.drive_pos(10, 10, None)
        h = od.driver.axis0.encoder.pos_cpr
        h = od.driver.axis1.encoder.pos_cpr
