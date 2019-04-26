#!/usr/bin/env python
from __future__ import print_function

#import roslib; roslib.load_manifest('BINCADDY')
import rospy
import tf.transformations
import tf_conversions
import tf2_ros

from std_msgs.msg import Float64, Int32
from jelly_msgs.msg import DriveCommand
import std_srvs.srv

import time
import math
import traceback
import Queue

from odrive_interface import ODriveInterfaceAPI, ODriveFailure

class ROSLogger(object):
    """Imitate a standard Python logger, but pass the messages to rospy logging.
    """
    def debug(self, msg):    rospy.logdebug(msg)  #  print(msg) #
    def info(self, msg):     rospy.loginfo(msg)   #  print(msg) #
    def warn(self, msg):     rospy.logwarn(msg)   #  print(msg) #
    def error(self, msg):    rospy.logerr(msg)    #  print(msg) #
    def critical(self, msg): rospy.logfatal(msg)  #  print(msg) #
    
class ODriveNode(object):
    driver = None
    prerolling = False
    
    # Robot wheel_track params for velocity -> motor speed conversion
    encoder_counts_per_rev = None
    axis_for_right = 0
    encoder_cpr = 8192 # TODO pass in as argument
    
    # Startup parameters
    connect_on_startup = False
    calibrate_on_startup = False
    engage_on_startup = False

    
    def __init__(self):
        self.main_loop_count = 0
        self.fast_loop_count = 0

        self.queue_drops = 0
        self.start_time = rospy.get_time()
        self.axis_for_right = float(rospy.get_param('~axis_for_right', 0)) # if right calibrates first, this should be 0, else 1
        
        self.connect_on_startup   = rospy.get_param('~connect_on_startup', True)
        self.calibrate_on_startup = rospy.get_param('~calibrate_on_startup', False)
        self.engage_on_startup    = rospy.get_param('~engage_on_startup', True)
        
        self.has_preroll          = rospy.get_param('~use_preroll', False)
                
        self.publish_current      = rospy.get_param('~publish_current', True)
        self.publish_raw_kinematics = rospy.get_param('~publish_raw_kinematics', True)
        
        self.odom_calc_hz    = rospy.get_param('~odom_calc_hz', 200)
        
        rospy.on_shutdown(self.terminate)

        rospy.Service('connect_driver',    std_srvs.srv.Trigger, self.connect_driver)
        rospy.Service('disconnect_driver', std_srvs.srv.Trigger, self.disconnect_driver)
    
        rospy.Service('calibrate_motors',  std_srvs.srv.Trigger, self.calibrate_motor)
        rospy.Service('engage_motors',     std_srvs.srv.Trigger, self.engage_motor)
        rospy.Service('release_motors',    std_srvs.srv.Trigger, self.release_motor)
        
        self.command_queue = Queue.Queue(maxsize=5)
        self.command_subscribe = rospy.Subscriber("/motor_cmd", DriveCommand, self.cmd_callback, queue_size=2)
        
        if self.publish_current:
            self.current_loop_count = 0
            self.left_current_accumulator  = 0.0
            self.right_current_accumulator = 0.0
            self.current_publisher_left  = rospy.Publisher('odrive/left_current', Float64, queue_size=2)
            self.current_publisher_right = rospy.Publisher('odrive/right_current', Float64, queue_size=2)
            rospy.loginfo("ODrive will publish motor currents.")
        
        self.last_cmd_time = rospy.Time.now()
        
        if self.publish_raw_kinematics:
            self.raw_kinematics_publisher_encoder_left  = rospy.Publisher('odrive/raw_kinematics/encoder_left',   Int32, queue_size=2)
            self.raw_kinematics_publisher_encoder_right = rospy.Publisher('odrive/raw_kinematics/encoder_right',  Int32, queue_size=2)
            self.raw_kinematics_publisher_vel_left      = rospy.Publisher('odrive/raw_kinematics/velocity_left',  Int32, queue_size=2)
            self.raw_kinematics_publisher_vel_right     = rospy.Publisher('odrive/raw_kinematics/velocity_right', Int32, queue_size=2)
            rospy.loginfo("ODrive will publish motor positions and velocities")
                            
        
        
    def main_loop(self):
        # Main control, handle startup and error handling
        # while a ROS timer will handle the high-rate (~50Hz) comms + odometry calcs
        main_rate = rospy.Rate(1) # hz
        self.main_loop_count += 1
        rospy.logerr("main loop freq" + str((self.main_loop_count)/(rospy.get_time() - self.start_time)))
        # Start timer to run high-rate comms
        self.fast_timer = rospy.Timer(rospy.Duration(1/float(self.odom_calc_hz)), self.fast_timer)
        
        self.fast_timer_comms_active = False
        
        while not rospy.is_shutdown():
            try:
                main_rate.sleep()
            except rospy.ROSInterruptException: # shutdown / stop ODrive??
                break
            
            # fast timer running, so do nothing and wait for any errors
            if self.fast_timer_comms_active:
                continue
            
            # check for errors
            if self.driver:
                try:
                    # driver connected, but fast_comms not active -> must be an error?
                    if self.driver.get_errors(clear=True):
                        rospy.logerr("Had errors, disconnecting and retrying connection.")
                        self.driver.disconnect()
                        self.driver = None
                    else:
                        # must have called connect service from another node
                        self.fast_timer_comms_active = True
                except:
                    rospy.logerr("Errors accessing ODrive:" + traceback.format_exc())
                    self.driver = None
            
            if not self.driver:
                if not self.connect_on_startup:
                    #rospy.loginfo("ODrive node started, but not connected.")
                    continue
                
                if not self.connect_driver(None)[0]:
                    rospy.logerr("Failed to connect.") # TODO: can we check for timeout here?
                    continue
            
            else:
                pass # loop around and try again
    def pub_state(self):
        self.vel_l = 0
        self.vel_r = 0
        self.new_pos_l = 0
        self.new_pos_r = 0
        self.current_l = 0
        self.current_r = 0
        
        # Handle reading from Odrive and sending odometry
        try:
            # read all required values from ODrive for odometry
            self.vel_l = self.driver.left_axis.encoder.vel_estimate  # units: encoder counts/s
            self.vel_r = -self.driver.right_axis.encoder.vel_estimate # neg is forward for right
            self.new_pos_l = self.driver.left_axis.encoder.pos_cpr    # units: encoder counts
            self.new_pos_r = -self.driver.right_axis.encoder.pos_cpr  # sign!
            
            # for current
            self.current_l = self.driver.left_axis.motor.current_control.Ibus
            self.current_r = self.driver.right_axis.motor.current_control.Ibus
            
        except:
            rospy.logerr("Fast timer exception reading:" + traceback.format_exc())
            self.fast_timer_comms_active = False

        
    def fast_timer(self, timer_event):
        time_now = rospy.Time.now()
        self.fast_loop_count += 1
        # rospy.logerr("fast loop freq" + str((self.fast_loop_count)/(rospy.get_time() - self.start_time)))
	# rospy.logerr(time_now)
        # in case of failure, assume some values are zero
        self.pub_state()

        if self.publish_current:
            self.pub_current()

        if self.publish_raw_kinematics:
            self.pub_raw_kinematics()
            

        # handle sending drive commands.

        # from here, any errors return to get out
        self.handle_queue_command()
    def handle_queue_command(self):
        if self.command_queue.empty():
            # check to see if we're initialised and engaged motor
          #  if not self.driver.prerolled():
          #      self.driver.preroll()
          #      return
            self.fast_timer_comms_active = False                
            
            try:
                motor_command = self.command_queue.get_nowait()
            except Queue.Empty:
                rospy.logerr("Queue was empty??" + traceback.format_exc())
                return
            
            try:
                control_type = motor_command[1]
                motor_num = motor_command[2]
                value = motor_command[3]
                trajectory = motor_command[4]
                if not self.driver.engaged():
                    self.driver.engage()
                left_val = value if motor_num == 0 else None
                right_val = value if motor_num == 1 else None
                if control_type == 0:
                    self.driver.drive_pos(left_val, right_val, trajectory)
                elif control_type == 1:
                    self.driver.drive_vel(left_val, right_val)
                elif control_type == 2:
                    self.driver.drive_current(left_val, right_val)
                self.last_cmd_time = time_now
    #                elif motor_command[0] == 'release':
    ###                   pass
        #            else:
        #               pass
    
            except:
                rospy.logerr("Fast timer exception on %s cmd: %s" % (str(motor_command), traceback.format_exc()))
                self.fast_timer_comms_active = False

        
    def terminate(self):
        self.fast_timer.shutdown()
        if self.driver:
            self.driver.release()
    
    # ROS services
    def connect_driver(self, request):
        if self.driver:
            return (False, "Already connected.")
        
        self.driver = ODriveInterfaceAPI(logger=ROSLogger())
        rospy.loginfo("Connecting to ODrive...")
        if not self.driver.connect(right_axis=self.axis_for_right):
            self.driver = None
            #rospy.logerr("Failed to connect.")
            return (False, "Failed to connect.")
            
        rospy.loginfo("ODrive connected.")
        self.start_time = rospy.get_time()
        
        # okay, connected, 
        self.fast_timer_comms_active = True
        
        return (True, "ODrive connected successfully")
    
    def disconnect_driver(self, request):
        if not self.driver:
            rospy.logerr("Not connected.")
            return (False, "Not connected.")
        try:
            if not self.driver.disconnect():
                return (False, "Failed disconnection, but try reconnecting.")
        except:
            rospy.logerr('Error while disconnecting: {}'.format(traceback.format_exc()))
        finally:
            self.driver = None
        return (True, "Disconnection success.")
    
    def calibrate_motor(self, request):
        if not self.driver:
            rospy.logerr("Not connected.")
            return (False, "Not connected.")
            
        if self.has_preroll:
            if not self.driver.preroll():
                return (False, "Failed preroll.")        
        else:
            if not self.driver.calibrate():
                return (False, "Failed calibration.")
                
        return (True, "Calibration success.")
                    
    def engage_motor(self, request):
        if not self.driver:
            rospy.logerr("Not connected.")
            return (False, "Not connected.")
        if not self.driver.engage():
            return (False, "Failed to engage motor.")
        return (True, "Engage motor success.")
    
    def release_motor(self, request):
        if not self.driver:
            rospy.logerr("Not connected.")
            return (False, "Not connected.")
        if not self.driver.release():
            return (False, "Failed to release motor.")
        return (True, "Release motor success.")
    
    
    # Helpers and callbacks
    
                
    def pub_current(self):
        current_quantizer = 5
        
        self.left_current_accumulator += self.current_l
        self.right_current_accumulator += self.current_r
    
        self.current_loop_count += 1
        if self.current_loop_count >= current_quantizer:
            self.current_publisher_left.publish(float(self.left_current_accumulator) / current_quantizer)
            self.current_publisher_right.publish(float(self.right_current_accumulator) / current_quantizer)
    
            self.current_loop_count = 0
            self.left_current_accumulator = 0.0
            self.right_current_accumulator = 0.0

    def pub_raw_kinematics(self):
        self.raw_kinematics_publisher_encoder_left.publish(self.new_pos_l)
        self.raw_kinematics_publisher_encoder_right.publish(self.new_pos_r)
        self.raw_kinematics_publisher_vel_left.publish(self.vel_l)
        self.raw_kinematics_publisher_vel_right.publish(self.vel_r)



    def cmd_callback(self, msg):
        # rospy.loginfo("Received a /DriveCommand message!")
        # rospy.loginfo("Control Type: %i, Motor Number: %i, value: %f, trajectory: %s" % (msg.control_type, msg.motor_num, msg.value, str(msg.trajectory)))

        control_type = msg.control_type
        motor_num = msg.motor_num
        value = msg.value
        trajectory = msg.trajectory if len(msg.trajectory) == 4 else None

        try:
            drive_command = ('drive', control_type, motor_num, value, trajectory)
            self.command_queue.put_nowait(drive_command)
        except Queue.Full:
            self.queue_drops += 1
            rospy.logerr("dropped queue rate drops/sec: " + str((self.queue_drops)/(rospy.get_time() - self.start_time)))
            pass
        self.last_cmd_time = rospy.Time.now()
    
def start_odrive():
    rospy.init_node('odrive')
    odrive_node = ODriveNode()
    odrive_node.main_loop()
    #rospy.spin() 
    
if __name__ == '__main__':
    try:
        start_odrive()
    except rospy.ROSInterruptException:
        pass
