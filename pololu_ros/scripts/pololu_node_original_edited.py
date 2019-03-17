#!/usr/bin/python
from __future__ import division
from pololu_driver import clip, Pololu # serial controller for Pololu

import rospy
import threading
from std_msgs.msg import Bool, Float32
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# TODO: if necessary, add more try and excepts for error catching



class FreshVal(object):
    """Holds a value and returns the timeout val if the data gets old"""
    def __init__(self, stale_val=0, timeout=None, time=rospy.Time(), name=""):
        if timeout is None:
            raise ValueError("timeout must be set")
        self.timeout = timeout
        self.stale_val = stale_val
        self._val = self.stale_val # initialize the value to stale
        self.last_update_time = time
        self.name = name

        self._has_shown_message = False

    def update(self, val):
        self._val = val 
        self.last_update_time = rospy.Time.now()

    @property
    def is_fresh(self):
        fresh = True
        fresh = fresh and self._val is not None
        fresh = fresh and (rospy.Time.now() - self.last_update_time).to_sec() < self.timeout
        return fresh

    @property
    def val(self):
        # if stale, return timeout_val
        if (rospy.Time.now() - self.last_update_time).to_sec() > self.timeout:
            if not self._has_shown_message: 
                rospy.logdebug("[POLOLU]: %s value timed out %d", self.name, self._val)
                self._has_shown_message = True
            return self.stale_val
        else:
            self._has_shown_message = False
            return self._val

class Node(object):
    def __init__(self):
        """Init ros node"""
        rospy.init_node("pololu_node", log_level=rospy.INFO) 
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("[POLOLU]: Connecting to pololu daisy chain")
        self.last_set_speed_time = rospy.get_rostime()
        self.timeout = rospy.get_param("~timeout")  # time between hearing commands before we shut off the motors
        self.left_wheel_state = rospy.get_param("~wheel_state")
        self.max_accel = int(rospy.get_param("~max_accel"))

        # Commands and actuator state variables
        self.last_left_wheel_msg = FreshVal(stale_val=0, timeout=self.timeout, name="left_wheel")
        self.last_right_wheel_msg = FreshVal(stale_val=0, timeout=self.timeout, name="right_wheel")
        self.left_wheel_state = FreshVal(stale_val=0, timeout=self.timeout, name="left_wheel_state")
        self.right_wheel_state =  FreshVal(stale_val=0, timeout=self.timeout, name="right_wheel_state")

        rospy.loginfo("[POLOLU]: timeout = %s", self.timeout)

        # get device numbers (see daisy_node.launch) and open a serial connection
        # (la = linear_actuator)
        # (ela = extend_linear_actuator)
        # (ila = insert_linear_actuator)
        self.left_wheel = Pololu(rospy.get_param("~dev_num/left_wheel"), flip=False) #was ela
        print(self.left_wheel)
        # NOTE: THESE MUST HAVE THE SAME FLIP
        self.right_wheel =  Pololu(rospy.get_param("~dev_num/right_wheel"), flip=False) # was ila_left

        self.devices = [self.left_wheel, self.right_wheel]
        self._check_devices()
        self._set_accel_limits()

        rospy.sleep(0.1) # wait for params to get set

        # Subscribers
        rospy.Subscriber('keyboard_velocity', wheelvel, self.wheel_callback)
        self.wheel_sub = rospy.Subscriber("keyboard_velocity", wheelvel, self.wheel_callback, queue_size=1)
        self.left_wheel_state_pub = rospy.Publisher("/left_wheel/state", Float32, queue_size=1)
        self.right_wheel_state_pub = rospy.Publisher("/right_wheel/state", Float32, queue_size=1)
        self._have_shown_message = False  # flag to indicate we have showed the motor shutdown message

    def _set_accel_limits(self):
        for dev in self.devices:
            dev.set_motor_limits(self.max_accel, self.max_accel)

        rospy.loginfo("[POLOLU]: Set acceleration limit to ({}/3200)".format(self.max_accel))

    def _check_devices(self): 
        """just check that you can read from a device and that you get the
        expected value. this will crash cause the node to crash if something
        is up"""
        for dev in self.devices:
            baud = dev.get_baud()
            if baud != 3750: # 3750 means 19200 (in manual, you divide 72e6/baud)
                raise Exception("Read BAUD=={} from device {}".format(baud, dev.dev_num))
        rospy.loginfo("[POLOLU]: Devices connected")

    def _stop_all_motors(self):
        """Send stop command to all daisy chained motors"""
        for dev in self.devices:
            dev.stop()

    def _send_left_wheel_cmd(self):
        last_cmd = self.last_left_wheel_msg.val
        # TODO (p1): add a check here to monitor that the la is not jamming.
        self.left_wheel.drive(last_cmd)

    def _send_right_wheel_cmd(self):
        last_cmd = self.last_right_wheel_msg.val # make sure they get the same command
        self.right_wheel.drive(last_cmd)

    def _read_wheel_states(self):
        # read from serial and update variables
        self.left_wheel_state.update(self.left_wheel.get_an1())
        self.right_wheel_state.update(self.right_wheel.get_an1())
        # debug msg
        rospy.logdebug("[POLOLU]: left wheel state = {}".format(self.left_wheel_state.val))
        rospy.logdebug("[POLOLU]: right wheel state = {}".format(self.right_wheel_state.val))

    def _publish_la_states(self):
        # TODO (p2): convert these to meters once they are attached to the robot 
        # publish left wheel state
        if self.left_wheel_state.is_fresh:
            self.left_wheel_state_pub.publish(self.left_wheel_state.val)
        # publish right wheel state
        if self.right_wheel_state.is_fresh:
            self.right_wheel_state_pub.publish(self.right_wheel_state.val)

            # TODO (p1) : uncomment and test this when stuff is wired
            ## if we are extended (this is received by roboclaw_node to make sure we never dig when we are not extended)
            #if val > self.extended_state:
            #    self.digger_extended_pub.publish(True)
            #else:
            #    self.digger_extended_pub.publish(False)

    def run(self):
        """Run the main ros loop. All Serial communication should happen in this thread"""
        rospy.loginfo("[POLOLU]: Starting node loop")
        rate = rospy.Rate(30) 

        while not rospy.is_shutdown():
            # Write
            self._send_left_wheel_cmd()
            self._send_right_wheel_cmd()
            # Read
            #self._read_la_states()
            # ROS publish
            #self._publish_la_states()

            rate.sleep()

    def _scale_and_clip(self, cmd):
        """[-1,1] --> [-3200, 3200] for (ROS --> Pololu SMC)"""
        return clip(int(cmd * 3200), -3200, 3200)

    # ROS command callbacks
    def wheel_callback(self, msg):
        """Update left wheel command value"""
        rospy.logdebug("[POLOLU]: Velocity cmd to left wheel: %.4f", msg.one)
        cmd = self._scale_and_clip(msg.one)
        self.last_left_wheel_msg.update(cmd)
        rospy.logdebug("[POLOLU]: Left wheel serial cmd = %d", cmd)
        """Update right wheel command value"""
        rospy.logdebug("[POLOLU]: Velocity cmd to right wheel: %.4f", msg.two)
        cmd = self._scale_and_clip(msg.two)
        self.last_right_wheel_msg.update(cmd)
        rospy.logdebug("[POLOLU]: Right wheel serial cmd = %d", cmd)


    def shutdown(self):
        """Handle shutting down the node"""
        rospy.loginfo("Shutting down daisy node")
        # so these don't get called while the node is shutting down
        # (need to add checks in case it immediately shuts down. (i think))
        if hasattr(self, "wheel_sub"):
            self.wheel_sub.unregister()
        self._stop_all_motors()

if __name__ == "__main__":
    try:
        node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("[POLOLU]: Exiting node")
