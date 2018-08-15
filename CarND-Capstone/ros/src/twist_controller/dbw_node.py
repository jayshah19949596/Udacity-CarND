#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
WheelBase) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        VehicleMass = rospy.get_param('~vehicle_mass', 1736.35)
        DecelerationLimit = rospy.get_param('~decel_limit', -5)
        WheelRadius = rospy.get_param('~wheel_radius', 0.2413)
        WheelBase = rospy.get_param('~wheel_base', 2.8498)
        SteerRatio = rospy.get_param('~steer_ratio', 14.8)
        MaxLateralAcceleration = rospy.get_param('~max_lat_accel', 3.)
        MaxSteeringAngle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # TODO: Create `Controller` object
        self.Controller = Controller(VehicleMass=VehicleMass,
                                     DecelerationLimit=DecelerationLimit,
                                     WheelRadius=WheelRadius,
                                     WheelBase=WheelBase,
                                     SteerRatio=SteerRatio,
                                     MaxLateralAcceleration=MaxLateralAcceleration,
                                     MaxSteeringAngle=MaxSteeringAngle)

        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)

        self.CurrentVelocity = None
        self.CurrentAngularVelocity = None
        self.dbw_enabled = None
        self.DesiredVelocity = None
        self.DesiredAngularVelocity = None
        self.Throttle = 0
        self.SteeringAngle = 0
        self.BreakPress = 0

        self.loop()

    def loop(self):
        Freq = rospy.Rate(50)
        while not rospy.is_shutdown():
            # TODO: Get predicted Throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            if not None in (self.CurrentVelocity, self.DesiredVelocity, self.DesiredAngularVelocity):

                self.Throttle, self.BreakPress, self.SteeringAngle = self.Controller.Control( self.DesiredVelocity,
                                                                                              self.DesiredAngularVelocity,
                                                                                              self.CurrentVelocity,
                                                                                              self.dbw_enabled    )
            if self.dbw_enabled:
                self.publish(self.Throttle, self.BreakPress, self.SteeringAngle)
            Freq.sleep()


    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg

    def twist_cb(self, msg):
        self.DesiredVelocity = msg.twist.linear.x
        self.DesiredAngularVelocity = msg.twist.angular.z

    def velocity_cb(self, msg):
        self.CurrentVelocity = msg.twist.linear.x


    def publish(self, Throttle, BreakPress, SteeringAngle):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = Throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = SteeringAngle
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = BreakPress
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    DBWNode()
