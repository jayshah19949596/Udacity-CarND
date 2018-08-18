#!/usr/bin/env python

import rospy

from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree

import numpy
import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.
As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.
Once you have created dbw_node, you will update this node to use the status of traffic lights too.
Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
'''

LOOKAHEAD_WPS = 50  # Number of waypoints we will publish. You can change this numbe
MaxDeceleration = 0.5


class WaypointUpdater(object):
    def __init__(self):

        # Initialise This Node
        rospy.init_node('waypoint_updater')

        # Subscribers ==> Callback Function
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)

        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_velocity',TwistStamped, self.currentvel_cb)

        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.FinalWaypointsPublisher = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.Pose = None
        self.StopWaypointID = -1
        self.BaseWaypoints = None
        self.Waypoint = None
        self.WaypointTree = None
        self.current_vel = None

        # Suspend 
        self.loop()
    #--------------------------------------
    def loop(self):

        ExeRate = rospy.Rate(50) 
        # Check if the Roscore is Active
        while not rospy.is_shutdown():
            if not self.Pose is None and not self.BaseWaypoints is None and not self.WaypointTree is None:
                # publish closest waypoint
                self.publish_waypoints()
            # got to sleep for 20ms
            ExeRate.sleep()
    #--------------------------------------
    def get_next_waypoint_idx(self):
        pX = self.Pose.pose.position.x
        pY = self.Pose.pose.position.y
        ClosestwaypointID = self.WaypointTree.query([pX, pY], 1)[1]
        ClosestWaypoint = self.Waypoint[ClosestwaypointID]
        PrevWaypoint = self.Waypoint[ClosestwaypointID - 1]
        # Position Vector
        SelfPos = numpy.array([pX, pY])
        ClosestWaypoint = numpy.array(ClosestWaypoint)
        PrevWaypoint = numpy.array(PrevWaypoint)
        # Check if the Waypoint is behind the vehicle
        if numpy.dot(ClosestWaypoint - PrevWaypoint, SelfPos - ClosestWaypoint) > 0:
            # Use Next Waypoint
            ClosestwaypointID = (ClosestwaypointID + 1) % len(self.Waypoint)

        return ClosestwaypointID
    #--------------------------------------
    def publish_waypoints(self):
        final_lane = self.generate_lane()
        self.FinalWaypointsPublisher.publish(final_lane)
    #--------------------------------------
    def generate_lane(self):
        lane = Lane()
        
        ClosestwaypointID = self.get_next_waypoint_idx()
        FarthestWaypointID = ClosestwaypointID + LOOKAHEAD_WPS
        BaseWaypoints = self.BaseWaypoints.waypoints[ClosestwaypointID:FarthestWaypointID]
        
        if self.StopWaypointID == -1 or (self.StopWaypointID >= FarthestWaypointID):
            lane.waypoints = BaseWaypoints
        else:
            lane.waypoints = self.decelerate_waypoints(BaseWaypoints, ClosestwaypointID)
        
        return lane
    #--------------------------------------
    def decelerate_waypoints(self, waypoints, ClosestwaypointID):
        temp = []
        for i, wp in enumerate(waypoints):
        
            p = Waypoint()
            p.pose = wp.pose

            StopDist = 2
            # Stop behind line
            StopWaypointID = max(self.StopWaypointID - ClosestwaypointID - StopDist, 0)

            if self.current_vel < 0.5:
                StopWaypointID = 0

            DistanceToStop = self.distance(waypoints, i, StopWaypointID)

            # DistanceToStop = max(self.distance(waypoints, i, StopWaypointID) - StopDist, 0)

            vel = math.sqrt(2 * MaxDeceleration * DistanceToStop)

            if vel < 1.:
                vel == 0.

            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)
            
        return temp
    #--------------------------------------
    def pose_cb(self, msg):
        self.Pose = msg
    #--------------------------------------
    def currentvel_cb(self, msg):
        self.current_vel = msg.twist.linear.x
    
    def publish_nxt_waypoints(self, NextWaypointID):
        WaypointsOfLane = Lane()
        WaypointsOfLane.header = self.BaseWaypoints.header
        # All the waypoints from Closest Waipoint to the Car upto 200 waypoints
        WaypointsOfLane.waypoints = self.BaseWaypoints.waypoints[NextWaypointID:NextWaypointID + LOOKAHEAD_WPS]
        # store into final variable
        self.FinalWaypointsPublisher.publish(WaypointsOfLane)
    #--------------------------------------
    def waypoints_cb(self, waypoints):
        # copy of lane waypoints
        self.BaseWaypoints = waypoints
        if not self.Waypoint:
            self.Waypoint = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in
                                 waypoints.waypoints]
            self.WaypointTree = KDTree(self.Waypoint)
    #--------------------------------------
    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.StopWaypointID = msg.data
    #--------------------------------------
    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass
    #--------------------------------------
    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x
    #--------------------------------------
    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity
    #--------------------------------------
    def distance(self, waypoints, Start, End):
        dist = 0
        distFunc = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(Start, End + 1):
            dist += distFunc(waypoints[Start].pose.pose.position, waypoints[i].pose.pose.position)
            Start = i
        return dist
    #--------------------------------------

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
