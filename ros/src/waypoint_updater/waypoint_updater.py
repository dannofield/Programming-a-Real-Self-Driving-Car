#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy.spatial import KDTree

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 0.5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.stopline_wp_idx = -1
        
        #rospy.spin()
        #we prefer this loop to have control of the rate
        self.loop()

    def loop(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            #If we already have data to calculate the closest waypoint
            if self.pose and self.base_waypoints:
                #get the closest waypoint
                closest_waypoint_idx = self.get_closest_waypoint_idx()
                #call the function to publish these waypoints
                self.publish_waypoints(closest_waypoint_idx)
            rate.sleep()
            
    def get_closest_waypoint_idx(self):
        #get data we got on pose_callback
        x = self.pose.pose.position.x #float 64
        y = self.pose.pose.position.y #float 64

        #finde the closest waypoint's idx
        closest_idx = self.waypoint_tree.query([x,y], 1)[1]
        
        # check if the closest is ahead or behind vehicle
        closest_waypoint = self.waypoints_2d[closest_idx]
        pre_closest_waypoint = self.waypoints_2d[closest_idx-1]
        
        #equation for hyperplane through closest_waypoint
        closest_vect = np.array(closest_waypoint)
        pre_vect = np.array(pre_closest_waypoint)
        pos_vect = np.array([x,y])

        #if dot product is positive, this closest coordinate is behind the car
        val = np.dot(closest_vect - pre_vect, pos_vect - closest_vect)
        if val > 0:
            #so take the closest_idx + 1 (without overrunning the vector)
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx
    
    def publish_waypoints(self, closest_idx):
        lane = Lane()
        lane.header = self.base_waypoints.header
        
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        lane.waypoints = self.base_waypoints.waypoints[closest_idx : farthest_idx]
        
        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
            self.final_waypoints_pub.publish(lane)
        else:
            lane.waypoints = self.decelerate_waypoints(lane.waypoints, closest_idx)
            self.final_waypoints_pub.publish(lane)
     
    def decelerate_waypoints(self, waypoints, closest_idx):
        laneWaypoints = []
        for i , waypoint in enumerate(waypoints):
            newWaypoint = Waypoint()
            newWaypoint.pose = waypoint.pose

            stop_idx = max(self.stopline_wp_idx - closest_idx - 3, 0)
            dist = self.distance(waypoints, i, stop_idx)
            vel = math.sqrt(2*MAX_DECEL*dist)
            if vel < 1.0:
                vel = 0.

            newWaypoint.twist.twist.linear.x = min(vel, waypoint.twist.twist.linear.x)
            laneWaypoints.append(newWaypoint)

        return laneWaypoints

    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg
        

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints
        #base waypoints dont change so we only need them once
        
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            # for quick nearest-neighbor lookup
            # This class provides an index into a set of k-D points which can be used to 
            #rapidly look up the nearest neighbors of any point.
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        # self.stopline_wp_idx = msg.data
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
