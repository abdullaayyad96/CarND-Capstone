#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
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


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.base_waypoints_2d = None
        self.base_waypoints_tree = None
        self.final_waypoints = None

        self.pub_loop()
        
    def pub_loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints_tree:
                next_waypoint = self.get_next_waypoint()
                self.publish_waypoints(next_waypoint)
                rate.sleep()
    
    def publish_waypoints(self, start_idx):
        #create Lane msg instance
        lane_msg = Lane()
        
        lane_msg.header = self.base_waypoints.header
        lane_msg.waypoints = self.base_waypoints.waypoints[start_idx:(start_idx + LOOKAHEAD_WPS)]
        
        #publish to relevant topic
        self.final_waypoints_pub.publish(lane_msg)
    
    def get_next_waypoint(self):
        #returns the index of the next waypoint to the current pose
        pose_x = self.pose.pose.position.x
        pose_y = self.pose.pose.position.y
        
        #get index of closest waypoint
        next_waypoint_idx = self.base_waypoints_tree.query([pose_x, pose_y], 1) [1]
        rospy.loginfo('waypoint: %s', next_waypoint_idx)
        #ensure that closest waypoint is the next waypoint, otherwise inceremnt waypoint
        next_waypoint = np.array(self.base_waypoints_2d[next_waypoint_idx])
        prev_waypoint = np.array(self.base_waypoints_2d[next_waypoint_idx - 1])
        current_pose = np.array([pose_x, pose_y])
        
        pose_projection = np.dot( next_waypoint - prev_waypoint, current_pose - next_waypoint)         
        if pose_projection > 0:
            next_waypoint_idx += 1
            next_waypoint_idx = next_waypoint_idx % len(self.base_waypoints_2d)
            
        return next_waypoint_idx
    
    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        if self.base_waypoints is None:
            self.base_waypoints = waypoints
            self.base_waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.base_waypoints_tree = KDTree(self.base_waypoints_2d)            

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
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
