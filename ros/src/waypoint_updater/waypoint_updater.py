#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg     import Lane, Waypoint

import math
import numpy as np
from scipy.spatial import KDTree
from std_msgs.msg import Int32


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



"""
    The eventual purpose of this node is to publish a fixed number of waypoints ahead of the vehicle 
    with the correct target velocities, depending on traffic lights and obstacles.
"""
LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_DECEL     = 0.5


class WaypointUpdater(object):
    def __init__(self):
        print("Init WaypointUpdater")
        rospy.init_node('waypoint_updater')
        
        # rospy.Subscriber('Topic',            Msg_Type,    Calback)
        rospy.Subscriber('/base_waypoints',    Lane,        self.waypoints_cb)
        rospy.Subscriber('/current_pose',      PoseStamped, self.pose_cb)
        rospy.Subscriber('/traffic_waypoint',  Int32,       self.traffic_cb)
        #rospy.Subscriber('/obstacle_waypoint', Int32,       self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_lane      = None
        self.pose           = None
        self.base_waypoints = None
        self.waypoints_2d   = None
        self.waypoint_tree  = None

        rospy.spin()

    # Give us control over Publisher frequency.
    # We want a target 50Hz. We can go for less than that. You can say 30Hz.
    def spin(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints:
                # Get closest waypoint
                closest_waypoint_idx = self.get_closest_waypoint_idx()
                self.publish_waypoints(closest_waypoint_idx)
            rate.sleep()

    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x # you can go the message definition.
        y = self.pose.pose.position.y # you can go the message definition.
        closest_idx = self.waypoint_tree.query([x,y], 1)[1] # First "1": return closest one item,  Second "1": return the index

        # Check if closest is ahead or behinf vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord    = self.waypoints_2d[closest_idx-1]

        # Equation for hyperplane through closest_coords
        cl_vect   = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect  = np.array([x,y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        if val > 0: # Check that the waypoints in the front of the car
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx

    def publish_waypoints(self, closest_idx):
        lane = Lane()
        lane.header = self.base_waypoints.header
        lane.waypoints = self.base_waypoints.waypoints[closest_idx:closest_idx + LOOKAHEAD_WPS]
        self.final_waypoints_pub.publish(lane)

        #final_lane = self.generate_lane()
        #self.final_waypoints_pub.publish_waypoints(final_lane)

    def generate_lane(self):
        lane = Lane()

        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_lane.waypoints[closest_idx:farthest_idx]

        if self.stopline_wp_idx == 0.1 or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)
        
        return lane

    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose
            stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0) # Two waypoints back from line so front car stops at line
            dist = self.distance(waypoints, i, stop_idx)
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1.0:
                vel = 0

            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)
        
        return temp


    def pose_cb(self, msg):
        # TODO: Implement
        # Store the current car pose. It is called arounf 50Hz. 
        self.pose = msg
        pass
    
    # Callback for the topic "base_waypoints"
    # It is latched subscriber. Once callback is called, it does not send the waypoints anymore.
    # The "base_waypoints" are never changing and that will be inefficient to call that every second.
    # We want to take a chank of these waypoints and use the first 200 that are at the front of the car as reference.
    # The KDTree is used from the "from scipy.spatial" library. 
    # KDTree is a data structure that let you look up to the closest point in the space efficently log(n) instead of n.
    # KDTree is used to figure out which waypoint is closest to the car.
    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints # Store in the object
        # Make sure that "waypoints_2d" is intialized before the subscriber. 
        # Otherwise we have a race condition.
        if not self.waypoints_2d: 
            # Converted the waypoints to 2D coordinates for each points
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            # KDTree is used to figure out which waypoint is closest to the car.
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg
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
