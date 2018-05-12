#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree

import math
import numpy as np

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

class WaypointUpdater(object):
    """ update the target velocity of each waypoint based on traffic light and obstacle detection data

        @subscribed /current_pose:      the vehicle's current position
        @subscribed /base_waypoints:    the complete list of waypoints the car will be following
        @subscribed /obstacle_waypoint: the locations to stop for obstacles    
        @subscribed /traffic_waypoint:  the locations to stop for red traffic lights

        @published  /final_waypoints:   the list of waypoints ahead of the car with final target velocities   
    """
    WAYPOINT_UPDATE_FREQ = 30 # Waypoint update frequency
    LOOKAHEAD_WPS = 200       # Number of waypoints we will publish. You can change this number

    def __init__(self):
        rospy.init_node('waypoint_updater')

        # ego vehicle pose:
        self.pose = None 
        self.base_waypoints = None
        self._base_waypoints_size = None
        self._base_waypoints_location = None
        self._base_waypoints_index = None

        # subscribe:
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)
        rospy.Subscriber('/traffic_waypoint', Lane, self.traffic_cb)

        # publish:
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.spin()

    def pose_cb(self, pose):
        """ update ego vehicle pose

        sample ego vehicle pose message:
            header: 
                seq: 13066
                stamp: 
                    secs: 1526130554
                    nsecs: 841849088
                frame_id: "/world"
            pose: 
                position: 
                    x: 1387.015
                    y: 1200.397
                    z: 0.01770636
                orientation: 
                    x: 0.0
                    y: 0.0
                    z: 0.012926360012
                    w: 0.999916451118

        """
        self.pose = pose

    def waypoints_cb(self, waypoints):
        """ load base waypoints from system 

        sample base waypoints message:
        header: 
            seq: 5
            stamp: 
                secs: 0
                nsecs:         0
            frame_id: "/world"
        waypoints: 
        - 
            pose: 
                header: 
                    seq: 0
                    stamp: 
                        secs: 0
                        nsecs: 0
                    frame_id: ''
                pose: 
                    position: 
                        x: 896.233
                        y: 1128.82
                        z: 0.0
                    orientation: 
                        x: -0.0
                        y: 0.0
                        z: 0.00117809681728
                        w: -0.999999306044
            twist: 
                header: 
                    seq: 0
                    stamp: 
                        secs: 0
                        nsecs: 0
                    frame_id: ''
                twist: 
                    linear: 
                        x: 0.0
                        y: 0.0
                        z: 0.0
                    angular: 
                        x: 0.0
                        y: 0.0
                        z: 0.0
        """
        if not self.base_waypoints:
            # load waypoints:
            self.base_waypoints = waypoints
            # build index upon waypoints:
            self._base_waypoints_location = np.array(
                [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            )
            self._base_waypoints_size, _ = self._base_waypoints_location.shape
            self._base_waypoints_index = KDTree(
                self._base_waypoints_location
            )

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
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

    def get_next_waypoint_index(self):
        """ get next waypoint index 
        """
        # ego vehicle location:
        ego_vehicle_location = np.array(
            [self.pose.pose.position.x, self.pose.pose.position.y]
        )

        # closest waypoint
        _, closest_waypoint_index = self._base_waypoints_index.query(ego_vehicle_location)

        closest_waypoint_location = self._base_waypoints_location[closest_waypoint_index]
        previous_waypoint_location = self._base_waypoints_location[closest_waypoint_index - 1]

        # whether the closest waypoint is the next waypoint:
        is_next_waypoint = (
            np.dot(
                closest_waypoint_location - ego_vehicle_location,
                previous_waypoint_location - ego_vehicle_location
            ) < 0.0
        )

        # next waypoint index:
        next_waypoint_index = closest_waypoint_index
        if not is_next_waypoint:
            next_waypoint_index = (closest_waypoint_index + 1) % self._base_waypoints_size

        return next_waypoint_index

    def publish_next_waypoints(self, next_waypoint_index):
        """ publish next waypoints
        """
        # init:
        next_lane = Lane()

        # set header:
        next_lane.header = self.base_waypoints.header
        # set waypoints:
        next_lane.waypoints = self.base_waypoints.waypoints[
            next_waypoint_index: next_waypoint_index + WaypointUpdater.LOOKAHEAD_WPS
        ]

        # publish:
        self.final_waypoints_pub.publish(next_lane)

    def spin(self):
        """ do spin
        """
        # update frequency:
        rate=rospy.Rate(WaypointUpdater.WAYPOINT_UPDATE_FREQ)
        # do spin:
        while not rospy.is_shutdown():
            # if both base waypoints and ego vehicle pose present
            if self.base_waypoints and self.pose:
                next_waypoint_index = self.get_next_waypoint_index()
                self.publish_next_waypoints(next_waypoint_index)
            rate.sleep()

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
