#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray
from std_msgs.msg import Bool
from std_msgs.msg import Int32

import math
import tf
import waypoint_util

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


        # DONE: Add other member variables you need below
        self.base_waypoints = []
        self.pose = None
        self.dbw_enabled = False
        self.required_velocity = (rospy.get_param('/waypoint_loader/velocity')
                * 1000.) / (60. * 60.)
        self.decel_limit = rospy.get_param('/dbw_node/decel_limit', -5)
        self.target_decel = 1.
        self.current_velocity = 0.
        self.zero_padding_wps = 50
        self.last_next_wp_idx = -1

        self.stop_line_wp_idx = -1
        self.last_stop_line_wp_idx = -1
        self.max_updated_wp_idx = -1
        self.stop_line_buffer = 5
        self.has_traffic_waypoint = False
        self.waypoint_velocity_updated = False

        rospy.loginfo('required_vel=%s, decel_limit=%s'
            , self.required_velocity, self.decel_limit)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)

        # DONE: Add a subscriber for /traffic_waypoint 
        # LATER: and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_waypoint_cb)


        self.final_waypoints_pub = rospy.Publisher('/final_waypoints'
                                        , Lane, queue_size=1)
        rospy.spin()


    def current_velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear.x

    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg.data
        rospy.logdebug('dbw_enabled: %s', self.dbw_enabled)

    def pose_cb(self, msg):
        self.pose = msg.pose
        #rospy.loginfo('current: x=%s, vel=%s'
        #    , self.pose.position.x, self.current_velocity)

        if not self.dbw_enabled:
            # to make full search for closest next waypoint,
            # when dbw control comes back
            self.last_next_wp_idx = -1
            return

        # find next_waypoint considering car's yaw
        next_wp_idx = self.find_next_waypoint()
        if next_wp_idx == -1:
            return

        # if it's the same as before, it doesn't need to publish
        if next_wp_idx == self.last_next_wp_idx:
            if self.last_stop_line_wp_idx == self.stop_line_wp_idx:
                # test lot has no traffic_waypoint
                if self.has_traffic_waypoint:
                    return

        # if the line to stop has been changed,
        # we need to update velocity of the waypoints
        if self.last_stop_line_wp_idx != self.stop_line_wp_idx:
            rospy.loginfo('update: stop_line_updated: %s'
                    , self.stop_line_wp_idx)

            # when the stop_line is before the car
            if self.stop_line_wp_idx > next_wp_idx:
                dist_for_stop = self.distance(self.base_waypoints
                    , next_wp_idx, self.stop_line_wp_idx)

                dist = max(0.00001, dist_for_stop)
                decel = self.current_velocity**2/(2*dist)

                # if it is possible to stop
                if decel < abs(self.decel_limit):
                    max_decel = max(abs(self.decel_limit), decel)
                    self.update_waypoints_for_stop(next_wp_idx, max_decel)
                    rospy.loginfo("update: stop in time by max_decel %s"
                        , max_decel)
                # just pass
                else:
                    self.update_waypoints_for_run(next_wp_idx)
                    rospy.loginfo("update: cannot stop in time by decel %s"
                        , decel)
            # just pass
            else:
                self.update_waypoints_for_run(next_wp_idx)
                rospy.loginfo('update: car(%s) went past stop_line(%s)'
                    , next_wp_idx, self.stop_line_wp_idx)

        self.publish_final_waypoints(next_wp_idx)

    def waypoints_cb(self, msg):
        # Lane {header, waypoints[]}
        # Waypoint {
        #   twist {header, twist {linear {x, y, z}, angular {x, y, z}}}
        #  , pose {header, pose {position {x, y, z}, orientation {x, y, z, w}}}}
        # }
        rospy.logdebug('waypoints_cb: len=%s', len(msg.waypoints))
        self.base_waypoints = msg.waypoints


    def traffic_waypoint_cb(self, msg):
        # DONE: Callback for /traffic_waypoint message. Implement
        self.has_traffic_waypoint = True
        self.stop_line_wp_idx = msg.data


    def update_waypoints_for_run(self, next_wp_idx):
        if next_wp_idx < self.max_updated_wp_idx:
            for i in range(next_wp_idx, self.max_updated_wp_idx):
                self.set_waypoint_velocity(self.base_waypoints
                        , i, self.required_velocity)
        return

    def update_waypoints_for_stop(self, next_wp_idx, decel):
        # to make stop a little before the stop line
        if self.stop_line_wp_idx >= self.stop_line_buffer:
            stop_line_wp_idx = self.stop_line_wp_idx - self.stop_line_buffer

        for i in range(next_wp_idx, stop_line_wp_idx):
            dist = self.distance(self.base_waypoints, i, stop_line_wp_idx)
            dist = max(0., dist)
            stopping_vel = math.sqrt(2*decel*dist)
            stopping_vel = min(stopping_vel, self.required_velocity)
            self.set_waypoint_velocity(self.base_waypoints, i, stopping_vel)

        # update by zero speed padding
        for i in range(stop_line_wp_idx
                    , stop_line_wp_idx + self.zero_padding_wps):
            self.set_waypoint_velocity(self.base_waypoints, i, 0.)

        self.max_updated_wp_idx = stop_line_wp_idx + self.zero_padding_wps

    def publish_final_waypoints(self, next_wp_idx):
        end_wp_idx = min(next_wp_idx + LOOKAHEAD_WPS
            , len(self.base_waypoints)) - 1

        lane = Lane()
        lane.header.frame_id = '/updater'
        lane.header.stamp = rospy.get_rostime()
        waypoints = []
        for i in range(next_wp_idx, end_wp_idx + 1):
            waypoints.append(self.base_waypoints[i])
            '''
            if self.waypoint_velocity_updated:
                rospy.loginfo('updated vel: x=%s, vel=%s'
                    , self.base_waypoints[i].pose.pose.position.x
                    , self.base_waypoints[i].twist.twist.linear.x)
            '''

        lane.waypoints = waypoints
        self.final_waypoints_pub.publish(lane)
        self.last_next_wp_idx = next_wp_idx
        self.last_stop_line_wp_idx = self.stop_line_wp_idx
        self.waypoint_velocity_updated = False

    def find_next_waypoint(self):
        if not self.pose or not self.base_waypoints:
            return -1

        x = self.pose.position.x
        y = self.pose.position.y
        quarternion = (self.pose.orientation.x
                    , self.pose.orientation.y
                    , self.pose.orientation.z
                    , self.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quarternion)

        next_wp_idx = waypoint_util.next_waypoint(
                x, y, euler[2], self.base_waypoints, self.last_next_wp_idx)

        return next_wp_idx

    def obstacle_cb(self, msg):
        # LATER: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        self.waypoint_velocity_updated = True
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
