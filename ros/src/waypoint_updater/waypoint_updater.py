#!/usr/bin/env python

import copy
import math

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from styx_msgs.msg import Lane

'''
This node will publish waypoints from the car's current position
to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which
does not care about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status
of traffic lights too.

Please note that our simulator also provides the exact location of
traffic lights and their current status in `/vehicle/traffic_lights` message.
You can use this message to build this node as well as to
verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200  # Number of waypoints we publish


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)

        self.pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.waypoints = None
        self.ego = None

        rospy.spin()

    def publish(self):
        idx = self.find_next_waypoint()
        if idx > -1 and not rospy.is_shutdown():
            rospy.loginfo("Current position ({}, {}), next waypoint: {}"
                          .format(self.ego.pose.position.x,
                                  self.ego.pose.position.y,
                                  idx))
            waypoints = self.waypoints + self.waypoints
            waypoints = waypoints[idx:idx+LOOKAHEAD_WPS]
            lane = Lane()
            lane.header.frame_id = '/world'
            lane.header.stamp = rospy.Time.now()
            lane.waypoints = waypoints
            self.pub.publish(lane)

    def pose_cb(self, msg):
        if self.ego is None or self.ego.header.seq < msg.header.seq:
            # TODO: Calculate ego's velocity (and maybe acceleration) if needed
            self.ego = msg
            self.publish()

    def waypoints_cb(self, lane):
        if self.waypoints is None:
            self.waypoints = lane.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message.
        # We will implement it later
        pass

    @classmethod
    def yaw_from_quaternion(cls, q):
        quaternion = [q.x, q.y, q.z, q.w]
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        return yaw

    def find_next_waypoint(self):
        min_dist = 1e10
        min_idx = -1
        if self.ego and self.waypoints:
            ego_pose = self.ego.pose
            n_waypoints = len(self.waypoints)
            # Find the closest waypoint
            for i in range(n_waypoints):
                wp_pos = self.waypoints[i].pose.pose.position
                dl = self.euclidean(ego_pose.position, wp_pos)
                if dl < min_dist:
                    min_dist = dl
                    min_idx = i

            # Check if we are behind or past the closest waypoint
            wp_pos = self.waypoints[min_idx].pose.pose.position
            pos = copy.deepcopy(ego_pose.position)
            yaw = self.yaw_from_quaternion(ego_pose.orientation)
            pos.x += math.cos(yaw) * .1
            pos.y += math.sin(yaw) * .1
            if self.euclidean(wp_pos, pos) > min_dist:
                min_idx = (min_idx + 1) % n_waypoints
        return min_idx

    @staticmethod
    def get_waypoint_velocity(waypoint):
        return waypoint.twist.twist.linear.x

    @staticmethod
    def set_waypoint_velocity(waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    @classmethod
    def euclidean(cls, pos1, pos2):
        """
        Return the Euclidean distance between two points

        :param pos1: geometry_msgs/Point
        :param pos2: geometry_msgs/Point
        :return: Euclidean distance between two points
        """
        return math.sqrt((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2 +
                         (pos1.z - pos2.z) ** 2)

    @classmethod
    def distance(cls, waypoints, wp1, wp2):
        dl = 0
        for i in range(wp1, wp2+1):
            dl += cls.euclidean(waypoints[wp1].pose.pose.position,
                                waypoints[i].pose.pose.position)
            wp1 = i
        return dl


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
