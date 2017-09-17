#!/usr/bin/env python

import copy
import math

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from styx_msgs.msg import Lane
from nav_msgs.msg import Path

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

LOOKAHEAD_WPS = 50  # Number of waypoints we publish


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped,
                         self.pose_cb, queue_size=1)
        self.base_waypoints_sub = \
            rospy.Subscriber('/base_waypoints', Lane,
                             self.waypoints_cb, queue_size=1)
        rospy.Subscriber('/traffic_waypoint', Int32,
                         self.traffic_cb, queue_size=1)
        rospy.Subscriber('/obstacle_waypoint', Lane,
                         self.obstacle_cb, queue_size=1)

        self.pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)
        self.pub_path = rospy.Publisher('/local_path', Path, queue_size=1)
        self.pub_next = rospy.Publisher('/next_waypoint',
                                        PoseStamped, queue_size=1)

        # TODO: Add other member variables you need below
        self.waypoints = None
        self.ego = None
        self.next_idx = -1

        rospy.spin()

    def publish(self):
        self.next_idx = self.find_next_waypoint()
        if self.next_idx > -1 and not rospy.is_shutdown():
            rospy.loginfo("Current position ({}, {}), next waypoint: {}"
                          .format(self.ego.pose.position.x,
                                  self.ego.pose.position.y,
                                  self.next_idx))
            waypoints = self.waypoints + self.waypoints
            waypoints = waypoints[self.next_idx:self.next_idx+LOOKAHEAD_WPS]

            next_pose = copy.deepcopy(waypoints[0].pose)
            next_pose.header.frame_id = '/world'
            next_pose.header.stamp = rospy.Time.now()
            self.pub_next.publish(next_pose)

            lane = Lane()
            lane.header.frame_id = '/world'
            lane.header.stamp = rospy.Time.now()
            lane.waypoints = waypoints
            self.pub.publish(lane)

            # This is needed for visualising in rviz
            path = Path()
            path.header.frame_id = '/world'
            path.header.stamp = rospy.Time.now()
            path.poses = map(lambda wp: wp.pose, waypoints)
            self.pub_path.publish(path)

    def pose_cb(self, msg):
        if self.ego is None or self.ego.header.seq < msg.header.seq:
            # TODO: Calculate ego's velocity (and maybe acceleration) if needed
            self.ego = msg
            self.publish()

    def waypoints_cb(self, lane):
        if self.waypoints is None:
            self.waypoints = lane.waypoints

            # Unsubscribe so that waypoint loader stops publishing
            self.base_waypoints_sub.unregister()

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message.
        # We will implement it later
        pass

    @classmethod
    def vector_from_quaternion(cls, q):
        quaternion = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
        x = math.cos(yaw) * math.cos(pitch)
        y = math.sin(yaw) * math.cos(pitch)
        z = math.sin(pitch)
        return x, y, z

    def find_next_waypoint(self):
        min_dist = 1e10
        min_idx = -1
        if self.ego and self.waypoints:
            ego_pose = self.ego.pose
            n_waypoints = len(self.waypoints)
            # Find the closest waypoint
            for i in range(self.next_idx, self.next_idx + n_waypoints):
                idx = (i + n_waypoints) % n_waypoints
                wp_pos = self.waypoints[idx].pose.pose.position
                dl = self.euclidean(ego_pose.position, wp_pos)
                if dl < min_dist:
                    min_dist = dl
                    min_idx = idx
                if min_dist < 10 and dl > min_dist:
                    break

            # Check if we are behind or past the closest waypoint
            wp_pos = self.waypoints[min_idx].pose.pose.position
            pos = copy.deepcopy(ego_pose.position)
            x, y, z = self.vector_from_quaternion(ego_pose.orientation)
            pos.x += x * .1
            pos.y += y * .1
            pos.z += z * .1
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
