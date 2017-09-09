#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math

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

        self.pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.world = None
        self.ego = None

        self.publish()
        rospy.spin()

    def publish(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            idx = self.find_next_waypoint()
            if idx > -1:
                rospy.loginfo("Next waypoint: {}".format(idx))
                waypoints = self.world.waypoints + self.world.waypoints
                waypoints = waypoints[idx:idx+LOOKAHEAD_WPS]
                for i in range(LOOKAHEAD_WPS):
                    self.set_waypoint_velocity(waypoints, i, 20)
                lane = Lane()
                lane.header.frame_id = '/world'
                lane.header.stamp = rospy.Time.now()
                lane.waypoints = waypoints
                self.pub.publish(lane)
            rate.sleep()

    def pose_cb(self, msg):
        if self.ego is None or self.ego.header.seq < msg.header.seq:
            # TODO: Calculate ego's velocity (and maybe acceleration?) if needed
            self.ego = msg

    def waypoints_cb(self, waypoints):
        if self.world is None or self.world.header.seq < waypoints.header.seq:
            self.world = waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message.
        # We will implement it later
        pass

    def find_next_waypoint(self):
        min_dist = 1e10
        min_idx = -1
        if self.ego and self.world:
            # Find the closest waypoint
            for i in range(len(self.world.waypoints)):
                wp = self.world.waypoints[i]
                dl = self.dist(self.ego.pose.position, wp.pose.pose.position)
                if dl < min_dist:
                    min_dist = dl
                    min_idx = i

            # Check if we are behind or past the closest waypoint
            wp = self.world.waypoints[min_idx]
            pos = self.ego.pose.position
            pos.x += self.ego.pose.orientation.x
            pos.y += self.ego.pose.orientation.y
            if self.dist(wp.pose.pose.position, pos) > min_dist:
                min_idx = (min_idx + 1) % len(self.world.waypoints)
        return min_idx

    @staticmethod
    def get_waypoint_velocity(waypoint):
        return waypoint.twist.twist.linear.x

    @staticmethod
    def set_waypoint_velocity(waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    @staticmethod
    def dist(pos1, pos2):
        """
        Return the Euclidean distance between two points
        
        :param pos1: geometry_msgs/Point 
        :param pos2: geometry_msgs/Point
        :return: Euclidean distance between two points
        """
        return math.sqrt((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2 + (pos1.z - pos2.z) ** 2)

    @staticmethod
    def distance(waypoints, wp1, wp2):
        dl = 0
        for i in range(wp1, wp2+1):
            dl += WaypointUpdater.dist(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dl


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
