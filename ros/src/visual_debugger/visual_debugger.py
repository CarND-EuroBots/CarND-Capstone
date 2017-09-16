#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from styx_msgs.msg import Lane


class VisualDebugger(object):
    def __init__(self):
        rospy.init_node('visual_debugger')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/final_waypoints', Lane, self.waypoints_cb)

        self.pub_vehicle = rospy.Publisher('/vehicle_visualizer',
                                           Marker, queue_size=1)

        self.pub_waypoints = rospy.Publisher('/waypoints_visualizer',
                                             Marker, queue_size=1)

        rospy.spin()

    def pose_cb(self, msg):
        marker = Marker()
        marker.header.frame_id = '/world'
        marker.header.stamp = rospy.Time.now()

        marker.ns = "visual_debugger"
        marker.id = 0

        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position = msg.pose.position
        marker.pose.orientation = msg.pose.orientation

        marker.scale.x = 4.6
        marker.scale.y = 2.6
        marker.scale.z = 1.6
        marker.pose.position.z = marker.pose.position.z + marker.scale.z

        marker.color.r = .0
        marker.color.g = 1.
        marker.color.b = .0
        marker.color.a = 1.

        marker.lifetime = rospy.Duration(1)

        self.pub_vehicle.publish(marker)

    def waypoints_cb(self, lane):
        points, line_strip = Marker(), Marker()

        points.header.frame_id = "/world"
        line_strip.header.frame_id = "/world"
        points.header.stamp = rospy.Time.now()
        line_strip.header.stamp = rospy.Time.now()

        points.ns = line_strip.ns = "waypoints"
        points.action = line_strip.action = Marker.ADD
        points.pose.orientation.w = 1.
        line_strip.pose.orientation.w = 1.

        points.id = 0
        line_strip.id = 1

        points.type = Marker.POINTS
        line_strip.type = Marker.LINE_STRIP

        points.scale.x = .2
        points.scale.y = .2
        line_strip.scale.x = .1

        points.color.g = 1.
        points.color.a = 1.

        line_strip.color.b = 1.
        line_strip.color.a = 1.

        for wp in lane.waypoints:
            p = Point()
            p.x = wp.pose.pose.position.x
            p.y = wp.pose.pose.position.y

            points.points.append(p)
            line_strip.points.append(p)

        self.pub_waypoints.publish(points)
        self.pub_waypoints.publish(line_strip)

if __name__ == '__main__':
    try:
        VisualDebugger()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start visual debugger node.')
