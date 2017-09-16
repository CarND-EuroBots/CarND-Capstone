#!/usr/bin/env python
import rospy
import tf
import math
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

    @classmethod
    def vector_from_quaternion(cls, q):
        quaternion = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
        x = math.cos(yaw) * math.cos(pitch)
        y = math.sin(yaw) * math.cos(pitch)
        z = math.sin(pitch)
        return x, y, z

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

        marker.color.r = .0
        marker.color.g = 1.
        marker.color.a = 1.

        marker.lifetime = rospy.Duration(1)

        self.pub_vehicle.publish(marker)

    def waypoints_cb(self, lane):
        points, line_strip, orientations = Marker(), Marker(), Marker()

        # Set headers
        points.header.frame_id = "/world"
        line_strip.header.frame_id = "/world"
        orientations.header.frame_id = "/world"
        points.header.stamp = rospy.Time.now()
        line_strip.header.stamp = rospy.Time.now()
        orientations.header.stamp = rospy.Time.now()

        # Set namespace and actions for markers
        points.ns = line_strip.ns = orientations.ns = "waypoints"
        points.action = line_strip.action = orientations.action = Marker.ADD

        # Set object ids
        points.id = 0
        line_strip.id = 1
        orientations.id = 2

        # Set types of markers
        points.type = Marker.POINTS
        line_strip.type = Marker.LINE_STRIP
        orientations.type = Marker.LINE_LIST

        points.pose.orientation.w = 1.
        line_strip.pose.orientation.w = 1.
        orientations.pose.orientation.w = 1.

        points.scale.x = .2
        points.scale.y = .2
        line_strip.scale.x = .1
        orientations.scale.x = .1

        # Set colors
        points.color.g = 1.
        points.color.a = 1.

        line_strip.color.b = 1.
        line_strip.color.a = 1.

        orientations.color.r = 1.
        orientations.color.a = 1.

        for wp in lane.waypoints:
            p, p2 = Point(), Point()
            p.x = wp.pose.pose.position.x
            p.y = wp.pose.pose.position.y

            x, y, z = self.vector_from_quaternion(wp.pose.pose.orientation)
            velocity = wp.twist.twist.linear.x
            p2.x = p.x + x * velocity / 10
            p2.y = p.y + y * velocity / 10
            p2.z = p.z + z * velocity / 10

            points.points.append(p)
            line_strip.points.append(p)
            orientations.points.append(p)
            orientations.points.append(p2)

        self.pub_waypoints.publish(points)
        self.pub_waypoints.publish(line_strip)
        self.pub_waypoints.publish(orientations)


if __name__ == '__main__':
    try:
        VisualDebugger()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start visual debugger node.')
