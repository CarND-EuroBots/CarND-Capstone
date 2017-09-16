#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker


class VisualDebugger(object):
    def __init__(self):
        rospy.init_node('visual_debugger')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.pub = rospy.Publisher('/visualization_marker',
                                   Marker, queue_size=1)
        self.x = 0
        self.dir = 1
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

        marker.color.r = .0
        marker.color.g = 1.
        marker.color.b = .0
        marker.color.a = 1.

        marker.lifetime = rospy.Duration(1)

        self.x = self.x + self.dir
        if self.x > 1000 or self.x < -1000:
            self.dir = -self.dir
        self.pub.publish(marker)

if __name__ == '__main__':
    try:
        VisualDebugger()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start visual debugger node.')
