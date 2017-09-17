#!/usr/bin/env python
import os
import sys
import rospy
import cv2
from datetime import datetime
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from styx_msgs.msg import TrafficLightArray


class TLGroundTruthLogger(object):
    def __init__(self, log_dir):
        rospy.init_node('tl_ground_truth_logger')

        self.log_folder = os.path.join(log_dir, self.get_date_time_str())
        os.makedirs(self.log_folder)
        print('Logging traffic light data in {}...'.format(self.log_folder))

        self.csv_path = os.path.join(self.log_folder, 'light_state.csv')
        self.csv_fd = open(self.csv_path, 'w')

        self.cv_bridge = CvBridge()

        self.camera_image = None
        self.lights = []

        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray,
                         self.traffic_cb, queue_size=1)
        rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1)
        rospy.Subscriber('/vehicle/visible_light_idx', Int32,
                         self.light_cb, queue_size=1)

        rospy.spin()

    @staticmethod
    def get_date_time_str():
        return datetime.utcnow().strftime('%Y%m%d_%H%M%S_%f')[:-3]

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        self.camera_image = msg

    def light_cb(self, msg):
        visible_light_idx = msg.data
        if self.lights and self.camera_image and visible_light_idx >= 0:
            self.store_image(visible_light_idx)

    def store_image(self, visible_light_idx):
        # Convert image to CV2 and write to disk
        self.camera_image.encoding = "rgb8"
        cv_image = self.cv_bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        path = os.path.join(self.log_folder,
                            '{}.png'.format(self.get_date_time_str()))
        cv2.imwrite(path, cv_image)

        # Write state to CSV
        light_state = self.lights[visible_light_idx].state
        self.csv_fd.write('{},{}\n'.format(path, light_state))


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: python tl_ground_truth_logger.py <path_to_log_folder>')
        sys.exit(1)
    else:
        try:
            TLGroundTruthLogger(sys.argv[1])
        except rospy.ROSInterruptException:
            rospy.logerr('Could not start traffic node.')
