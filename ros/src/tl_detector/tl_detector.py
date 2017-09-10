#!/usr/bin/env python
import math
import rospy
import tf
import cv2
import yaml
import numpy as np
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose, Point
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier

STATE_COUNT_THRESHOLD = 3


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.tl_waypoints = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights helps you acquire an accurate ground truth
        data source for the traffic light classifier, providing the location
        and current color state of all traffic lights in the simulator.
        This state can be used to generate classified images or subbed into
        your solution to help you work on another single component of the node.
        This topic won't be available when testing your solution in real life
        so don't rely on it in the final submission.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray,
                                self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint',
                                                      Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes
            the index of the waypoint closest to the red light to
            /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        # Publish upcoming red lights at camera frequency.
        # Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        # of times till we start using it.
        # Otherwise the previous stable state is used.
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def distance(self, p1, p2):
        """
        Distance between two map coordinates copied from WaypointLoader class.
        """
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)

    def get_closest_waypoint_idx(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """

        # Very high value is set as as initial distance.
        closest_waypoint_dist = 100000
        closest_waypoint_ind = -1

        # Looping through base waypoints to find the one closest to the car.
        for i in range(0, len(self.waypoints.waypoints)):
            waypoint_distance = self.distance(
                self.waypoints.waypoints[i].pose.pose.position,
                pose.position
            )
            if waypoint_distance < closest_waypoint_dist:
                # In case that closer waypoint has been found,
                # set new distance and new closest waypoint index.
                closest_waypoint_dist = waypoint_distance
                closest_waypoint_ind = i
        return closest_waypoint_ind

    def transform_world_to_camera(self, point_in_world):
        """
        Transforms a point from 3D world coordinates to 3D camera coordinates
        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            point_in_camera (Point): 3D location of a point in the camera frame
        """
        # Get transform between pose of camera and world frame
        trans = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link", "/world", now,
                                           rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                                                         "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        # Create transformation matrix
        T = self.listener.fromTranslationRotation(trans, rot)

        # Transform point from world to camera using homogeneous coordinates
        point_in_world_h = np.array([[point_in_world.x],
                                     [point_in_world.y],
                                     [point_in_world.z],
                                     [1.0]])

        point_in_camera_h = np.dot(T, point_in_world_h)

        # Output as a Point
        point_in_camera = Point(point_in_camera_h[0][0],
                                point_in_camera_h[1][0],
                                point_in_camera_h[2][0])
        return point_in_camera

    def project_to_image_plane(self, point_in_camera):
        """
        Projects point from 3D camera coordinates to 2D camera image location

        Args:
            point_in_camera (Point): 3D location of a point in the camera

        Returns:
            u (int): u coordinate of target point in image
            v (int): v coordinate of target point in image

        """
        # Get camera intrinsics
        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']

        # Project to image plane to get u,v image coordinates
        # Note that X is pointing forward, Y to the left and Z up
        u = int(-(fx / point_in_camera.x) * point_in_camera.y)
        v = int(-(fy / point_in_camera.x) * point_in_camera.z)

        return (u, v)

    def crop_light_image(self, light, cv_image):
        """
        Crops the input image to keep only the part that
        contains the requested traffic light

        Args:
            light (TrafficLight): light to get image of
            cv_image (cv2 image): image to crop

        Returns:
            img_traffic_light (cv2 image): the cropped image
        """
        # Get light center in camera coordinates
        light_pos_world = light.pose.pose.position
        light_pos_camera = self.transform_world_to_camera(light_pos_world)

        # Get coordinate of top-left corner of bounding box
        corner = Point(light_pos_camera.x,
                       light_pos_camera.y,
                       light_pos_camera.z)

        LIGHT_CROP_OFFSET_Y = 0.25  # [m]
        LIGHT_CROP_OFFSET_Z = 0.5   # [m]

        corner.y += LIGHT_CROP_OFFSET_Y
        corner.z += LIGHT_CROP_OFFSET_Z

        # Project the previous points to the image plane
        u_center, v_center = self.project_to_image_plane(light_pos_camera)
        u_corner, v_corner = self.project_to_image_plane(corner)

        # Compute offset w.r.t. center
        off_u = abs(u_center - u_corner)
        off_v = abs(v_center - v_corner)

        # Crop the image to get the traffic light only
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        min_u = max(0, u_center - off_u)
        max_u = min(u_center + off_u, image_width - 1)
        min_v = max(0, v_center - off_v)
        max_v = min(v_center + off_v, image_height - 1)

        img_traffic_light = cv_image[min_u:max_u][min_v:max_v]

        # Output
        return img_traffic_light

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color
                 (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        self.camera_image.encoding = "rgb8"
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # Use light location to zoom in on traffic light in image
        img_traffic_light = self.crop_light_image(light, cv_image)

        # TODO(carlos): publish the cropped image on a ROS topic

        # Get classification
        return self.light_classifier.get_classification(img_traffic_light)

    def get_tl_waypoints(self):
        """ Converts list self.config['light_positions'] with trafic
        light positions to get_closest_waypoint_idx array with PoseStamped
        traffic light waypoints

        """

        for light_position in self.config['light_positions']:
            tl_waypoint = PoseStamped()
            tl_waypoint.pose.position.x = light_position[0]
            tl_waypoint.pose.position.y = light_position[1]
            tl_waypoint.pose.position.z = 0
            self.tl_waypoints.append(
                self.get_closest_waypoint_idx(tl_waypoint.pose)
            )

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists,
           and determines its location and color

        Returns:
            int: index of waypoint closes to the upcoming traffic light
                 (-1 if none exists)
            int: ID of traffic light color
                 (specified in styx_msgs/TrafficLight)

        """

        # searching_distance_tl parameter sets distance in which
        # traffic lights will be searched as number of waypoints
        searching_distance_tl = 120

        light = None
        light_wp = -1
        if(self.pose and self.waypoints):
            # In case that array with traffic light waypoints does not exist,
            # create it
            if(self.tl_waypoints == []):
                self.get_tl_waypoints()
            car_position_wp = self.get_closest_waypoint_idx(self.pose.pose)
            # Loop thorugh waypoints to find the closest traffic ligt waypoint
            smallest_tl_distance = 10000
            for tl_waypoint in self.tl_waypoints:
                distance_between_waypoints = abs(car_position_wp-tl_waypoint)
                if(distance_between_waypoints < searching_distance_tl and
                    distance_between_waypoints < smallest_tl_distance and
                        car_position_wp < tl_waypoint):
                    light_wp = tl_waypoint
                    smallest_tl_distance = distance_between_waypoints
        # If waypoint has been found get traffic light state
        if light_wp > -1:
            light = self.waypoints.waypoints[light_wp]
            state = self.get_light_state(light)
            return light_wp, state

        return -1, TrafficLight.UNKNOWN


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
