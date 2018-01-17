#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math



STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.has_image = False
        self.lights = []
        self.stop_line_wp_idxs = []

        # there are stop lines before traffic lights
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_stop_line_wp_idx = -1
        self.car_wp_idx = -1
        self.last_car_wp_idx = -1
        self.state_count = 0
        self.visible_distance_wps = 200

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        # for devel test using simulator's ground truth
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_lights_cb)
        # for real test on the vehicle using traffic light classifier for camera image
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)


        rospy.spin()



    def pose_cb(self, msg):
        self.pose = msg.pose
        #rospy.logdebug('tl_detector: pose x=%s', msg.pose.position.x)

    def waypoints_cb(self, msg):
        self.waypoints = msg.waypoints
        stop_line_positions = self.config['stop_line_positions']

        for stop_line_position in stop_line_positions:
            pose = Pose()
            pose.position.x = stop_line_position[0]
            pose.position.y = stop_line_position[1]
            stop_line_wp_idx = self.get_closest_waypoint(pose)
            self.stop_line_wp_idxs.append(stop_line_wp_idx)
            rospy.loginfo('stop_line: %s, %s, [%s]', stop_line_position[0], stop_line_position[1], stop_line_wp_idx)

    def publish_traffic(self):
        stop_line_wp_idx, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
            rospy.loginfo('publish_traffic: has_image(%s), state=%s at %s(car_wp_idx=%s), %s'
                , self.has_image, state, stop_line_wp_idx, self.car_wp_idx, self.state_count)
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            stop_line_wp_idx = stop_line_wp_idx if state == TrafficLight.RED else -1
            self.last_stop_line_wp_idx = stop_line_wp_idx
            # reduce logging
            if self.state_count == STATE_COUNT_THRESHOLD:
                rospy.loginfo('publish_traffic: has_image(%s), state=%s at %s(car_wp_idx=%s), %s>=%s'
                    , self.has_image, state, stop_line_wp_idx, self.car_wp_idx
                        , self.state_count, STATE_COUNT_THRESHOLD)
            self.upcoming_red_light_pub.publish(Int32(stop_line_wp_idx))
        else:
            rospy.loginfo('publish_traffic: has_image(%s), state=%s at prev %s(car_wp_idx=%s), %s<%s'
                , self.has_image, state, self.last_stop_line_wp_idx, self.car_wp_idx
                    , self.state_count, STATE_COUNT_THRESHOLD)
            self.upcoming_red_light_pub.publish(Int32(self.last_stop_line_wp_idx))
        self.state_count += 1


    def traffic_lights_cb(self, msg):
        self.lights = msg.lights
        if not self.has_image:
            #rospy.logdebug('using ground_truth: lights len=%s', len(msg.lights))
            self.publish_traffic()

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        # camera is turned on
        self.has_image = True
        self.camera_image = msg
        self.publish_traffic()
        # there is no topic on camera on/off, so flag here. 
        # (not accurate. just for test toggling camera on/off)
        self.has_image = False


    def get_closest_waypoint(self, pose, start_wp_idx=0):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #DONE implement
        if self.waypoints is None:
            return -1

        closestLen = 100000; #large number
        closest_wp_idx = 0;

        lookup_range = 50
        # -1 or 0 for full scan
        if start_wp_idx <= 0:
            start_wp_idx = 0
            end_wp_idx = len(self.waypoints) 
        else:
            end_wp_idx = start_wp_idx + lookup_range
            end_wp_idx = min(end_wp_idx, len(self.waypoints))

        i = start_wp_idx
        x = pose.position.x
        y = pose.position.y
        for waypoint in self.waypoints[start_wp_idx:end_wp_idx]:
            map_x = waypoint.pose.pose.position.x
            map_y = waypoint.pose.pose.position.y
            dist = math.sqrt((map_x-x)**2 + (map_y-y)**2)
            if dist < closestLen:
                closestLen = dist
                closest_wp_idx = i
            i += 1

        # returns the index of the closest waypoint
        return closest_wp_idx


    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if not self.waypoints or not self.pose:
            return -1, TrafficLight.UNKNOWN

        light_wp = -1

        # List of positions that correspond to the line to stop in front of for a given intersection
        #stop_line_positions = self.config['stop_line_positions']
        self.car_wp_idx = self.get_closest_waypoint(self.pose, self.last_car_wp_idx)
        self.last_car_wp_idx = self.car_wp_idx

        #DONE find the closest visible traffic light (if one exists)
        min_dist = 100000
        for stop_line_wp_idx in self.stop_line_wp_idxs:
            dist = stop_line_wp_idx - self.car_wp_idx
            if dist >= 0 and dist < min_dist:
                min_dist = dist
                if min_dist < self.visible_distance_wps:
                    # It uses the stop_line postion rather than the traffic light position
                    light_wp = stop_line_wp_idx

        # If there is a visible traffic light
        if light_wp != -1:
            # if camera is on: through classifier
            if self.has_image:
                state = self.get_light_state(light_wp)
                return light_wp, state
            # if camera is off: through ground truth
            else:
                state = TrafficLight.UNKNOWN
                min_dist = 100000
                for light in self.lights:
                    light_x = light.pose.pose.position.x
                    light_y = light.pose.pose.position.y
                    stop_line_x = self.waypoints[light_wp].pose.pose.position.x
                    stop_line_y = self.waypoints[light_wp].pose.pose.position.y
                    dist = math.sqrt((light_x - stop_line_x)**2 + (light_y - stop_line_y)**2)
                    if dist < min_dist:
                        dist = min_dist
                        state = light.state
                return light_wp, state
            rospy.loginfo('process_traffic_lights: visible stop_line_wp: %s', light_wp)

        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('tl_detector: Could not start traffic node.')
