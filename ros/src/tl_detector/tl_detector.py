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

STATE_COUNT_THRESHOLD = 2 #was 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.stop_pts = []
        

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.waypoints = rospy.wait_for_message('/base_waypoints', Lane).waypoints
        
        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

      
        print ("Waiting for base waypoints......")
        rospy.wait_for_message('/base_waypoints', Lane)
        print ("Proceeding with base waypoints....")
        # List of points that correspond to the stopping line for a given intersection. Converting to easier processing format/
        for stop_pt in self.config['stop_line_positions']:
            tl = TrafficLight()
            tl.pose.pose.position.x, tl.pose.pose.position.y, tl.pose.pose.position.z = stop_pt[0], stop_pt[1], 0
            self.stop_pts.append( self.get_closest_waypoint(tl.pose.pose.position) )
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
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

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #IMPLEMENTED
        return min( xrange(len(self.waypoints)), key = lambda p: self.dist_between_pts(pose, self.waypoints[p].pose.pose.position) )
    
    def dist_between_pts(self, a, b):
        dist = math.sqrt( (a.x - b.x)**2 + (a.y - b.y)**2 + (a.z - b.z)**2 )
        return dist
        
    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        #cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #COPIED BLOCK
        #if hasattr(self.camera_image, 'encoding'):
        #    self.attribute = self.camera_image.encoding
        #    if self.camera_image.encoding == '8UC3':
        #        self.camera_image.encoding = "rgb8"
        #else:
        #    self.camera_image.encoding = 'rgb8'
        #cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        #END

        #It seems maybe it just needs to be set to rgb8?
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
        light = None
        
        #Exit early if no waypoints are set
        if (self.waypoints is None):
            rospy.logwarn("***EXITING TRAFFIC LIGHTS EARLY")
            return -1, TrafficLight.UNKNOWN
        if (len(self.stop_pts)==0):
            rospy.logwarn("***STOP POINTS IS ZERO WHILE PROCESSING LIGHTS, EXIT EARLY***")
            return -1, TrafficLight.UNKNOWN
     
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose.position)

        #IMPLEMENTED: find the closest visible traffic light (if one exists)
        dist_func = lambda p: self.stop_pts[p] - car_position if self.stop_pts[p] >= car_position else len(self.waypoints)+self.stop_pts[p]-car_position
        index = min(xrange(len(self.stop_pts)), key = dist_func)
        light_wp = self.stop_pts[index]
        light = self.lights[index]
        
        if light:
            state_pred = self.get_light_state(light) #Main way, uses classifier
            state_act = light.state 
#            rospy.logwarn('Predicted vs actual state: %i | %i --> WP== %i',state_pred,state_act, light_wp)
            return light_wp, state_pred
        
        #Otherwise, trash and exit.    
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
