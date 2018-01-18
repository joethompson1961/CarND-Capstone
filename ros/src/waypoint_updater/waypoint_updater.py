#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from std_msgs.msg import Bool
import copy
import math
import scipy.interpolate as inter
import numpy as np
#import pylab as plt

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 1.0 

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.all_waypoints = None
        self.pose = None
        self.num_waypoints = 0
        self.redLight_wp = -1
        self.obstacle_wp = -1 
        self.current_velocity = 0.0
        self.speed_lmt = 0.0
        self.dbw = False
        self.loop()

    def loop(self):
        rate = rospy.Rate(5) # 5Hz
        prev_closest = -1
        stopping = 1
        cruising = 0
        accelerating = 0
        stopHere = 0
        ref_vel = 0.0
        brake_wp_start = 0
        accel_wp_start = 0
        direction = 1 #assume it goes up
        while not rospy.is_shutdown():
            closest = 0
            #start with a long distance since this number will be decreasing as waypoints are measured
            dist = 1000000.0
            if (self.all_waypoints != None) and (self.pose != None):

                #search for the closest point to the pose and store index
                #only look close to the last closer point instead of all points
                pose = copy.deepcopy(self.pose)
                if prev_closest == -1:
                    # first pass need to find position within all waypoints and determine direction of travel
                    for i in range(0, self.num_waypoints-1):
                        wp = self.all_waypoints[i]
                        if self.distance2(wp.pose.pose, pose) < dist:
                            dist = self.distance2(wp.pose.pose, pose)
                            closest = i
                else:
                    # after first pass only search waypoints near where we were last time
                    for i in range(-40, 40):
                        j = (i + prev_closest + self.num_waypoints) % self.num_waypoints
                        wp = self.all_waypoints[j]
                        if self.distance2(wp.pose.pose, pose) < dist:
                            dist = self.distance2(wp.pose.pose, pose)
                            closest = j
                        if ((closest - prev_closest) * direction) < 0:
                            closest = prev_closest  # don't allow noisy pose to head backwards

                #convert quaternion to roll pitch and yaw (yaw is what we need)
                explicit_quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
                roll, pitch, yaw = tf.transformations.euler_from_quaternion(explicit_quat)

                #obtain indices for 3 closest points based on the closest point
                c_0 = (closest-1 + self.num_waypoints) % self.num_waypoints
                c_1 = closest % self.num_waypoints
                c_2 = (closest+1) % self.num_waypoints

                #calculate angle of 3 closest points and pose point
                angle_0 = math.atan2(self.all_waypoints[c_0].pose.pose.position.y-pose.position.y,self.all_waypoints[c_0].pose.pose.position.x-pose.position.x)
                angle_1 = math.atan2(self.all_waypoints[c_1].pose.pose.position.y-pose.position.y,self.all_waypoints[c_1].pose.pose.position.x-pose.position.x)
                angle_2 = math.atan2(self.all_waypoints[c_2].pose.pose.position.y-pose.position.y,self.all_waypoints[c_2].pose.pose.position.x-pose.position.x)

                #calculate angle in range of 0 to 360 instead of 0 180, 0 -180
                angle_0 = (math.degrees(angle_0) + 360) % 360;
                angle_1 = (math.degrees(angle_1) + 360) % 360;
                angle_2 = (math.degrees(angle_2) + 360) % 360;
                yaw = (math.degrees(yaw) + 360) % 360;

                #calculate angle between car and 3 closest points
                angle_0 = math.fabs(angle_0 - yaw)
                angle_1 = math.fabs(angle_1 - yaw)
                angle_2 = math.fabs(angle_2 - yaw)

                #first pass only select direction based on the angle of the 2nd and 3rd closest points
                if (prev_closest == -1) and (math.fabs(angle_0-180) > math.fabs(angle_2-180)):
                    direction = -1  #goes downstream
			
    			# if closest is behind current pose select next waypoint which is hopefully in front of car
                if (angle_1 > 90) and (angle_1 < 270):
                    closest += direction   

                prev_closest = closest
                
                # If not in stopping mode then check if stop is needed                                
                if (stopping == 0):
                    if (self.redLight_wp != -1):
                        # Red light ahead. If it's within the required stopping distance range, generate
                        # trajectory that decelerate to a complete stop at the red light stopline waypoint.
                        stopHere = self.redLight_wp - closest
 
                        # Required stopping distance is the number of waypoints required to stop. It's
                        # proportional to current velocity. At 25mph it takes approximately 30 waypoints
                        # to stop without exceeding max accel and jerk requirements. The velocity conversion
                        # factor of "3.0" converts velocity (mps) to distance (waypoints).  
                        distance_to_stopline = self.distance(self.all_waypoints, closest, self.redLight_wp)
                        required_stopping_distance = self.get_waypoint_velocity(self.all_waypoints[closest]) * 2.75
                        if distance_to_stopline <= required_stopping_distance:
                            # Stop is needed
#                            rospy.logwarn("dist to stop:%f   req'd distance:%f  closest:%i  redlight:%i", distance_to_stopline, required_stopping_distance, closest, self.redLight_wp)
#                            rospy.logwarn("stopping")
                            stopping = 1
                            cruising = 0

                            # A spline is used to generate low jerk deceleration for red lights.
                            # Setup the spline anchor points
                            x = []
                            y = []
                            j = (closest - direction + self.num_waypoints) % self.num_waypoints 
                            x.append(0)
                            y.append(self.get_waypoint_velocity(self.all_waypoints[j])) # start at current velocity
                            x.append(1)
                            y.append(self.get_waypoint_velocity(self.all_waypoints[j]))
                            x.append(stopHere)
                            y.append(0)                                                 # bring it down to zero at stopline
                            for i in range (1, 20):
                                x.append(stopHere+i*10)
                                y.append(0)            
    
                            # initialize spline
                            s = inter.InterpolatedUnivariateSpline(x, y)
    
                            brake_wp_start = closest
                            # restore global waypoint velocities to speed limit (those that were overridden for last acceleration)
                            for i in range(0, LOOKAHEAD_WPS-1):
                                j = (i*direction + accel_wp_start + self.num_waypoints) % self.num_waypoints
                                self.set_waypoint_velocity(self.all_waypoints, j, self.speed_lmt)

                            # set target velocities in waypoints ahead to come to a stop at the stopline
                            for i in range(0, LOOKAHEAD_WPS-1):

                                # Use spline to gracefully bring to stop
                                ref_vel = s(i+1)
                                if ref_vel < 0.5:
                                    ref_vel = 0.0
                                if math.isnan(ref_vel):
                                    ref_vel = 0.0
 #                               rospy.logwarn("ref_vel: %f", ref_vel)

                                # Set waypoint velocity
                                j = (i*direction + closest + self.num_waypoints) % self.num_waypoints # convert "i" to global waypoint number
                                self.set_waypoint_velocity(self.all_waypoints, j, ref_vel)
                else:
                    # if stopping mode keep checking for red light to clear
                    if (self.redLight_wp == -1):

                        # when red light clears return to cruising mode
#                        rospy.logwarn("cruising")
                        stopping = 0
                        cruising = 1

                        accel_wp_start = closest
                        # restore global waypoint velocities to speed limit (those that were overridden for last braking)
                        for i in range(0, LOOKAHEAD_WPS-1):
                            j = (i*direction + brake_wp_start + self.num_waypoints) % self.num_waypoints
                            self.set_waypoint_velocity(self.all_waypoints, j, self.speed_lmt)

                        ref_vel = 0.0
                        for i in range(0, LOOKAHEAD_WPS-1):

                            # Use proportional control to gracefully bring car to speed
                            if (ref_vel < self.speed_lmt):
                                ref_vel += (1.0* (1 - ref_vel/self.speed_lmt));
                            elif (ref_vel > self.speed_lmt):
                                ref_vel -= (1.0 * (1 - self.speed_lmt/ref_vel));
 #                           rospy.logwarn("ref_vel: %f", ref_vel)

                            # Set waypoint velocity
                            j = (i*direction + closest + self.num_waypoints) % self.num_waypoints # convert "i" to global waypoint number
                            self.set_waypoint_velocity(self.all_waypoints, j, ref_vel)

                #select the N waypoints that will be published (closer ones in front of the car)
                waypoints = []
                j = (closest - direction + self.num_waypoints) % self.num_waypoints
                ref_vel = self.get_waypoint_velocity(self.all_waypoints[j])
                for i in range(0, LOOKAHEAD_WPS-1):

                    # Set waypoint velocity
                    j = (i*direction + closest + self.num_waypoints) % self.num_waypoints
                    waypoints.append(self.all_waypoints[j])

                self.publish(waypoints)

                #display the current closest waypoint           
                rospy.logwarn('waypoint#:%i  target velocity:%f mps  velocity:%f  redLight:%i', closest, self.get_waypoint_velocity(waypoints[0]), self.current_velocity, self.redLight_wp)
            
            rate.sleep()

    def publish(self, waypoints):
        lane = Lane()
        lane.header.frame_id = '/route'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = waypoints
        self.final_waypoints_pub.publish(lane)
        
    def pose_cb(self, msg):
#        rospy.logwarn("pose_cb")
        self.pose = msg.pose

    def waypoints_cb(self, msg):
        self.all_waypoints = msg.waypoints;
        self.num_waypoints = len(self.all_waypoints)
        self.speed_lmt = self.get_waypoint_velocity(self.all_waypoints[0])
        rospy.logwarn("speed_lmt: %f", self.speed_lmt)
#        for i in range(0, self.num_waypoints-1):
#            self.set_waypoint_velocity(self.all_waypoints, i, 0.0)

    # Callback for /traffic_waypoint message (traffic lights)
    def traffic_cb(self, msg):
        self.redLight_wp = msg.data

    # Callback for /obstacle_waypoint message. We will implement it later
    def obstacle_cb(self, msg):
        self.obstacle_wp = msg.data

    def current_velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear.x  # simlulator returns velocity in mps

    def dbw_enabled_cb(self, msg):
        self.dbw = msg

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def distance2(self, pose1, pose2):
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        dist = dl(pose1.position, pose2.position)
        return dist

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
