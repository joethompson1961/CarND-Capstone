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
import pylab as plt

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
        rate = rospy.Rate(50) # 50Hz
        prev_j = -1
        stopping = 1
        cruising = 0
        stopHere = 0
        ref_vel = 0.0
        brake_wp_start = 0
        while not rospy.is_shutdown():
            j = 0
            #start with a long distance since this number will be decreasing as waypoints are measured
            dist = 1000000.0
            if (self.all_waypoints != None) and (self.pose != None):

                #search for the closest point to the pose and store index
                #only look close to the last closer point instead of all points
                pose = copy.deepcopy(self.pose)
                if prev_j == -1:
                    speed_lmt = self.get_waypoint_velocity(self.all_waypoints[0])
                    # first pass need to find position within all waypoints
                    for i in range(0, self.num_waypoints-1):
                        wp = self.all_waypoints[i]
                        if self.distance2(wp.pose.pose, pose) < dist:
                            dist = self.distance2(wp.pose.pose, pose)
                            j = i
                else:
                    # after first pass only waypoints near where we were last time
                    for k in range(0, 50):
                        i = (prev_j + k) % self.num_waypoints
                        wp = self.all_waypoints[i]
                        if self.distance2(wp.pose.pose, pose) < dist:
                            dist = self.distance2(wp.pose.pose, pose)
                            j = i
                prev_j = j

                #convert quaternion to roll pitch and yaw (yaw is what we need)
                explicit_quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
                roll, pitch, yaw = tf.transformations.euler_from_quaternion(explicit_quat)
                
                #obtain indices for 3 closest points based on the closest point
                j_0 = (j-1 + self.num_waypoints) % self.num_waypoints
                j_1 = j%self.num_waypoints
                j_2 = (j+1)%self.num_waypoints

                #calculate angle of 3 closest points and pose point
                angle_0 = math.atan2(self.all_waypoints[j_0].pose.pose.position.y-pose.position.y,self.all_waypoints[j_0].pose.pose.position.x-pose.position.x)
                angle_1 = math.atan2(self.all_waypoints[j_1].pose.pose.position.y-pose.position.y,self.all_waypoints[j_1].pose.pose.position.x-pose.position.x)
                angle_2 = math.atan2(self.all_waypoints[j_2].pose.pose.position.y-pose.position.y,self.all_waypoints[j_2].pose.pose.position.x-pose.position.x)
                
                #calculate angle in range of 0 to 360 instead of 0 180, 0 -180
                angle_0 = (math.degrees(angle_0) + 360) % 360;
                angle_1 = (math.degrees(angle_1) + 360) % 360;
                angle_2 = (math.degrees(angle_2) + 360) % 360;
                yaw = (math.degrees(yaw) + 360) % 360;
               
                #calculate angle between car and 3 closest points
                angle_0 = math.fabs(angle_0 - yaw)
                angle_1 = math.fabs(angle_1 - yaw)
                angle_2 = math.fabs(angle_2 - yaw)
               
                #select direction based on the angle of the 2nd and 3rd closest points
                direction = 1 #assume it goes up
                if math.fabs(angle_0-180) > math.fabs(angle_2-180):
                    direction = -1  #goes downstream
                if direction == -1:
                    rospy.logwarn("DOWNSTREAM")
                
                #skip a few waypoints to make sure they are in front of the car
                #TODO: it may be better to do this based on distance using distance() function
#                accum = 4*direction
#                closest = j + 4*direction

                accum = 0
                if angle_1 > 90:  # closest point is behind the car, but need closest in front of car
                    j += 1               
                closest = j

                # If not in stopping mode then check if stop is needed                                
                if (stopping == 0):
                    # If there's a red traffic light within the look-ahead waypoints, decelerate to a complete
                    # stop at the red light waypoint.
                    if (self.redLight_wp != -1):
                        stopHere = self.redLight_wp - closest
                        # Calculate number of waypoints required to stop for the current speed.
                        # Current_velocity is in mps. Each multiple of 25MPH requires approx 30 waypoints stopping distance.
                        # Required_stopping_distance is proportional to the current top speed.
                        required_stopping_points = int(math.ceil(self.get_waypoint_velocity(self.all_waypoints[closest]) * 3.0))
                        if stopHere <= required_stopping_points:
                            # Stop is needed
                            rospy.logwarn("stopping")
                            stopping = 1
                            cruising = 0

                            # A spline is used to generate low jerk deceleration for red lights.
                            # Setup the spline anchor points
                            x = []
                            y = []
                            x.append(0)
                            y.append(self.get_waypoint_velocity(self.all_waypoints[j-1]))  # start at current velocity
                            x.append(1)
                            y.append(self.get_waypoint_velocity(self.all_waypoints[j-1]))
                            x.append(stopHere-2)
                            y.append(0)            # down to zero velocity just before stopline
                            x.append(stopHere+20)
                            y.append(0)            
                            x.append(stopHere+40)
                            y.append(0)            
                            x.append(stopHere+60)
                            y.append(0)
                            x.append(stopHere+80)
                            y.append(0)
                            x.append(stopHere+100)
                            y.append(0)
                            x.append(stopHere+200)
                            y.append(0)
    
                            # initialize spline
                            s = inter.InterpolatedUnivariateSpline(x, y)
    
                            # set target velocities in waypoints ahead to come to a stop at the stopline
                            brake_wp_start = j
                            for i in range(0, LOOKAHEAD_WPS-1):
                                ref_vel = s(i+1)
                                if ref_vel < 0.5:
                                    ref_vel = 0.0
                                if math.isnan(ref_vel):
                                    ref_vel = 0.0
                                self.set_waypoint_velocity(self.all_waypoints, accum+j, ref_vel)
                                accum = (accum + direction + self.num_waypoints) % self.num_waypoints
                else:
                    # if stopping mode keep checking for red light to clear
                    if (self.redLight_wp == -1):
                        rospy.logwarn("cruising")
                        # when red light clears return to cruising mode
                        stopping = 0
                        cruising = 1
                        # restore velocities to speed limit for all previous waypoint velocities that were overridden for braking
                        for i in range(brake_wp_start, closest):
                            self.set_waypoint_velocity(self.all_waypoints, closest - i, self.speed_lmt)

                        ref_vel = 0.0
                        accum = 0
                        for i in range(0, LOOKAHEAD_WPS-1):
                            # For cruising overwrite old junk velocities in waypoints with current target velocities
                            # For stopping the waypoint velocities have already been configured up above
                            # Use proportional control to gracefully bring car to speed
                            if (ref_vel < self.speed_lmt):
                                ref_vel += (0.9* (1 - ref_vel/self.speed_lmt));
                            elif (ref_vel > self.speed_lmt):
                                ref_vel -= (0.9 * (1 - self.speed_lmt/ref_vel));
                            self.set_waypoint_velocity(self.all_waypoints, accum + closest, ref_vel)
                            accum = (accum + direction + self.num_waypoints) % self.num_waypoints

                #select the N waypoints that will be published (closer ones in front of the car)
                waypoints = []
                ref_vel = self.get_waypoint_velocity(self.all_waypoints[closest-1])
                accum = 0
                for i in range(0, LOOKAHEAD_WPS-1):
                    waypoints.append(self.all_waypoints[accum + closest])
                    accum = (accum + direction + self.num_waypoints) % self.num_waypoints

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
