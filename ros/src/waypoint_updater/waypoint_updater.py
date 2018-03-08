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

        self.speed_lmt = rospy.get_param('/waypoint_loader/velocity') * 1000 / 3600. # m/s 
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.all_waypoints = None
        self.pose = None
        self.num_waypoints = 0
        self.redLight_wp = -1
        self.obstacle_wp = -1 
        self.current_velocity = 0.0
        self.dbw = False
        self.direction = 1 #assume it goes up
        self.first_pass = True
        
        self.loop()

    # keep it valid when course wraps around to the beginning
    def wrap_course(self, waypoint):
        return (waypoint + self.num_waypoints) % self.num_waypoints

    # convert index to global waypoint number
    def convert_to_global(self, j, base):
        j = j * self.direction + base
        j = self.wrap_course(j)
        return j 

    def loop(self):
        rate = rospy.Rate(5) # 5Hz
        closest = 0
        prev_closest = -1
        policy_ACCEL = 0
        policy_STOP = 1
        policy_CRUISE = 2
        policy = policy_ACCEL
        stopHere = 0
        ref_vel = 0.0
        restore_wp_start = 0
        while not rospy.is_shutdown():
            if (self.all_waypoints != None) and (self.pose != None):
                # search for waypoint that's closest to current pose
                dist = 1000000.0 #start with a long distance since this number will be decreasing as waypoints are measured
                pose = copy.deepcopy(self.pose)
                if self.first_pass == True:
                    # on first pass search all waypoints to find position and determine direction of travel
                    for i in range(0, self.num_waypoints-1):
                        wp = self.all_waypoints[i]
                        if self.distance2(wp.pose.pose, pose) < dist:
                            dist = self.distance2(wp.pose.pose, pose)
                            closest = i
                else:
                    # after first pass search waypoints near where we were last time (for improved performance)
                    for i in range(-40, 40):
                        j = i + prev_closest  # convert to global waypoint number around the previous closest point  
                        j = self.wrap_course(j)
                        wp = self.all_waypoints[j]
                        if self.distance2(wp.pose.pose, pose) < dist:
                            dist = self.distance2(wp.pose.pose, pose)
                            closest = j
                        if ((closest - prev_closest) * self.direction) < 0:
                            closest = prev_closest  # don't allow noisy pose to head backwards

                #convert quaternion to roll pitch and yaw (yaw is what we need)
                explicit_quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
                roll, pitch, yaw = tf.transformations.euler_from_quaternion(explicit_quat)
                yaw = math.degrees(yaw)

                #obtain indices for 3 closest points based on the closest point
                c_0 = self.wrap_course(closest - 1)
                c_1 = self.wrap_course(closest)
                c_2 = self.wrap_course(closest + 1)

                #calculate angle from 3 closest points to pose point
                angle_0 = math.atan2(self.all_waypoints[c_0].pose.pose.position.y - pose.position.y, self.all_waypoints[c_0].pose.pose.position.x - pose.position.x)
                angle_0 = math.degrees(angle_0)

                angle_1 = math.atan2(self.all_waypoints[c_1].pose.pose.position.y - pose.position.y, self.all_waypoints[c_1].pose.pose.position.x - pose.position.x)
                angle_1 = math.degrees(angle_1)

                angle_2 = math.atan2(self.all_waypoints[c_2].pose.pose.position.y - pose.position.y, self.all_waypoints[c_2].pose.pose.position.x - pose.position.x)
                angle_2 = math.degrees(angle_2)

                #normalize angles to range of 0 to 360 instead of 0 180, 0 -180
                angle_0 = (angle_0 + 360) % 360
                angle_1 = (angle_1 + 360) % 360
                angle_2 = (angle_2 + 360) % 360
                yaw     = (yaw + 360) % 360;

                #make angles relative to ego car
                angle_0 = math.fabs(angle_0 - yaw)
                angle_1 = math.fabs(angle_1 - yaw)
                angle_2 = math.fabs(angle_2 - yaw)

                # first pass only: select direction based on the angle of the 2nd and 3rd closest points
                if (self.first_pass == True) and (math.fabs(angle_0-180) > math.fabs(angle_2-180)):
                    self.direction = -1  #goes downstream
			
    			# if closest is behind current pose select next waypoint which is hopefully in front of car
                if (angle_1 > 90) and (angle_1 < 270):
                    closest += self.direction   
                closest = self.wrap_course(closest)
                prev_closest = closest

                if self.dbw == False:
                    continue;
                
                # first pass only: initialize waypoints to accelerate the car
                if self.first_pass == True:
                    policy = policy_ACCEL
                    restore_wp_start = closest

                    # set initial waypoint velocites to accelerate the car
                    ref_vel = 0.0
                    for i in range(0, LOOKAHEAD_WPS-1):
                        # Use proportional control to gracefully bring car to speed
                        P = 1.25
                        if (ref_vel < self.speed_lmt):
                            ref_vel += (P * (1 - ref_vel/self.speed_lmt));
                        elif (ref_vel > self.speed_lmt):
                            ref_vel -= (P * (1 - self.speed_lmt/ref_vel));

                        # Set waypoint velocity
                        j = self.convert_to_global(i, closest)
                        self.set_waypoint_velocity(self.all_waypoints, j, ref_vel)
                self.first_pass = False

                # check for need to stop if red light ahead                
                distance_to_stopline = 100000.
                required_stopping_distance = 0.
                rwp = self.redLight_wp  # make a copy to product from callback updates in middle of loop
                wp_v = self.get_waypoint_velocity(self.all_waypoints, closest)  # target velocity at current waypoint
                if rwp != -1:
                    # there's a red light somewhere ahead
                    rwp -= 2  # adjust slightly to make front of car stop at stopline
                    
                    # distance to stop line
                    distance_to_stopline = self.distance2(pose, self.all_waypoints[rwp].pose.pose)
                    
                    # how much distance is needed to stop gently:  d = v**2/2*a
                    a = 1.0  # allows for a gentle deceleration of 1mpss
                    required_stopping_distance = (wp_v*wp_v)/(2*a)
                    
                    #rospy.logwarn("waypoint#:%i  target velocity:%f mps  req'd dist:%f  dist:%f  redlight:%i", closest, wp_v, required_stopping_distance, distance_to_stopline, rwp)

                if (policy == policy_ACCEL):
                    if (wp_v == self.speed_lmt):  # if acceleration phase is complete then switch to cruising mode
                        rospy.logwarn("cruising")
                        policy = policy_CRUISE
#                         for i in range(0, LOOKAHEAD_WPS-1):
#                             # Maintain speed in waypoints ahead
#                             j = self.convert_to_global(i, closest)
#                             self.set_waypoint_velocity(self.all_waypoints, j, self.speed_lmt)
                # If not in stopping mode then check if stop is needed                            
                if (policy == policy_ACCEL) or (policy == policy_CRUISE):
                    if distance_to_stopline <= required_stopping_distance:
                        rospy.logwarn("stopping")
                        policy = policy_STOP

                        # generate a trajectory that decelerates to a complete stop at the red light stopline waypoint.
                        stopHere = rwp - closest - 3 # stop distance expressed in waypoints
                        if stopHere < 2:
                            stopHere = 2;
 
                        # A spline is used to generate low jerk deceleration for red lights.
                        # Setup the spline anchor points
                        x = []
                        y = []
                        x.append(0)
                        y.append(wp_v) # start at current velocity
                        x.append(1)
                        y.append(wp_v) # another point at current velocity
                        x.append(stopHere)
                        y.append(0)                         # bring it down to zero at stopline
                        for i in range (10, 200, 10):                                  
                            x.append(stopHere+i)
                            y.append(0)                     # keep it at zero for many points past the stopline

                        # initialize spline with anchor points
                        s = inter.InterpolatedUnivariateSpline(x, y)

                        # set target velocities in waypoints ahead to come to a stop at the stopline
                        for i in range(0, LOOKAHEAD_WPS-1):
 
                            # Use spline to gracefully bring to stop
                            ref_vel = s(i+1)
                            if math.isnan(ref_vel):
                                ref_vel = 0.0
                            if ref_vel < 0.05:
                                ref_vel = 0.0
 
                            # Set waypoint velocity
                            j = self.convert_to_global(i, closest)
                            self.set_waypoint_velocity(self.all_waypoints, j, ref_vel)
                elif policy == policy_STOP:
                    # if stopping mode keep checking for red light to clear or to detect next light in far off distance
                    if (rwp == -1) or (rwp > (closest+150)):
                        # red light has cleared, switch to accelerating mode
                        rospy.logwarn("accelerating")
                        policy = policy_ACCEL

                        # begin accelerating from current target speed - capture before it's clobbered in restore loop just below
                        ref_vel = self.get_waypoint_velocity(self.all_waypoints, closest)

                        # set new waypoint velocites to accelerate the car
                        P = 1.25
                        ref_vel += 0.5  # add small acceleration to get the car moving when at full stop.
                        for i in range(0, LOOKAHEAD_WPS-1):
                            # Set waypoint velocity
                            j = self.convert_to_global(i, closest)
                            self.set_waypoint_velocity(self.all_waypoints, j, ref_vel)

                            # Use proportional control to gracefully bring car up to speed
                            if (ref_vel < self.speed_lmt):
                                ref_vel += (P * (1 - ref_vel/self.speed_lmt))
                            if ref_vel > self.speed_lmt:
                                ref_vel = self.speed_lmt

                #select the N waypoints that will be published (the ones in front of the car)
                waypoints = []
                for i in range(0, LOOKAHEAD_WPS-1):
                    j = self.convert_to_global(i, closest)
                    waypoints.append(self.all_waypoints[j])

                self.publish(waypoints)

                #display the current closest waypoint           
                rospy.logwarn('waypoint#:%i  target velocity:%f mps  velocity:%f  redLight:%i', closest, self.get_waypoint_velocity(waypoints, 0), self.current_velocity, rwp)
            
            rate.sleep()

    def publish(self, waypoints):
        lane = Lane()
        lane.header.frame_id = '/route'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = waypoints
        self.final_waypoints_pub.publish(lane)
        
    def pose_cb(self, msg):
        self.pose = msg.pose

    def waypoints_cb(self, msg):
        self.all_waypoints = msg.waypoints
        self.num_waypoints = len(self.all_waypoints)
        rospy.logwarn('waypoints_cb()')

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
#        rospy.logwarn('drive by wire: %s', msg)        

    def get_waypoint_velocity(self, waypoints, waypoint):
        return copy.deepcopy(waypoints[waypoint].twist.twist.linear.x)

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = copy.deepcopy(velocity)

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

    # convert kilometers per hour to meters per sec
    def kmph2mps(self, velocity_kmph):
        return (velocity_kmph * 1000.) / (60. * 60.)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
