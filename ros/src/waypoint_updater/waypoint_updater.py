#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import copy
import math

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

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.all_waypoints = None
        self.pose = None
        self.num_waypoints = 0
        self.redLight_wp = -1
        self.obstacle_wp = -1 
        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            j = 0
            #start with a long distance since this number will be decreasing as waypoints are measured
            dist = 1000000.0
            if (self.all_waypoints != None) and (self.pose != None):
                #search for the closest point to the pose and store index
                #TODO: this could be improved by looking only close to the last closer point instead of all points
                for i in range(0,self.num_waypoints-1):
                    waypoint = self.all_waypoints[i]
                    if self.distance2(waypoint.pose.pose,self.pose)<dist:
                        dist = self.distance2(waypoint.pose.pose,self.pose)
                        j = i

                #convert quaternion to roll pitch and yaw (yaw is what we need)
                explicit_quat = [self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w]
                roll, pitch, yaw = tf.transformations.euler_from_quaternion(explicit_quat)
                
                #obtain indices for 3 closest points based on the closest point
                j_0 = (j-1)%self.num_waypoints
                j_1 = j%self.num_waypoints
                j_2 = (j+1)%self.num_waypoints

                #calculate angle of 3 closest points and pose point
                angle_0 = math.atan2(self.all_waypoints[j_0].pose.pose.position.y-self.pose.position.y,self.all_waypoints[j_0 ].pose.pose.position.x-self.pose.position.x)
                angle_1 = math.atan2(self.all_waypoints[j_1].pose.pose.position.y-self.pose.position.y,self.all_waypoints[j_1].pose.pose.position.x-self.pose.position.x)
                angle_2 = math.atan2(self.all_waypoints[j_2].pose.pose.position.y-self.pose.position.y,self.all_waypoints[j_2].pose.pose.position.x-self.pose.position.x)
                
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
                
                #skip a few waypoints to make sure they are in front of the car
                #TODO: it may be better to do this based on distance using distance() function
                accum = 0                
                accum += 4*direction
                closest = j + 4*direction
                
                #select the N waypoints that will be published (closer ones in front of the car)
                waypoints = []
                for i in range(LOOKAHEAD_WPS):
                    accum = accum + direction
                    #stop sending waypoints if circuit is finished                    
                    if (accum + j) >= self.num_waypoints or (accum + j) < 0:
                        break 
                    # Needs to obey max velocities in the base waypoints except for when avoiding obstacles and obeying traffic signals. 
                    waypoints.append(copy.deepcopy(self.all_waypoints[accum + j]))

                # If there's a red traffic light within the look-ahead waypoints, decelerate to a complete stop at the red light waypoint.
                # TODO: Acceleration should not exceed 10 m/s^2 and jerk should not exceed 10 m/s^3
                if (self.redLight_wp != -1):
                    if (self.redLight_wp >= closest) and (self.redLight_wp < (closest + LOOKAHEAD_WPS)):
                        stopHere = self.redLight_wp - closest
                        waypoints = self.decelerate(waypoints, stopHere)

                self.publish(waypoints)
                            
                #display the current closest waypoint           
                rospy.logwarn('waypoint#:%i  target velocity:%f MPH  redLight:%i', closest, self.get_waypoint_velocity(waypoints[0]), self.redLight_wp)
            
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
        self.all_waypoints = msg.waypoints;
        self.num_waypoints = len(self.all_waypoints)

    # Callback for /traffic_waypoint message (traffic lights)
    def traffic_cb(self, msg):
        self.redLight_wp = msg.data

    # Callback for /obstacle_waypoint message. We will implement it later
    def obstacle_cb(self, msg):
        self.obstacle_wp = msg.data

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

    def decelerate(self, waypoints, stopHere):
        last = waypoints[stopHere]
        dist = self.distance2(waypoints[0].pose.pose, last.pose.pose)
        for i in range(0,stopHere):
            dist = self.distance2(waypoints[i].pose.pose, last.pose.pose)
            vel = math.sqrt(2 * MAX_DECEL * dist)   # A minimum jerk formula would probalby work better here
            if vel < 1.:
                vel = 0.
            waypoints[i].twist.twist.linear.x = min(vel, waypoints[i].twist.twist.linear.x)
        # Set all waypoints past stopping point to 0mph
        for i in range(stopHere,len(waypoints)-1):
            waypoints[i].twist.twist.linear.x = 0.
        return waypoints

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
