#!/usr/bin/env python
# The script is based on spencer-project, https://github.com/spencer-project/spencer_people_tracking
# Subscribes to the topic /spencer/perception/tracked_persons, publishs velocity to drive the robot

import rospy
import math
from geometry_msgs.msg import PoseArray
from spencer_tracking_msgs.msg import DetectedPersons
from spencer_tracking_msgs.msg import TrackedPersons
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist, Vector3

# Unit of distance is meter, angle is degree
DISTANCE_UPPER_LIMIT = 1.0
DISTANCE_LOWER_LIMIT = 0.5
ANGLE_UPPER_LIMIT = 10
ANGLE_LOWER_LIMIT = -10

def callback_people(pose_array):
    global Linear_Vel
    global Angular_Vel
    global Last_detected_position
    poses = pose_array.poses
    people_num = len(poses)
    if (people_num==1):
        Any_tracked_person = poses[0]
        Tracked_position = Any_tracked_person.position
        distance_dep = Tracked_position.z
        distance_hor = -Tracked_position.x
        Last_detected_position = Tracked_position
    # To avoid position jumping when more than one people is detected 
    elif (people_num>1):
        target_people_index = 0
        minimum_distance = 100
        for index in range(people_num):
            Any_tracked_person = poses[index]
            Tracked_position = Any_tracked_person.position
            distance_from_last_position = (Tracked_position.x - Last_detected_position.x)**2 \
                + (Tracked_position.y - Last_detected_position.y)**2 \
                + (Tracked_position.x - Last_detected_position.x)**2
            if(distance_from_last_position < minimum_distance):
                minimum_distance = distance_from_last_position
                target_people_index = index
        Any_tracked_person = poses[index]
        Tracked_position = Any_tracked_person.position
        distance_dep = Tracked_position.z
        distance_hor = -Tracked_position.x
        Last_detected_position = Tracked_position
    else:
        distance_dep = 0
        distance_hor = 0

    if distance_dep != 0:    
        angular = (math.atan(distance_hor/distance_dep)*360)/(2*math.pi)
    else:
        angular = 0
    # linear velocity controller
    if distance_dep > DISTANCE_UPPER_LIMIT:
        Linear_Vel = 0.1 + 0.4*(distance_dep - DISTANCE_UPPER_LIMIT)  
    elif distance_dep < DISTANCE_LOWER_LIMIT and distance_dep != 0:
        Linear_Vel = -0.1
    else:
        Linear_Vel = 0

    # angular velocity controller
    if angular > ANGLE_UPPER_LIMIT:
        Angular_Vel = 0.2 + 0.6*(angular - ANGLE_UPPER_LIMIT)/angular
    elif angular < ANGLE_LOWER_LIMIT:
        Angular_Vel = -0.2 - 0.6*(angular - ANGLE_LOWER_LIMIT)/angular
    else:
        Angular_Vel = 0


def main():
    global Linear_Vel
    global Angular_Vel
    global Last_detected_position
    Linear_Vel = 0
    Angular_Vel = 0
    Last_detected_position = Point(0,0,0)
    LAST_VELOCITY = 0 # for velocity smoothing    

    rospy.init_node('Follow_Node')
    rospy.Subscriber('/spencer/perception_internal/people_detection/rgbd_front_top/upper_body_detector/bounding_box_centres', PoseArray, callback=callback_people, queue_size=10)
    vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)

    rate = rospy.Rate(3)
    while not rospy.is_shutdown():
        Linear_Vel = (LAST_VELOCITY + Linear_Vel)/2
        if Linear_Vel < 0.01:
            Linear_Vel = 0 
        vel_msg = Twist(Vector3(Linear_Vel,0,0), Vector3(0,0,Angular_Vel))
        vel_pub.publish(vel_msg)
        LAST_VELOCITY = Linear_Vel
        print ("velocity:(%s,0,0) (0,0,%s)"%(Linear_Vel,Angular_Vel))
        rate.sleep()

if __name__=='__main__':
    main()
