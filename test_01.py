#!/usr/bin/env python
# The script is based on spencer-project, https://github.com/spencer-project/spencer_people_tracking
# Subscribes to the topic /spencer/perception/tracked_persons, publishs velocity to drive the robot

import rospy
import math
from spencer_tracking_msgs.msg import DetectedPersons
from spencer_tracking_msgs.msg import TrackedPersons
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist, Vector3

# Unit of distance is meter, angle is degree
DISTANCE_UPPER_LIMIT = 1.2
DISTANCE_LOWER_LIMIT = 0.5
ANGLE_UPPER_LIMIT = 10
ANGLE_LOWER_LIMIT = -10
LINEAR_ACCELERATION = 0.4
LINEAR_BASE_VELOCITY = 0.1
ANGULAR_ACCELERATION = 0.3
ANGULAR_BASE_VELOCITY = 0.2

#LAST_VELOCITY_INFO = [0,0,0] # Recording info of last velocity ( Linear_Vel, Angular_Vel, and countinus_loss_track_count ) 

def callback_people(tracked_persons):
    global Linear_Vel
    global Angular_Vel
    global LAST_VELOCITY_INFO
    # By default, no person is tracked
    distance_dep = 0
    distance_hor = 0
    tracks = tracked_persons.tracks

    # If there are many tracked persons, follow the high_priority person. The smaller ID, the higher priority.
    distance_dep = 0
    distance_hor = 0
    priority_id = 100
    priority_index = 100   

    for index in range(len(tracks)): 
    	if tracks[index].track_id < priority_id and tracks.is_matched:
    		priority_id = tracks[index].track_id
    		priority_index = index

	if priority_id != 100:    		
		Target_tracked_person = tracks[priority_index]
		Tracked_position = Target_tracked_person.pose.pose.position
		distance_dep = Tracked_position.x
		distance_hor = Tracked_position.y

		if distance_dep > 4 or distance_dep < 2 or distance_hor < -1 or distance_hor > 1:
			distance_dep = 0
    		distance_hor = 0			


    if distance_dep != 0:    
        angular = (math.atan(distance_hor/distance_dep)*360)/(2*math.pi)
    else:
        angular = 0
    # linear velocity controller
    if distance_dep > DISTANCE_UPPER_LIMIT:
        Linear_Vel = LINEAR_BASE_VELOCITY + LINEAR_ACCELERATION*(distance_dep - DISTANCE_UPPER_LIMIT)  
    elif distance_dep < DISTANCE_LOWER_LIMIT and distance_dep != 0:
        Linear_Vel = -LINEAR_BASE_VELOCITY
    else:
        Linear_Vel = 0

    # angular velocity controller
    if angular > ANGLE_UPPER_LIMIT:
        Angular_Vel = ANGULAR_BASE_VELOCITY + ANGULAR_ACCELERATION*(angular - ANGLE_UPPER_LIMIT)/angular
    elif angular < ANGLE_LOWER_LIMIT:
        Angular_Vel = -ANGULAR_BASE_VELOCITY - ANGULAR_ACCELERATION*(angular - ANGLE_LOWER_LIMIT)/angular
    else:
        Angular_Vel = 0


def main():
    global Linear_Vel
    global Angular_Vel
    Linear_Vel = 0
    Angular_Vel = 0
    LAST_VELOCITY = 0 # for velocity smoothing    

    rospy.init_node('Follow_Node')
    rospy.Subscriber('/spencer/perception/tracked_persons', TrackedPersons, callback=callback_people, queue_size=10)
    vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)

    rate = rospy.Rate(5)
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
