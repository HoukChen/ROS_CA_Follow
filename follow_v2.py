#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist, Vector3

# Unit of distance is meter, angle is degree
DISTANCE_UPPER_LIMIT = 1.0
DISTANCE_LOWER_LIMIT = 0.5
ANGLE_UPPER_LIMIT = 10
ANGLE_LOWER_LIMIT = -10

def callback_people(People_Position):
    global Linear_Vel
    global Angular_Vel
    
    distance_dep = People_Position.z/1000
    distance_hor = People_Position.x/1000
    if distance_dep != 0:    
        angular = (math.atan(distance_hor/distance_dep)*360)/(2*math.pi)
    else:
		angular = 0
    # linear velocity controller
    if distance_dep > DISTANCE_UPPER_LIMIT:
        Linear_Vel = 0.1 + 0.3*(distance_dep - DISTANCE_UPPER_LIMIT)  
    elif distance_dep < DISTANCE_LOWER_LIMIT and distance_dep != 0:
        Linear_Vel = -0.1
    else:
        Linear_Vel = 0

    # angular velocity controller
    if angular > ANGLE_UPPER_LIMIT:
        Angular_Vel = 0.2 + 0.4*(angular - ANGLE_UPPER_LIMIT)/angular
    elif angular < ANGLE_LOWER_LIMIT:
        Angular_Vel = -0.2 - 0.4*(angular - ANGLE_LOWER_LIMIT)/angular
    else:
        Angular_Vel = 0


def main():
    global Linear_Vel
    global Angular_Vel
    Linear_Vel = 0
    Angular_Vel = 0
    LAST_VELOCITY = 0 # for velocity smoothing    

    rospy.init_node('Follow_Node')
    rospy.Subscriber('/nearest/position', Point, callback=callback_people, queue_size=10)
    vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)

    rate = rospy.Rate(3)
    while not rospy.is_shutdown():
        Linear_Vel = (LAST_VELOCITY + Linear_Vel)/2
        vel_msg = Twist(Vector3(Linear_Vel,0,0), Vector3(0,0,Angular_Vel))
        vel_pub.publish(vel_msg)
        LAST_VELOCITY = Linear_Vel
        print ("velocity:(%s,0,0) (0,0,%s)"%(Linear_Vel,Angular_Vel))
        rate.sleep()

if __name__=='__main__':
    main()
