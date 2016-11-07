#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
#from people_msgs import People

class GoToPose():
    
    def __init__(self):
        self.goal_sent = False
        rospy.on_shutdown(self.shutdown)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")
        self.move_base.wait_for_server(rospy.Duration(5))

    def goto(self, pos, quat):
        # Send a goal
        self.goal_sent = True
        goal = MoveBaseGoal()

        #=============map has been changed to camera_rgb_frame=============#
        goal.target_pose.header.frame_id = 'camera_rgb_frame'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000), Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

        # Start moving, a maximum time of 60s being set to finish the goal
        self.move_base.send_goal(goal)
        success = self.move_base.wait_for_result(rospy.Duration(60)) 
        state = self.move_base.get_state()
        result = False
        if success and state == GoalStatus.SUCCEEDED:
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)


def callback_people(Point_Data):
    global Goal_To_Be_Update
    Goal_To_Be_Update = Point_Data

def callback_point(People_Data):
    global Goal_To_Be_Update
    if (len(People_Data.people)):
        First_Person = People_Data.people[0]
        Goal_To_Be_Update = First_Person.position


if __name__ == '__main__':

    try:
        global Goal_To_Be_Sent
        global Goal_To_Be_Update
        Goal_To_Be_Sent = (1,0,0)
        Goal_To_Be_Update = (1,0,0)
        rospy.init_node('Follow_Node', anonymous=False)
        rospy.Subscriber('/body_tracker/people', Point, callback=callback_people, queue_size=10)
        #rospy.Subscriber('/body_tracker/people', People, callback=callback_point, queue_size=10)
        navigator = GoToPose()
        
        while True:
            if Goal_To_Be_Sent == Goal_To_Be_Update:
                Goal_To_Be_Sent = Goal_To_Be_Update
                position = {'x': Goal_To_Be_Sent[0], 'y' : Goal_To_Be_Sent[1]}
                quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 0.000}
                rospy.loginfo("========== Goal has been set! ==========")
                success = navigator.goto(position, quaternion)
                if success:
                    rospy.loginfo("========== Wow, goal finished! ==========")
                else:
                    rospy.loginfo("========== Oops, goal failed! ==========")
            else:
                rospy.loginfo("========== No new goals found! ==========")
            rospy.sleep(5)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")

