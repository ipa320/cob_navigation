#! /usr/bin/env python

import roslib; roslib.load_manifest('cob_linear_nav')
import rospy
import actionlib

from move_base_msgs.msg import *

if __name__ == '__main__':
    rospy.init_node('move_base_linear_client')
    client = actionlib.SimpleActionClient('move_base_linear', MoveBaseAction)
    
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "/arm_0_link"
    goal.target_pose.pose.position.x=1.0
    goal.target_pose.pose.position.y=0.0
    goal.target_pose.pose.position.z=0.0
    
    goal.target_pose.pose.orientation.x=0.0
    goal.target_pose.pose.orientation.y=0.0
    goal.target_pose.pose.orientation.z=-0.121
    goal.target_pose.pose.orientation.w=0.993
    
    # Fill in the goal here
    
    client.send_goal(goal)
    #client.cancel_goal()
    
    #client.cancel_all_goals()
    #client.wait_for_result(rospy.Duration.from_sec(5.0))
