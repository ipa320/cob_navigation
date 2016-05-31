#!/usr/bin/env python

import rospy
import random
import math
from geometry_msgs.msg import Pose2D
from cob_map_accessibility_analysis.blocked_goal_alternative import BlockedGoalAlternative

if __name__ == "__main__":

  rospy.init_node("blocked_goal_alternative_node")
  bga = BlockedGoalAlternative()
  
  pose2d = Pose2D()
  pose2d.x = random.uniform(-10.0, 10.0)
  pose2d.y = random.uniform(-10.0, 10.0)
  pose2d.theta = random.uniform(-math.pi, math.pi)

  rospy.loginfo("Incoming Nav Goal")
  print pose2d
  
  (success, goal) = bga.check_nav_goal(pose2d)
  if not success:
      (success, goal) = bga.check_perimeter(pose2d)
