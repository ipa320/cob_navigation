#!/usr/bin/env python
#
# Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


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
    print(pose2d)

    (success, goal) = bga.check_nav_goal(pose2d)
    if not success:
        (success, goal) = bga.check_perimeter(pose2d)
