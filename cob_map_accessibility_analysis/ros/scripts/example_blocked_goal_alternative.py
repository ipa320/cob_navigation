#!/usr/bin/env python
#
# Copyright (c) 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

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
