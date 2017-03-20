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

from geometry_msgs.msg import Pose2D
from cob_map_accessibility_analysis.srv import CheckPointAccessibility, CheckPointAccessibilityRequest, CheckPointAccessibilityResponse
from cob_map_accessibility_analysis.srv import CheckPerimeterAccessibility, CheckPerimeterAccessibilityRequest, CheckPerimeterAccessibilityResponse


class BlockedGoalAlternative():

    def __init__(self):
        rospy.wait_for_service("/map_accessibility_analysis/map_points_accessibility_check")
        rospy.wait_for_service("/map_accessibility_analysis/map_perimeter_accessibility_check")

        rospy.loginfo("All map_accessibility_analysis services available")

        self.point_client = rospy.ServiceProxy(
            "/map_accessibility_analysis/map_points_accessibility_check", CheckPointAccessibility)
        self.point_req = None
        self.point_res = None
        self.perimeter_client = rospy.ServiceProxy(
            "/map_accessibility_analysis/map_perimeter_accessibility_check", CheckPerimeterAccessibility)
        self.perimeter_req = None
        self.perimeter_res = None

    def check_nav_goal(self, pose2d):
        try:
            self.point_req = CheckPointAccessibilityRequest()
            # array of points which should be checked for accessibility
            self.point_req.points_to_check.append(pose2d)
            # if true, the path to a goal position must be accessible as well
            self.point_req.approach_path_accessibility_check = False

            self.point_res = self.point_client(self.point_req)
            # rospy.loginfo("CheckPointAccessibilityResponse")
            # print self.point_res
        except rospy.ServiceException, e:
            rospy.logerr("Service call 'map_points_accessibility_check' failed: %s" % e)
            return (False, None)

        for i in range(len(self.point_req.points_to_check)):
            if self.point_res.accessibility_flags[i]:
                rospy.loginfo("Nav Goal is valid")
                # print self.point_req.points_to_check[i]
                return (True, self.point_req.points_to_check[i])

        rospy.logwarn("Nav Goal is not accessible")
        return (False, None)

    def check_perimeter(self, pose2d):
        for radius in [0.1, 0.3, 0.5, 0.7, 1.0, 2.0]:
            rospy.loginfo("Checking with perimeter radius %f", radius)
            try:
                self.perimeter_req = CheckPerimeterAccessibilityRequest()
                # center of the circle whose perimeter should be checked for accessibility, in [m,m,rad]
                self.perimeter_req.center = pose2d
                self.perimeter_req.radius = radius                   # radius of the circle, in [m]
                # rotational sampling step width for checking points on the perimeter, in [rad]
                self.perimeter_req.rotational_sampling_step = 0.0

                self.perimeter_res = self.perimeter_client(self.perimeter_req)
                # rospy.loginfo("CheckPerimeterAccessibilityResponse")
                # print self.perimeter_res
            except rospy.ServiceException, e:
                rospy.logerr("Service call 'map_perimeter_accessibility_check' failed: %s" % e)
                break

            # use first valid alternative
            if not self.perimeter_res.accessible_poses_on_perimeter == []:
                rospy.loginfo("Found valid alternative")
                # print self.perimeter_res.accessible_poses_on_perimeter[0]
                return (True, self.perimeter_res.accessible_poses_on_perimeter[0])

        rospy.logerr("No valid alternative in perimeter")
        return (False, None)
