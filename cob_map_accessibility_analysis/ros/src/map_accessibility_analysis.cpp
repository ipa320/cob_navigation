/*
* Copyright (c) 2016-2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <cob_map_accessibility_analysis/map_accessibility_analysis.h>
#include <ros/ros.h>

//#define __DEBUG_DISPLAYS__

MapAccessibilityAnalysis::MapAccessibilityAnalysis()
{

}

MapAccessibilityAnalysis::~MapAccessibilityAnalysis()
{

}

void MapAccessibilityAnalysis::inflateMap(const cv::Mat& original_map, cv::Mat& inflated_map, const int robot_radius_pixel)
{
	cv::erode(original_map, inflated_map, cv::Mat(), cv::Point(-1, -1), robot_radius_pixel);
}

void MapAccessibilityAnalysis::checkPoses(const std::vector<cv::Point>& points_to_check, std::vector<bool>& accessibility_flags,
		const cv::Mat& inflated_map, const bool approach_path_accessibility_check, const cv::Point& robot_location)
{
#ifdef __DEBUG_DISPLAYS__
	cv::Mat display_map = inflated_map.clone();
#endif

	// find the individual connected areas
	std::vector<std::vector<cv::Point> > area_contours; // first index=contour index;  second index=point index within contour
	if (approach_path_accessibility_check == true)
	{
		cv::Mat inflated_map_copy = inflated_map.clone();
		cv::findContours(inflated_map_copy, area_contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
	}

	for (unsigned int i = 0; i < points_to_check.size(); ++i)
	{
		const int u = points_to_check[i].x;
		const int v = points_to_check[i].y;
		ROS_INFO_STREAM("Checking accessibility of point (" << u << ", " << v << ")px.");
		if (inflated_map.at<uchar>(v, u) != 0)
		{
			// check if robot can approach this position
			if (approach_path_accessibility_check == false ||
					isApproachPositionAccessible(robot_location, cv::Point(u, v), area_contours) == true)
				accessibility_flags[i] = true;
		}

#ifdef __DEBUG_DISPLAYS__
		if (accessibility_flags[i] == false)
			cv::circle(display_map, cv::Point(u, v), 2, cv::Scalar(32), 10);
		else
			cv::circle(display_map, cv::Point(u, v), 2, cv::Scalar(192), 10);
#endif
	}

#ifdef __DEBUG_DISPLAYS__
	cv::imshow("points", display_map);
	cv::waitKey();
#endif
}


void MapAccessibilityAnalysis::checkPerimeter(std::vector<Pose>& accessible_poses_on_perimeter,
		const Pose& center, const double radius, const double rotational_sampling_step,
		const cv::Mat& inflated_map, const bool approach_path_accessibility_check, const cv::Point& robot_location)
{
#ifdef __DEBUG_DISPLAYS__
	cv::Mat display_map = inflated_map.clone();
#endif

	// find the individual connected areas
	std::vector<std::vector<cv::Point> > area_contours; // first index=contour index;  second index=point index within contour
	if (approach_path_accessibility_check == true)
	{
		cv::Mat inflated_map_copy = inflated_map.clone();
		cv::findContours(inflated_map_copy, area_contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
	}

	for (double angle = center.orientation; angle < center.orientation + 2 * CV_PI; angle += rotational_sampling_step)
	{
		const double x = center.x + radius * cos(angle);
		const double y = center.y + radius * sin(angle);
		const int u = cvRound(x);
		const int v = cvRound(y);
		bool found_pose = false;
		if (inflated_map.at<uchar>(v, u) != 0)
		{
			// check if robot can approach this position
			if (approach_path_accessibility_check == false
					|| isApproachPositionAccessible(robot_location, cv::Point(u, v), area_contours) == true)
			{
				// add accessible point to results
				Pose pose;
				pose.x = x;
				pose.y = y;
				pose.orientation = angle + CV_PI;
				while (pose.orientation > 2 * CV_PI)
					pose.orientation -= 2 * CV_PI;
				while (pose.orientation < 0.)
					pose.orientation += 2 * CV_PI;
				accessible_poses_on_perimeter.push_back(pose);
				found_pose = true;
			}
		}
#ifdef __DEBUG_DISPLAYS__
		if (found_pose == true)
			cv::circle(display_map, cv::Point(u, v), 2, cv::Scalar(192), 5);
		else
			cv::circle(display_map, cv::Point(u, v), 2, cv::Scalar(32), 5);
#endif
	}

#ifdef __DEBUG_DISPLAYS__
	cv::imshow("perimeter", display_map);
	cv::waitKey();
#endif
}


bool MapAccessibilityAnalysis::isApproachPositionAccessible(const cv::Point& robotLocation,
		const cv::Point& potentialApproachPose, std::vector<std::vector<cv::Point> > contours)
{
	// check whether potentialApproachPose and robotLocation are in the same area (=same contour)
	int contourIndexRobot = -1;
	int contourIndexPotentialApproachPose = -1;
	for (unsigned int i = 0; i < contours.size(); i++)
	{
		if (0 <= cv::pointPolygonTest(contours[i], potentialApproachPose, false))
			contourIndexPotentialApproachPose = i;
		if (0 <= cv::pointPolygonTest(contours[i], robotLocation, false))
			contourIndexRobot = i;
	}
	ROS_DEBUG_STREAM("contourIndexPotentialApproachPose=" << contourIndexPotentialApproachPose
					<< "  contourIndexRobot=" << contourIndexRobot);
	if (contourIndexRobot != contourIndexPotentialApproachPose ||
			(contourIndexRobot == -1 && contourIndexPotentialApproachPose == -1))
		return false;

	return true;
}


void MapAccessibilityAnalysis::computeClosestPointOnPolygon(const cv::Mat& map_with_polygon,
		const Pose& pose_p, Pose& closest_point_on_polygon)
{
	double closest_pixel_distance_squared = 1e10;
	for (int v = 0; v < map_with_polygon.rows; v++)
	{
		for (int u = 0; u < map_with_polygon.cols; u++)
		{
			double dist_squared = 0.;
			if (map_with_polygon.at<uchar>(v, u) == 128 &&
				(dist_squared = (pose_p.x - u) * (pose_p.x - u) + (pose_p.y - v) * (pose_p.y - v)) < closest_pixel_distance_squared)
			{
				closest_pixel_distance_squared = dist_squared;
				closest_point_on_polygon.x = u;
				closest_point_on_polygon.y = v;
			}
		}
	}
}
