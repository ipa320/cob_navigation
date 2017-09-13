/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 

#pragma once

#include <vector>

#include <opencv2/opencv.hpp>

class MapAccessibilityAnalysis
{
public:

	struct Pose
	{
		float x;
		float y;
		float orientation;   // usually in [rad]

		Pose()
		{
			x = 0;
			y = 0;
			orientation = 0;
		}

		Pose(float x_, float y_, float orientation_)
		{
			x = x_;
			y = y_;
			orientation = orientation_;
		}
	};


	MapAccessibilityAnalysis();
	~MapAccessibilityAnalysis();

	/**
	 * Inflates the walls and obstacles in a map by the radius of the robot, resulting in a map of accessible areas for the robot.
	 *
	 * \param robot_radius_pixel Provides the robot radius converted to pixels in the given maps, in [px]
	 */
	void inflateMap(const cv::Mat& original_map, cv::Mat& inflated_map, const int robot_radius_pixel);

	/**
	 * This function checks a vector of map positions for their accessibility.
	 *
	 * \param points_to_check All the map positions whose accessibility shall be tested, in map coordinates [px]
	 * \param accessibility_flags Response to each map position whether it is accessible (true) or not (false)
	 * \param inflated_map The map that is to be tested with walls and obstacles inflated by the robot radius
	 * \param approach_path_accessibility_check If true, there must be a path from robot_location to a goal position in order to report accessibility
	 * \param robot_location [Optional] The current robot position in the map if approach_path_accessibility_check=true, in map coordinates [px]
	 */
	void checkPoses(const std::vector<cv::Point>& points_to_check, std::vector<bool>& accessibility_flags,
			const cv::Mat& inflated_map, const bool approach_path_accessibility_check, const cv::Point& robot_location);

	/**
	 * This function checks the accessibility of points on a perimeter around a center point.
	 *
	 * \param accessible_poses_on_radius The returned set of accessible poses on the perimeter of the given circle, in map coordinates [px,px,rad]
	 * \param center Center of the circle whose perimeter should be checked for accessibility, in map coordinates [px,px,rad]
	 * \param radius Radius of the circle whose perimeter should be checked for accessibility, in [px]
	 * \param rotational_sampling_step Rotational sampling step width for checking points on the perimeter, in [rad]
	 * \param inflated_map The map that is to be tested with walls and obstacles inflated by the robot radius
	 * \param approach_path_accessibility_check If true, there must be a path from robot_location to a goal position in order to report accessibility
	 * \param robot_location [Optional] The current robot position in the map if approach_path_accessibility_check=true, in map coordinates [px]
	 */
	void checkPerimeter(std::vector<Pose>& accessible_poses_on_perimeter,
			const Pose& center, const double radius, const double rotational_sampling_step,
			const cv::Mat& inflated_map, const bool approach_path_accessibility_check, const cv::Point& robot_location);

	template<class T>
	T convertFromMeterToPixelCoordinates(const Pose& pose, const cv::Point2d& map_origin, const double inverse_map_resolution)
	{
		T val;
		val.x = (pose.x - map_origin.x) * inverse_map_resolution;
		val.y = (pose.y - map_origin.y) * inverse_map_resolution;
		return val;
	}

	template<class T>
	T convertFromPixelCoordinatesToMeter(const Pose& pose, const cv::Point2d& map_origin, const double map_resolution)
	{
		T val;
		val.x = map_resolution * pose.x + map_origin.x;
		val.y = map_resolution * pose.y + map_origin.y;
		return val;
	}

protected:

	/*
	 * This function computes whether a given point (potentialApproachPose) is accessible by the robot at location robotLocation
	 */
	bool isApproachPositionAccessible(const cv::Point& robotLocation,
			const cv::Point& potentialApproachPose,
			std::vector<std::vector<cv::Point> > contours);

	/*
	 * pose_p and closest_point_on_polygon in pixel coordinates!
	 */
	void computeClosestPointOnPolygon(const cv::Mat& map_with_polygon,
			const Pose& pose_p, Pose& closest_point_on_polygon);

};

