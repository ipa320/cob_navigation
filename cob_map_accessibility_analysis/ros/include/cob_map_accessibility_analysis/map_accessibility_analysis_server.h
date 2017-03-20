/*
* Copyright (c) 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
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

#ifndef MAP_ACCESSIBILITY_ANALYSIS_H
#define MAP_ACCESSIBILITY_ANALYSIS_H

#include <iostream>
#include <string>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <XmlRpc.h>

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/GridCells.h>

#include <image_transport/image_transport.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include <tf/transform_listener.h>

#include <cob_map_accessibility_analysis/CheckPointAccessibility.h>
#include <cob_map_accessibility_analysis/CheckPerimeterAccessibility.h>
#include <cob_3d_mapping_msgs/GetApproachPoseForPolygon.h>

// opencv
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


#include <pcl/conversions.h>
#include <pcl/point_types.h>

#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>


class MapAccessibilityAnalysis
{
public:
	MapAccessibilityAnalysis(ros::NodeHandle nh);
	~MapAccessibilityAnalysis();

protected:

	struct Pose
	{
	    float x;
	    float y;
	    float orientation; //in degree

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

	// reads out the robot footprint from a string or array
	std::vector<geometry_msgs::Point> loadRobotFootprint(XmlRpc::XmlRpcValue& footprint_list);

	// original map initializer
	void mapInit(ros::NodeHandle& nh_map);

	// dynamic obstacles map initializer using obstacles and inflated obstacles topics
	void inflationInit(ros::NodeHandle& nh);

	// dynamic obstacles map initializer using obstacles and robot radius
	void dynamicObstaclesInit(ros::NodeHandle& nh);

	// map data call-back function to get the original map and also to write original inflated map for the obstacles
	void mapDataCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg_data);

	// to create dynamic obstacles map from obstacles and inflated obstacles topics
	void inflatedObstacleDataCallback(const nav_msgs::GridCells::ConstPtr& obstacles_data, const nav_msgs::GridCells::ConstPtr& inflated_obstacles_data);

	// to create dynamic obstacles map from obstacles topic and robot radius
	void obstacleDataCallback(const nav_msgs::GridCells::ConstPtr& obstacles_data);

	// callback for service checking the accessibility of a vector of points
	bool checkPose2DArrayCallback(cob_map_accessibility_analysis::CheckPointAccessibility::Request &req, cob_map_accessibility_analysis::CheckPointAccessibility::Response &res);

	// callback for service checking the accessibility of a perimeter around a center point
	bool checkPerimeterCallback(cob_map_accessibility_analysis::CheckPerimeterAccessibility::Request &req, cob_map_accessibility_analysis::CheckPerimeterAccessibility::Response &res);

	// callback for service checking the accessibility of a perimeter around a polygon
	bool checkPolygonCallback(cob_3d_mapping_msgs::GetApproachPoseForPolygon::Request& req, cob_3d_mapping_msgs::GetApproachPoseForPolygon::Response& res);

	// reads the robot coordinates from tf
	cv::Point getRobotLocationInPixelCoordinates();

	// this function computes whether a given point (potentialApproachPose) is accessible by the robot at location robotLocation
	bool isApproachPositionAccessible(const cv::Point& robotLocation, const cv::Point& potentialApproachPose, std::vector< std::vector<cv::Point> > contours);

	// pose_p and closest_point_on_polygon in pixel coordinates!
	void computeClosestPointOnPolygon(const cv::Mat& map_with_polygon, const Pose& pose_p, Pose& closest_point_on_polygon);

	template <class T>
	T convertFromMeterToPixelCoordinates(const Pose& pose);

	template <class T>
	T convertFromPixelCoordinatesToMeter(const Pose& pose);

	ros::NodeHandle node_handle_;

	ros::Subscriber map_msg_sub_;		// subscriber to the map topic
	bool map_data_recieved_;			// flag whether the map has already been received by the node

	image_transport::ImageTransport* it_;
	image_transport::Publisher inflated_map_image_pub_;
	bool publish_inflated_map_;
	message_filters::Subscriber<nav_msgs::GridCells> obstacles_sub_;
	message_filters::Subscriber<nav_msgs::GridCells> inflated_obstacles_sub_;
	typedef message_filters::sync_policies::ApproximateTime<nav_msgs::GridCells, nav_msgs::GridCells> InflatedObstaclesSyncPolicy;
	boost::shared_ptr<message_filters::Synchronizer<InflatedObstaclesSyncPolicy> > inflated_obstacles_sub_sync_; //< Synchronizer
	double obstacle_topic_update_rate_;				// defines how often this topic should be updated (set to a low rate to save ressources of your computing system)
	ros::Duration obstacle_topic_update_delay_;		// the inverse of the update rate
	ros::Time last_update_time_obstacles_;			// time of last update from the obstacle topic

	tf::TransformListener tf_listener_;

	ros::ServiceServer map_points_accessibility_check_server_;	// server handling requests for checking the accessibility of a set of points
	ros::ServiceServer map_perimeter_accessibility_check_server_;	// server handling requests for checking the accessibility of any point on the perimeter of a given position
	ros::ServiceServer map_polygon_accessibility_check_server_;	// server handling requests for checking the accessibility of any point around a given polygon (obeying the safety margin around the polygon)

	// maps
	cv::Mat original_map_;
	cv::Mat inflated_original_map_;		// contains only the inflated static obstacles
	cv::Mat inflated_map_;				// contains inflated static and dynamic obstacles

	boost::mutex mutex_inflated_map_;		// mutex for access on inflated_map

	// map properties
	double map_resolution_; // in [m/cell]
	double inverse_map_resolution_; // in [cell/m]
	cv::Point2d map_origin_; // in [m]
	std::string map_link_name_;

	// robot
	std::vector<geometry_msgs::Point> footprint_;	// polygonal footprint of the robot in [m]
	double robot_radius_; // in [m]
	std::string robot_base_link_name_;
	bool approach_path_accessibility_check_;		// if true, the path to a goal position must be accessible as well

};

#endif	//MAP_ACCESSIBILITY_ANALYSIS_H
