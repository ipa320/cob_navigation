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


#ifndef MAP_ACCESSIBILITY_ANALYSIS_H
#define MAP_ACCESSIBILITY_ANALYSIS_H

#include <iostream>
#include <sstream>
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
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <pcl/conversions.h>
#include <pcl/point_types.h>

#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

#include <cob_map_accessibility_analysis/map_accessibility_analysis.h>

class MapAccessibilityAnalysisServer : public MapAccessibilityAnalysis
{
public:
  MapAccessibilityAnalysisServer(ros::NodeHandle nh);
  ~MapAccessibilityAnalysisServer();

protected:

  // reads out the robot footprint from a string or array
  // returns the polygonal footprint of the robot in [m]
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
  void inflatedObstacleDataCallback(const nav_msgs::GridCells::ConstPtr& obstacles_data,
                                    const nav_msgs::GridCells::ConstPtr& inflated_obstacles_data);

  // to create dynamic obstacles map from obstacles topic and robot radius
  void obstacleDataCallback(const nav_msgs::GridCells::ConstPtr& obstacles_data);

  // callback for service checking the accessibility of a vector of points
  bool checkPose2DArrayCallback(cob_map_accessibility_analysis::CheckPointAccessibility::Request& req,
                                cob_map_accessibility_analysis::CheckPointAccessibility::Response& res);

  // callback for service checking the accessibility of a perimeter around a center point
  bool checkPerimeterCallback(cob_map_accessibility_analysis::CheckPerimeterAccessibility::Request& req,
                              cob_map_accessibility_analysis::CheckPerimeterAccessibility::Response& res);

  // callback for service checking the accessibility of a perimeter around a polygon
  bool checkPolygonCallback(cob_3d_mapping_msgs::GetApproachPoseForPolygon::Request& req,
                            cob_3d_mapping_msgs::GetApproachPoseForPolygon::Response& res);

  // reads the robot coordinates from tf
  cv::Point getRobotLocationInPixelCoordinates();

  ros::NodeHandle node_handle_;

  ros::Subscriber map_msg_sub_;  // subscriber to the map topic
  bool map_data_recieved_;       // flag whether the map has already been received by the node

  image_transport::ImageTransport* it_;
  image_transport::Publisher inflated_map_image_pub_;
  bool publish_inflated_map_;
  message_filters::Subscriber<nav_msgs::GridCells> obstacles_sub_;
  message_filters::Subscriber<nav_msgs::GridCells> inflated_obstacles_sub_;
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::GridCells, nav_msgs::GridCells>
      InflatedObstaclesSyncPolicy;
  boost::shared_ptr<message_filters::Synchronizer<InflatedObstaclesSyncPolicy> >
      inflated_obstacles_sub_sync_;    //< Synchronizer
  double obstacle_topic_update_rate_;  // defines how often this topic should be updated (set to a low rate to save
                                       // ressources of your computing system)
  ros::Duration obstacle_topic_update_delay_;  // the inverse of the update rate
  ros::Time last_update_time_obstacles_;       // time of last update from the obstacle topic

  tf::TransformListener tf_listener_;

  ros::ServiceServer map_points_accessibility_check_server_;  // server handling requests for checking the accessibility
                                                              // of a set of points
  ros::ServiceServer map_perimeter_accessibility_check_server_;  // server handling requests for checking the
                                                                 // accessibility of any point on the perimeter of a
                                                                 // given position
  ros::ServiceServer map_polygon_accessibility_check_server_;    // server handling requests for checking the
                                                                 // accessibility of any point around a given polygon
                                                                 // (obeying the safety margin around the polygon)

  // maps
  cv::Mat original_map_;
  cv::Mat inflated_original_map_;  // contains only the inflated static obstacles
  cv::Mat inflated_map_;           // contains inflated static and dynamic obstacles

  boost::mutex mutex_inflated_map_;  // mutex for access on inflated_map

  // map properties
  double map_resolution_;          // in [m/cell]
  double inverse_map_resolution_;  // in [cell/m]
  cv::Point2d map_origin_;         // in [m,m,rad]
  std::string map_link_name_;

  // robot
  double robot_radius_;                          // in [m]
  std::string robot_base_link_name_;
  bool approach_path_accessibility_check_;  // if true, the path to a goal position must be accessible as well
};

#endif  // MAP_ACCESSIBILITY_ANALYSIS_H
