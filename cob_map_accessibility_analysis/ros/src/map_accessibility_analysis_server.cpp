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
 

#include <cob_map_accessibility_analysis/map_accessibility_analysis_server.h>
#include <pcl_ros/point_cloud.h>

//#define __DEBUG_DISPLAYS__

MapAccessibilityAnalysisServer::MapAccessibilityAnalysisServer(ros::NodeHandle nh) : node_handle_(nh)
{
  // read in parameters
  ROS_INFO("\n--------------------------------------\nMap Accessibility Analysis "
           "Parameters:\n--------------------------------------");
  node_handle_.param("approach_path_accessibility_check", approach_path_accessibility_check_, false);	// this setting is overwritten temporarily on receiving the CheckPointAccessibility.srv message
  ROS_INFO_STREAM("approach_path_accessibility_check = " << approach_path_accessibility_check_);
  node_handle_.param<std::string>("map_link_name", map_link_name_, "/map");
  ROS_INFO_STREAM("map_link_name = " << map_link_name_);
  node_handle_.param<std::string>("robot_base_link_name", robot_base_link_name_, "/base_link");
  ROS_INFO_STREAM("robot_base_link_name_ = " << robot_base_link_name_);
  node_handle_.param("obstacle_topic_update_rate", obstacle_topic_update_rate_, 5.0);
  ROS_INFO_STREAM("obstacle_topic_update_rate = " << obstacle_topic_update_rate_);
  obstacle_topic_update_delay_ = ros::Duration(1.0 / obstacle_topic_update_rate_);
  last_update_time_obstacles_ = ros::Time::now();
  node_handle_.param("publish_inflated_map", publish_inflated_map_, false);
  ROS_INFO_STREAM("publish_inflated_map = " << publish_inflated_map_);
  robot_radius_ = 0.;
  if (node_handle_.hasParam("/local_costmap_node/costmap/footprint"))
  {
    // compute robot radius from footprint
    XmlRpc::XmlRpcValue footprint_list;
    node_handle_.getParam("/local_costmap_node/costmap/footprint", footprint_list);
    std::vector<geometry_msgs::Point> footprint = loadRobotFootprint(footprint_list);	// polygonal footprint of the robot in [m]
    std::stringstream footprint_string;
    footprint_string << "footprint = [ ";
    for (unsigned int i = 0; i < footprint.size(); ++i)
    {
      robot_radius_ = std::max<double>(robot_radius_, sqrt(footprint[i].x * footprint[i].x + footprint[i].y * footprint[i].y));
      footprint_string << "[" << footprint[i].x << ", " << footprint[i].y << "] ";
    }
    footprint_string << "]";
    ROS_INFO("%s", footprint_string.str().c_str());
  }
  if (robot_radius_ == 0.0)
  {
    // if no footprint is set take the robot radius
    node_handle_.param("/local_costmap_node/costmap/robot_radius", robot_radius_, 0.8);
  }
  // hack:
  robot_radius_ = 0.35;
  ROS_INFO_STREAM("robot_radius = " << robot_radius_);

  // receive ground floor map once
  mapInit(node_handle_);

  // then set up dynamic obstacle callbacks
  // inflationInit(node_handle_);
  dynamicObstaclesInit(node_handle_);

  // advertise inflated map image
  it_ = new image_transport::ImageTransport(node_handle_);
  inflated_map_image_pub_ = it_->advertise("inflated_map", 1);

  // advertise services
  map_points_accessibility_check_server_ = node_handle_.advertiseService(
      "map_points_accessibility_check", &MapAccessibilityAnalysisServer::checkPose2DArrayCallback, this);
  map_perimeter_accessibility_check_server_ = node_handle_.advertiseService(
      "map_perimeter_accessibility_check", &MapAccessibilityAnalysisServer::checkPerimeterCallback, this);
  map_polygon_accessibility_check_server_ = node_handle_.advertiseService(
      "map_polygon_accessibility_check", &MapAccessibilityAnalysisServer::checkPolygonCallback, this);

  ROS_INFO("MapPointAccessibilityCheck initialized.");
}

MapAccessibilityAnalysisServer::~MapAccessibilityAnalysisServer()
{
  if (it_ != 0)
    delete it_;
}

std::vector<geometry_msgs::Point> MapAccessibilityAnalysisServer::loadRobotFootprint(XmlRpc::XmlRpcValue& footprint_list)
{
  std::vector<geometry_msgs::Point> footprint;
  geometry_msgs::Point pt;
  double scale_factor = 1.0;

  // grab the footprint from the provided parameter
  std::string footprint_string;
  std::vector<std::string> footstring_list;
  if (footprint_list.getType() == XmlRpc::XmlRpcValue::TypeString)
  {
    footprint_string = std::string(footprint_list);

    // if there's just an empty footprint up there, return
    if (footprint_string == "[]" || footprint_string == "")
      return footprint;

    boost::erase_all(footprint_string, " ");

    boost::char_separator<char> sep("[]");
    boost::tokenizer<boost::char_separator<char> > tokens(footprint_string, sep);
    footstring_list = std::vector<std::string>(tokens.begin(), tokens.end());
  }
  // make sure we have a list of lists
  if (!(footprint_list.getType() == XmlRpc::XmlRpcValue::TypeArray && footprint_list.size() > 2) &&
      !(footprint_list.getType() == XmlRpc::XmlRpcValue::TypeString && footstring_list.size() > 5))
  {
    ROS_FATAL("The footprint must be specified as list of lists on the parameter server, but it was specified as %s",
              std::string(footprint_list).c_str());
    throw std::runtime_error("The footprint must be specified as list of lists on the parameter server with at least 3 "
                             "points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
  }

  if (footprint_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    for (int i = 0; i < footprint_list.size(); ++i)
    {
      // make sure we have a list of lists of size 2
      XmlRpc::XmlRpcValue point = footprint_list[i];
      if (!(point.getType() == XmlRpc::XmlRpcValue::TypeArray && point.size() == 2))
      {
        ROS_FATAL("The footprint must be specified as list of lists on the parameter server eg: [[x1, y1], [x2, y2], "
                  "..., [xn, yn]], but this spec is not of that form");
        throw std::runtime_error("The footprint must be specified as list of lists on the parameter server eg: [[x1, "
                                 "y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
      }

      // make sure that the value we're looking at is either a double or an int
      if (!(point[0].getType() == XmlRpc::XmlRpcValue::TypeInt ||
            point[0].getType() == XmlRpc::XmlRpcValue::TypeDouble))
      {
        ROS_FATAL("Values in the footprint specification must be numbers");
        throw std::runtime_error("Values in the footprint specification must be numbers");
      }
      pt.x = point[0].getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(point[0]) : (double)(point[0]);
      pt.x *= scale_factor;

      // make sure that the value we're looking at is either a double or an int
      if (!(point[1].getType() == XmlRpc::XmlRpcValue::TypeInt ||
            point[1].getType() == XmlRpc::XmlRpcValue::TypeDouble))
      {
        ROS_FATAL("Values in the footprint specification must be numbers");
        throw std::runtime_error("Values in the footprint specification must be numbers");
      }
      pt.y = point[1].getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(point[1]) : (double)(point[1]);
      pt.y *= scale_factor;

      footprint.push_back(pt);

      //				node.deleteParam(footprint_param);
      //				std::ostringstream oss;
      //				bool first = true;
      //				BOOST_FOREACH(geometry_msgs::Point p, footprint)
      //							{
      //								if (first)
      //								{
      //									oss << "[[" << p.x << "," << p.y << "]";
      //									first = false;
      //								}
      //								else
      //								{
      //									oss << ",[" << p.x << "," << p.y << "]";
      //								}
      //							}
      //				oss << "]";
      //				node.setParam(footprint_param, oss.str().c_str());
      //				node.setParam("footprint", oss.str().c_str());
    }
  }
  else if (footprint_list.getType() == XmlRpc::XmlRpcValue::TypeString)
  {
    std::vector<geometry_msgs::Point> footprint_spec;
    bool valid_foot = true;
    BOOST_FOREACH (std::string t, footstring_list)
    {
      if (t != ",")
      {
        boost::erase_all(t, " ");
        boost::char_separator<char> pt_sep(",");
        boost::tokenizer<boost::char_separator<char> > pt_tokens(t, pt_sep);
        std::vector<std::string> point(pt_tokens.begin(), pt_tokens.end());

        if (point.size() != 2)
        {
          ROS_WARN("Each point must have exactly 2 coordinates");
          valid_foot = false;
          break;
        }

        std::vector<double> tmp_pt;
        BOOST_FOREACH (std::string p, point)
        {
          std::istringstream iss(p);
          double temp;
          if (iss >> temp)
          {
            tmp_pt.push_back(temp);
          }
          else
          {
            ROS_WARN("Each coordinate must convert to a double.");
            valid_foot = false;
            break;
          }
        }
        if (!valid_foot)
          break;

        geometry_msgs::Point pt;
        pt.x = tmp_pt[0];
        pt.y = tmp_pt[1];

        footprint_spec.push_back(pt);
      }
    }
    if (valid_foot)
    {
      footprint = footprint_spec;
      // node.setParam("footprint", footprint_string);
    }
    else
    {
      ROS_FATAL("This footprint is not valid it must be specified as a list of lists with at least 3 points, you "
                "specified %s",
                footprint_string.c_str());
      throw std::runtime_error("The footprint must be specified as list of lists on the parameter server with at least "
                               "3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
    }
  }

  return footprint;
}

void MapAccessibilityAnalysisServer::mapInit(ros::NodeHandle& nh)
{
  map_data_recieved_ = false;
  map_msg_sub_ = nh.subscribe<nav_msgs::OccupancyGrid>("map", 1, &MapAccessibilityAnalysisServer::mapDataCallback, this);
  ROS_INFO("MapPointAccessibilityCheck: Waiting to receive map...");
  while (map_data_recieved_ == false)
    ros::spinOnce();
  ROS_INFO("MapPointAccessibilityCheck: Map received.");
}

void MapAccessibilityAnalysisServer::inflationInit(ros::NodeHandle& nh)
{
  obstacles_sub_.subscribe(nh, "obstacles", 1);
  inflated_obstacles_sub_.subscribe(nh, "inflated_obstacles", 1);

  inflated_obstacles_sub_sync_ = boost::shared_ptr<message_filters::Synchronizer<InflatedObstaclesSyncPolicy> >(
      new message_filters::Synchronizer<InflatedObstaclesSyncPolicy>(InflatedObstaclesSyncPolicy(5)));
  inflated_obstacles_sub_sync_->connectInput(obstacles_sub_, inflated_obstacles_sub_);
  inflated_obstacles_sub_sync_->registerCallback(
      boost::bind(&MapAccessibilityAnalysisServer::inflatedObstacleDataCallback, this, _1, _2));
}

void MapAccessibilityAnalysisServer::dynamicObstaclesInit(ros::NodeHandle& nh)
{
  obstacles_sub_.subscribe(nh, "obstacles", 1);
  obstacles_sub_.registerCallback(boost::bind(&MapAccessibilityAnalysisServer::obstacleDataCallback, this, _1));
}

void MapAccessibilityAnalysisServer::mapDataCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg_data)
{
  map_resolution_ = map_msg_data->info.resolution;
  inverse_map_resolution_ = 1. / map_resolution_;
  map_origin_ = cv::Point2d(map_msg_data->info.origin.position.x, map_msg_data->info.origin.position.y);
  ROS_INFO_STREAM("map resolution: " << map_msg_data->info.resolution);

  // create original map
  original_map_ = 255 * cv::Mat::ones(map_msg_data->info.height, map_msg_data->info.width, CV_8UC1);
  for (unsigned int v = 0, i = 0; v < map_msg_data->info.height; v++)
  {
    for (unsigned int u = 0; u < map_msg_data->info.width; u++, i++)
    {
      if (map_msg_data->data[i] != 0)
        original_map_.at<unsigned char>(v, u) = 0;
    }
  }

  // compute inflated static map
  ROS_INFO_STREAM("inflation thickness: " << cvRound(robot_radius_ * inverse_map_resolution_));
  inflateMap(original_map_, inflated_original_map_, cvRound(robot_radius_ * inverse_map_resolution_));
  if (inflated_map_.empty() == true)
    inflated_map_ = inflated_original_map_;  // initial setup (if no obstacle msgs were received yet)

  map_data_recieved_ = true;
  map_msg_sub_.shutdown();
}

void MapAccessibilityAnalysisServer::inflatedObstacleDataCallback(const nav_msgs::GridCells::ConstPtr& obstacles_data,
                                                            const nav_msgs::GridCells::ConstPtr& inflated_obstacles_data)
{
  if (obstacle_topic_update_rate_ != 0.0 &&
      (ros::Time::now() - last_update_time_obstacles_) > obstacle_topic_update_delay_)
  {
    boost::mutex::scoped_lock lock(mutex_inflated_map_);

    inflated_map_ = inflated_original_map_.clone();
    for (unsigned int i = 0; i < obstacles_data->cells.size(); ++i)
      inflated_map_.at<uchar>((obstacles_data->cells[i].y - map_origin_.y) * inverse_map_resolution_,
                              (obstacles_data->cells[i].x - map_origin_.x) * inverse_map_resolution_) = 0;

    for (unsigned int i = 0; i < inflated_obstacles_data->cells.size(); i++)
      inflated_map_.at<uchar>((inflated_obstacles_data->cells[i].y - map_origin_.y) * inverse_map_resolution_,
                              (inflated_obstacles_data->cells[i].x - map_origin_.x) * inverse_map_resolution_) = 0;

    last_update_time_obstacles_ = ros::Time::now();
  }
}

void MapAccessibilityAnalysisServer::obstacleDataCallback(const nav_msgs::GridCells::ConstPtr& obstacles_data)
{
  if (obstacle_topic_update_rate_ != 0.0 &&
      (ros::Time::now() - last_update_time_obstacles_) > obstacle_topic_update_delay_)
  {
    double radius = cvRound(robot_radius_ * inverse_map_resolution_);

    boost::mutex::scoped_lock lock(mutex_inflated_map_);

    inflated_map_ = inflated_original_map_.clone();
    for (unsigned int i = 0; i < obstacles_data->cells.size(); ++i)
    {
      const int u = (obstacles_data->cells[i].x - map_origin_.x) * inverse_map_resolution_;
      const int v = (obstacles_data->cells[i].y - map_origin_.y) * inverse_map_resolution_;
      inflated_map_.at<uchar>(v, u) = 0;
      cv::circle(inflated_map_, cv::Point(u, v), radius, cv::Scalar(0), -1);
    }

    if (publish_inflated_map_ == true)
    {
      // publish image
      cv_bridge::CvImage cv_ptr;
      cv_ptr.image = inflated_map_;
      cv_ptr.encoding = "mono8";
      inflated_map_image_pub_.publish(cv_ptr.toImageMsg());
    }

    last_update_time_obstacles_ = ros::Time::now();
  }
}

bool MapAccessibilityAnalysisServer::checkPose2DArrayCallback(
    cob_map_accessibility_analysis::CheckPointAccessibility::Request& req,
    cob_map_accessibility_analysis::CheckPointAccessibility::Response& res)
{
  ROS_INFO("Received request to check accessibility of %u points.", (unsigned int)req.points_to_check.size());

  // determine robot pose if approach path analysis activated
  cv::Point robot_location(0, 0);
  if (approach_path_accessibility_check_ == true)
    robot_location = getRobotLocationInPixelCoordinates();

  // prepare request and response data in appropriate format
  std::vector<cv::Point> points_to_check(req.points_to_check.size());
  for (size_t i=0; i<req.points_to_check.size(); ++i)
    points_to_check[i] = cv::Point(cvRound((req.points_to_check[i].x - map_origin_.x) * inverse_map_resolution_),
                                     cvRound((req.points_to_check[i].y - map_origin_.y) * inverse_map_resolution_));
  std::vector<bool> accessibility_flags(req.points_to_check.size(), false);

  // compute accessibility
  {
    boost::mutex::scoped_lock lock(mutex_inflated_map_);
    checkPoses(points_to_check, accessibility_flags, inflated_map_, req.approach_path_accessibility_check, robot_location);
  }

  // prepare return data
  res.accessibility_flags.resize(req.points_to_check.size());
  for (size_t i=0; i<accessibility_flags.size(); ++i)
    res.accessibility_flags[i] = accessibility_flags[i];

  return true;
}

bool MapAccessibilityAnalysisServer::checkPerimeterCallback(
    cob_map_accessibility_analysis::CheckPerimeterAccessibility::Request& req,
    cob_map_accessibility_analysis::CheckPerimeterAccessibility::Response& res)
{
  ROS_INFO("Received request to check accessibility of a circle with center (%f,%f), radius %f and sampling step %f.",
           req.center.x, req.center.y, req.radius, req.rotational_sampling_step);

  if (req.rotational_sampling_step == 0.0)
  {
    req.rotational_sampling_step = 10. / 180. * CV_PI;
    ROS_WARN("rotational_sampling_step was provided as 0.0 . Automatically changed it to %f.", req.rotational_sampling_step);
  }

  // determine robot pose if approach path analysis activated
  cv::Point robot_location(0, 0);
  if (approach_path_accessibility_check_ == true)
    robot_location = getRobotLocationInPixelCoordinates();

  // prepare request and response data in appropriate format
  Pose center((req.center.x-map_origin_.x)*inverse_map_resolution_, (req.center.y-map_origin_.y)*inverse_map_resolution_, req.center.theta);
  std::vector<Pose> accessible_poses_on_perimeter;

  // compute accessibility
  {
    boost::mutex::scoped_lock lock(mutex_inflated_map_);
    checkPerimeter(accessible_poses_on_perimeter, center, req.radius*inverse_map_resolution_, req.rotational_sampling_step, inflated_map_,
                   approach_path_accessibility_check_, robot_location);
  }

  // prepare return data
  for (size_t i=0; i<accessible_poses_on_perimeter.size(); ++i)
  {
    geometry_msgs::Pose2D pose;
    pose.x = accessible_poses_on_perimeter[i].x*map_resolution_ + map_origin_.x;
    pose.y = accessible_poses_on_perimeter[i].y*map_resolution_ + map_origin_.y;
    pose.theta = accessible_poses_on_perimeter[i].orientation;
    res.accessible_poses_on_perimeter.push_back(pose);
  }

  return true;
}

bool MapAccessibilityAnalysisServer::checkPolygonCallback(cob_3d_mapping_msgs::GetApproachPoseForPolygon::Request& req,
                                                    cob_3d_mapping_msgs::GetApproachPoseForPolygon::Response& res)
{
  ROS_INFO("Received request to check accessibility around a polygon.");

  // determine robot pose if approach path analysis activated
  cv::Point robot_location(0, 0);
  if (approach_path_accessibility_check_ == true)
    robot_location = getRobotLocationInPixelCoordinates();

  // copy contours
  std::vector<std::vector<cv::Point> > polygon_contours;
  for (unsigned int i = 0; i < req.polygon.points.size(); i++)
  {
    if (req.polygon.holes[i] == false)
    {
      pcl::PointCloud<pcl::PointXYZ> pc;
      pcl::fromROSMsg(req.polygon.points[i], pc);
      std::vector<cv::Point> p_vec(pc.size());
      for (unsigned int j = 0; j < pc.size(); j++)
      {
        p_vec[j] = convertFromMeterToPixelCoordinates<cv::Point>(Pose(pc.points[j].x, pc.points[j].y, 0), map_origin_, inverse_map_resolution_);
      }
      polygon_contours.push_back(p_vec);
    }
  }

  // compute inflated polygon
  cv::Mat polygon_expanded = 255 * cv::Mat::ones(original_map_.rows, original_map_.cols, original_map_.type());
  cv::drawContours(polygon_expanded, polygon_contours, -1, cv::Scalar(128), cv::FILLED);
  cv::erode(polygon_expanded, polygon_expanded, cv::Mat(), cv::Point(-1, -1), cvRound(robot_radius_ * inverse_map_resolution_));

  // combine inflated polygon with inflated map
  cv::Mat inflated_map;
  {
    boost::mutex::scoped_lock lock(mutex_inflated_map_);
    inflated_map = cv::min(polygon_expanded, inflated_map_);
  }
#ifdef __DEBUG_DISPLAYS__
  cv::imshow("inflated polygon map", inflated_map);
  cv::waitKey();
#endif

  // find the individual connected areas
  std::vector<std::vector<cv::Point> > area_contours;  // first index=contour index;  second index=point index within contour
  if (approach_path_accessibility_check_ == true)
  {
    cv::Mat inflated_map_copy = inflated_map.clone();
    cv::findContours(inflated_map_copy, area_contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
  }

// iterate through all white points and consider those as potential approach poses that have an expanded table pixel in
// their neighborhood
#ifdef __DEBUG_DISPLAYS__
  cv::Mat map_expanded_copy = inflated_map.clone();
  cv::drawContours(map_expanded_copy, area_contours, -1, cv::Scalar(128, 128, 128, 128), 2);
#endif
  for (int y = 1; y < inflated_map.rows - 1; y++)
  {
    for (int x = 1; x < inflated_map.cols - 1; x++)
    {
      if (inflated_map.at<uchar>(y, x) == 255)
      {
        bool close_to_polygon = false;
        for (int ky = -1; ky <= 1; ky++)
          for (int kx = -1; kx <= 1; kx++)
          {
            if (inflated_map.at<uchar>(y + ky, x + kx) == 128)
            {
              close_to_polygon = true;
              break;
            }
          }
        if (close_to_polygon == true)
        {
          // check if robot can approach this position
          if (approach_path_accessibility_check_ == false ||
              isApproachPositionAccessible(robot_location, cv::Point(x, y), area_contours) == true)
          {
            geometry_msgs::Pose pose;
            Pose pose_p(x, y, 0);
            Pose pose_m = convertFromPixelCoordinatesToMeter<Pose>(pose_p, map_origin_, map_resolution_);
            pose.position.x = pose_m.x;
            pose.position.y = pose_m.y;
            pose.position.z = 0;

            Pose closest_point_on_polygon;
            computeClosestPointOnPolygon(inflated_map, pose_p, closest_point_on_polygon);

            tf::quaternionTFToMsg(tf::createQuaternionFromYaw(atan2(closest_point_on_polygon.y - pose_p.y,
                                                                    closest_point_on_polygon.x - pose_p.x)),
                                  pose.orientation);
            // tf::quaternionTFToMsg(tf::createQuaternionFromYaw(atan2(-dy.at<float>(y,x),-dx.at<float>(y,x))),
            // pose.orientation);
            res.approach_poses.poses.push_back(pose);
          }

#ifdef __DEBUG_DISPLAYS__
          // display found contours
          cv::circle(map_expanded_copy, robot_location, 3, cv::Scalar(200, 200, 200, 200), -1);
          cv::circle(map_expanded_copy, cv::Point(x, y), 3, cv::Scalar(200, 200, 200, 200), -1);
//					ROS_DEBUG_STREAM(" x=" << x << "  y=" << y);
#endif
        }
      }
    }
  }
#ifdef __DEBUG_DISPLAYS__
  cv::imshow("contour areas", map_expanded_copy);
  cv::waitKey();
#endif

  return true;
}

cv::Point MapAccessibilityAnalysisServer::getRobotLocationInPixelCoordinates()
{
  tf::StampedTransform transform;
  try
  {
    ros::Time request_time = ros::Time(0);
    tf_listener_.waitForTransform(map_link_name_, robot_base_link_name_, request_time, ros::Duration(10));
    tf_listener_.lookupTransform(map_link_name_, robot_base_link_name_, request_time, transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("[registration] : %s", ex.what());
    return cv::Point(0, 0);
  }
  tf::Vector3 pose = transform.getOrigin();
  cv::Point robot_location = convertFromMeterToPixelCoordinates<cv::Point>(Pose(pose.x(), pose.y(), 0), map_origin_, inverse_map_resolution_);

  return robot_location;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_accessibility_analysis_server");

  ros::NodeHandle nh;

  MapAccessibilityAnalysisServer map_accessibility_analysis(nh);

  ros::spin();

  return 0;
}
