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

#include <ros/ros.h>

//#include <cob_3d_mapping_common/polygon.h>
//#include <cob_3d_mapping_common/ros_msg_conversions.h>
//#include <tf/tf.h>

// services - here you have to include the header file with exactly the same name as your message in the /srv folder
// (the Message.h is automatically generated from your Message.srv file during compilation)
#include <cob_map_accessibility_analysis/CheckPointAccessibility.h>
#include <cob_map_accessibility_analysis/CheckPerimeterAccessibility.h>
#include <cob_3d_mapping_msgs/GetApproachPoseForPolygon.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_point_accessibility_check_client");

  ros::NodeHandle n;

  std::string points_service_name = "/map_accessibility_analysis/map_points_accessibility_check";
  std::string perimeter_service_name = "/map_accessibility_analysis/map_perimeter_accessibility_check";
  std::string polygon_service_name = "/map_accessibility_analysis/map_polygon_accessibility_check";

  // here we wait until the service is available; please use the same service name as the one in the server; you may
  // define a timeout if the service does not show up
  ROS_INFO("Waiting for service server to become available...");
  bool serviceAvailable = ros::service::waitForService(points_service_name, 5000);
  serviceAvailable &= ros::service::waitForService(perimeter_service_name, 5000);
  serviceAvailable &= ros::service::waitForService(polygon_service_name, 5000);

  // only proceed if the service is available
  if (serviceAvailable == false)
  {
    ROS_ERROR("The services could not be found.\n");
    return -1;
  }
  ROS_INFO("The service servers are advertised.\n");

  // Call to point accessibility service
  cob_map_accessibility_analysis::CheckPointAccessibility::Request req_points;
  cob_map_accessibility_analysis::CheckPointAccessibility::Response res_points;

  geometry_msgs::Pose2D point;
  point.x = 1.;
  point.y = 0;
  req_points.points_to_check.push_back(point);
  point.x = -2.;
  point.y = 0;
  req_points.points_to_check.push_back(point);

  // this calls the service server to process our request message and put the result into the response message
  // this call is blocking, i.e. this program will not proceed until the service server sends the response
  bool success = ros::service::call(points_service_name, req_points, res_points);

  if (success == true)
  {
    ROS_INFO("Points request successful, results:\n");
    for (unsigned int i = 0; i < res_points.accessibility_flags.size(); ++i)
      ROS_INFO(" - (xy)=(%f, %f), accessible=%d\n",
               req_points.points_to_check[i].x,
               req_points.points_to_check[i].y,
               res_points.accessibility_flags[i]);
  }
  else
    ROS_WARN("The service call for points was not successful.\n");

  // Call to perimeter accessibility service
  cob_map_accessibility_analysis::CheckPerimeterAccessibility::Request req_perimeter;
  cob_map_accessibility_analysis::CheckPerimeterAccessibility::Response res_perimeter;

  req_perimeter.center.x = 2.5;
  req_perimeter.center.y = 2.25;
  req_perimeter.center.theta = 0.;
  req_perimeter.radius = 0.45;
  req_perimeter.rotational_sampling_step = 10. / 180. * M_PI;

  // this calls the service server to process our request message and put the result into the response message
  // this call is blocking, i.e. this program will not proceed until the service server sends the response
  success = ros::service::call(perimeter_service_name, req_perimeter, res_perimeter);

  if (success == true)
  {
    ROS_INFO("Accessible points on perimeter:\n");
    for (unsigned int i = 0; i < res_perimeter.accessible_poses_on_perimeter.size(); ++i)
      ROS_INFO(" - (xyt)=(%f, %f, %f)\n",
               res_perimeter.accessible_poses_on_perimeter[i].x,
               res_perimeter.accessible_poses_on_perimeter[i].y,
               res_perimeter.accessible_poses_on_perimeter[i].theta);
  }
  else
    ROS_WARN("The service call for perimeter points was not successful.\n");

  //// ===== example call to polygon accessibility service =====
  // cob_3d_mapping_msgs::GetApproachPoseForPolygonRequest req_polygon;
  // cob_3d_mapping_msgs::GetApproachPoseForPolygonResponse res_polygon;

  // cob_3d_mapping::Polygon p1;
  // Eigen::Vector2f v;
  // std::vector<Eigen::Vector2f> vv;
  // p1.id_ = 1;
  // p1.normal_ << 0.0,0.0,1.0;
  // p1.d_ = -1;
  // v << 1,-2;//,1;
  // vv.push_back(v);
  // v << 1,-3;//,1;
  // vv.push_back(v);
  // v << 2,-3;//,1;
  // vv.push_back(v);
  // v << 2,-2;//,1;
  // vv.push_back(v);
  // p1.contours_.push_back(vv);
  // p1.holes_.push_back(false);
  // cob_3d_mapping_msgs::Shape p_msg;
  // toROSMsg(p1, p_msg);
  // req_polygon.polygon = p_msg;

  //// this calls the service server to process our request message and put the result into the response message
  //// this call is blocking, i.e. this program will not proceed until the service server sends the response
  // success = ros::service::call(polygon_service_name, req_polygon, res_polygon);

  // if (success == true)
  //{
  // printf("Accessible points around the polygon:\n");
  // for (unsigned int i=0; i<res_polygon.approach_poses.poses.size(); i++)
  //{
  // tf::Quaternion q;
  // tf::quaternionMsgToTF(res_polygon.approach_poses.poses[i].orientation, q);
  // std::cout << "i=" << i << "  x=" << res_polygon.approach_poses.poses[i].position.x << "  y=" <<
  // res_polygon.approach_poses.poses[i].position.y << "  ang=" << tf::getYaw(q) << std::endl;
  //}
  //}
  // else
  // std::cout << "The service call for polygon points was not successful.\n" << std::endl;

  return 0;
}
