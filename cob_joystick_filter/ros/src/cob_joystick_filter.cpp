/****************************************************************
 *
 * Copyright (c) 2012
 *
 * Fraunhofer Institute for Manufacturing Engineering  
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_navigation
 * ROS package name: cob_joystick_filter
 *  							
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *  		
 * Author: Matthias Gruhler, email:Matthias.Gruhler@ipa.fhg.de
 *
 * Date of creation: February 2012
 * ToDo:
 *    - calculate Distance (of footprint) to obstacle
 *    - handle possible collisions (1. stop robot completely, 2. block only component of velocity that would lead to collision)
 *    - tests
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *  	 notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *  	 notice, this list of conditions and the following disclaimer in the
 *  	 documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Fraunhofer Institute for Manufacturing 
 *  	 Engineering and Automation (IPA) nor the names of its
 *  	 contributors may be used to endorse or promote products derived from
 *  	 this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public 
 * License LGPL along with this program. 
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <ros/ros.h>
#include <XmlRpc.h>

#include <pthread.h>

// ROS message includes
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <nav_msgs/GridCells.h>

// Listener includes
#include <tf/transform_listener.h>

//###############################
//#### joystick filter class ####
class JoystickFilterClass
{
  public:
  // create a handle for this node, initialize node
  ros::NodeHandle nh_;
  	
  // declaration of topics
//  ros::Publisher topic_pub_command_;
//  ros::Publisher topic_pub_relevant_obstacles_;

  ros::Subscriber joystick_velocity_sub_, obstacles_sub_;
  
  // Constructor
  JoystickFilterClass(std::string name)
  {
    m_mutex = PTHREAD_MUTEX_INITIALIZER;

    // node handle to get footprint from parameter server
    std::string costmap_name_ = "/local_costmap_node/costmap";
    ros::NodeHandle local_costmap_nh_(costmap_name_); 	

// implementation of topics to publish (command for base and list of relevant obstacles)
//  	topic_pub_command_ = nh_.advertise<geometry_msgs::Twist>("command", 1);
//  	topic_pub_relevant_obstacles_ = nh_.advertise<nav_msgs::GridCells>("relevant_obstacles", 1);
  	
    // subscribe to twist-movement of teleop 
  	joystick_velocity_sub_ = nh_.subscribe<geometry_msgs::Twist>("teleop_twist", 1, boost::bind(&JoystickFilterClass::joystickVelocityCB, this, _1));
  	// subscribe to the costmap to receive inflated cells
  	obstacles_sub_ = nh_.subscribe<nav_msgs::GridCells>("obstacles", 1, boost::bind(&JoystickFilterClass::obstaclesCB, this, _1));
		
		// read parameters from parameter server
		if(!local_costmap_nh_.hasParam(costmap_name_+"/global_frame")) ROS_WARN("Used default parameter for global_frame");
		local_costmap_nh_.param(costmap_name_+"/global_frame", global_frame_, std::string("/map"));
		
		if(!local_costmap_nh_.hasParam(costmap_name_+"/robot_base_frame")) ROS_WARN("Used default parameter for robot_frame");
		local_costmap_nh_.param(costmap_name_+"/robot_base_frame", robot_frame_, std::string("/base_link"));
		
    if(!nh_.hasParam(name+"/influence_radius")) ROS_WARN("Used default parameter for influence_radius");
		nh_.param(name+"/influence_radius", influence_radius_, 1.5);
		closest_obstacle_dist_ = influence_radius_;
		
		if(!nh_.hasParam(name+"/stop_threshold")) ROS_WARN("Used default parameter for stop_threshold");
		nh_.param(name+"/stop_threshold", stop_threshold_, 0.10);
		
		if(!nh_.hasParam(name+"/obstacle_damping_dist")) ROS_WARN("Used default parameter for obstacle_damping_dist");
		nh_.param(name+"/obstacle_damping_dist", obstacle_damping_dist_, 50.0);
		if(obstacle_damping_dist_ < stop_threshold_) {
			ROS_WARN("obstacle_damping_dist < stop_threshold => robot will stop without decceleration!");
		}
		
		if(!nh_.hasParam(name+"/use_circumscribed_threshold")) ROS_WARN("Used default parameter for use_circumscribed_threshold_");
		nh_.param(name+"/use_circumscribed_threshold", use_circumscribed_threshold_, 0.20);
		
    //load the robot footprint from the parameter server if its available in the local costmap namespace
		robot_footprint_ = loadRobotFootprint(local_costmap_nh_);
		if(robot_footprint_.size() > 4) 
			ROS_WARN("You have set more than 4 points as robot_footprint, cob_joystick_filter can deal only with rectangular footprints so far!");


		//we need to make sure that the transform between the robot base frame and the global frame is available
		ros::Time last_error = ros::Time::now();
		std::string tf_error;
		while(!tf_listener_.waitForTransform(global_frame_, robot_frame_, ros::Time(), ros::Duration(0.1), ros::Duration(0.01), &tf_error)) {
			ros::spinOnce();
			if(last_error + ros::Duration(5.0) < ros::Time::now()){
				ROS_WARN("Waiting on transform from %s to %s to become available before running cob_joystick_filter, tf error: %s", 
				robot_frame_.c_str(), global_frame_.c_str(), tf_error.c_str());
				last_error = ros::Time::now();
			}
		}
  } 

  // joystick_velocityCB reads twist command from joystick
  void joystickVelocityCB(const geometry_msgs::Twist::ConstPtr &twist){
    pthread_mutex_lock(&m_mutex);
    
    robot_twist_linear_ = twist->linear;
    robot_twist_angular_ = twist->angular;

    ROS_DEBUG("Received Twist linear: x=%f , y=%f ; angular: z=%f", robot_twist_linear_.x, robot_twist_linear_.y, robot_twist_angular_.z);

    pthread_mutex_unlock(&m_mutex);
  }
  
  // obstaclesCB reads obstacles from costmap
  void obstaclesCB(const nav_msgs::GridCells::ConstPtr &obstacles){
		pthread_mutex_lock(&m_mutex);
	
		if(obstacles->cells.size()!=0) costmap_received_ = true;
		last_costmap_received_ = * obstacles;
		
		if(stop_threshold_ < obstacles->cell_width / 2.0f || stop_threshold_ < obstacles->cell_height / 2.0f)
			ROS_WARN("You specified a stop_threshold that is smaller than resolution of received costmap!");
		
		pthread_mutex_unlock(&m_mutex);
  }
  
  // Destructor
  ~JoystickFilterClass() 
  {}
  
  private:
  pthread_mutex_t m_mutex;

  //tf transform
	tf::TransformListener tf_listener_;
	std::string global_frame_, robot_frame_;

  // velocity
  geometry_msgs::Vector3 robot_twist_linear_, robot_twist_angular_;
  
  //obstacle avoidence
  std::vector<geometry_msgs::Point> robot_footprint_;
  double footprint_left_, footprint_right_, footprint_front_, footprint_rear_;
  bool costmap_received_;
  nav_msgs::GridCells last_costmap_received_;
  double influence_radius_, stop_threshold_, obstacle_damping_dist_, use_circumscribed_threshold_;
  double closest_obstacle_dist_;

  //helper functions:
  std::vector<geometry_msgs::Point> loadRobotFootprint(ros::NodeHandle node);
	double sign(double x);

}; //JoystickFilterClass


// load robot footprint from costmap_2d_ros to keep same footprint format
std::vector<geometry_msgs::Point> JoystickFilterClass::loadRobotFootprint(ros::NodeHandle node){
	std::vector<geometry_msgs::Point> footprint;
	geometry_msgs::Point pt;
	double padding;

	std::string padding_param, footprint_param;
	if(!node.searchParam("footprint_padding", padding_param)) {
		padding = 0.01;
	} else
		node.param(padding_param, padding, 0.01);

	//grab the footprint from the parameter server if possible
	XmlRpc::XmlRpcValue footprint_list;
	if(node.searchParam("footprint", footprint_param)){
		node.getParam(footprint_param, footprint_list);
		//make sure we have a list of lists
		if(!(footprint_list.getType() == XmlRpc::XmlRpcValue::TypeArray && footprint_list.size() > 2)){
		ROS_FATAL("The footprint must be specified as list of lists on the parameter server with at least 3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
		throw std::runtime_error("The footprint must be specified as list of lists on the parameter server with at least 3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
		}
		for(int i = 0; i < footprint_list.size(); ++i){
		//make sure we have a list of lists of size 2
		XmlRpc::XmlRpcValue point = footprint_list[i];
		if(!(point.getType() == XmlRpc::XmlRpcValue::TypeArray && point.size() == 2)){
			ROS_FATAL("The footprint must be specified as list of lists on the parameter server eg: [[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
			throw std::runtime_error("The footprint must be specified as list of lists on the parameter server eg: [[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
		}

		//make sure that the value we're looking at is either a double or an int
		if(!(point[0].getType() == XmlRpc::XmlRpcValue::TypeInt || point[0].getType() == XmlRpc::XmlRpcValue::TypeDouble)){
			ROS_FATAL("Values in the footprint specification must be numbers");
			throw std::runtime_error("Values in the footprint specification must be numbers");
		}
		pt.x = point[0].getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(point[0]) : (double)(point[0]);
		pt.x += sign(pt.x) * padding;

		//make sure that the value we're looking at is either a double or an int
		if(!(point[1].getType() == XmlRpc::XmlRpcValue::TypeInt || point[1].getType() == XmlRpc::XmlRpcValue::TypeDouble)){
			ROS_FATAL("Values in the footprint specification must be numbers");
			throw std::runtime_error("Values in the footprint specification must be numbers");
		}
		pt.y = point[1].getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(point[1]) : (double)(point[1]);
		pt.y += sign(pt.y) * padding;

		footprint.push_back(pt);
		}
	}
	
	footprint_right_ = 0.0f; footprint_left_ = 0.0f; footprint_front_ = 0.0f; footprint_rear_ = 0.0f;
	//extract rectangular borders for simplifying:
	for(unsigned int i=0; i<footprint.size(); i++) {
		if(footprint[i].x > footprint_front_) footprint_front_ = footprint[i].x;
		if(footprint[i].x < footprint_rear_) footprint_rear_ = footprint[i].x;
		if(footprint[i].y > footprint_left_) footprint_left_ = footprint[i].y;
		if(footprint[i].y < footprint_right_) footprint_right_ = footprint[i].y;
	}
	ROS_DEBUG("Extracted rectangular footprint for cob_joystick_filter: Front: %f, Rear %f, Left: %f, Right %f", footprint_front_, footprint_rear_, footprint_left_, footprint_right_);

	return footprint;
}
	
double JoystickFilterClass::sign(double x) {
	if(x >= 0.0f) return 1.0f;
	else return -1.0f;
}


//#######################
//#### main programm ####
int main(int argc, char** argv)
{
  // initialize ROS, spezify name of node
  ros::init(argc, argv, "cob_joystick");
  
  // create nodeClass
  JoystickFilterClass joystickFilterClass("joystick_filter");

  ros::spin();

  return 0;
}

