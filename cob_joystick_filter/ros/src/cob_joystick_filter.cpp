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
 *    - implement Subscriber processing
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
    ros::NodeHandle local_costmap_nh("/local_costmap_node/costmap"); 	

// implementation of topics to publish (command for base and list of relevant obstacles)
//  	topic_pub_command_ = nh_.advertise<geometry_msgs::Twist>("command", 1);
//  	topic_pub_relevant_obstacles_ = nh_.advertise<nav_msgs::GridCells>("relevant_obstacles", 1);
  	
    // subscribe to twist-movement of teleop 
  	joystick_velocity_sub_ = nh_.subscribe<geometry_msgs::Twist>("teleop_twist", 1, boost::bind(&JoystickFilterClass::joystickVelocityCB, this, _1));
  	// subscribe to the costmap to receive inflated cells
  	obstacles_sub_ = nh_.subscribe<nav_msgs::GridCells>("obstacles", 1, boost::bind(&JoystickFilterClass::obstaclesCB, this, _1));
		
    //load the robot footprint from the parameter server if its available in the local costmap namespace
		robot_footprint_ = loadRobotFootprint(local_costmap_nh);
		if(robot_footprint_.size() > 4) 
			ROS_WARN("You have set more than 4 points as robot_footprint, cob_joystick_filter can deal only with rectangular footprints so far!");
  } 
  // joystick_velocityCB reads twist command from joystick
  void joystickVelocityCB(const geometry_msgs::Twist::ConstPtr &twist){
  	ROS_INFO("joystick_velocityCB called");
  }
  
  // obstaclesCB reads obstacles from costmap
  void obstaclesCB(const nav_msgs::GridCells::ConstPtr &obstacles){
  	ROS_INFO("obstaclesCB called");
  }
  
  // Destructor
  ~JoystickFilterClass() 
  {}
  
  private:
  pthread_mutex_t m_mutex;

  //obstacle avoidence
  std::vector<geometry_msgs::Point> robot_footprint_;
  double footprint_left_, footprint_right_, footprint_front_, footprint_rear_;

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
	ROS_INFO("Extracted rectangular footprint for cob_joystick_filter: Front: %f, Rear %f, Left: %f, Right %f", footprint_front_, footprint_rear_, footprint_left_, footprint_right_);

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

