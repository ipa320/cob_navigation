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

  ros::Subscriber joystick_velocity_sub_, obstacles_sub_, robot_footprint_sub_;
  
  // Constructor
  JoystickFilterClass(std::string name)
  {
    m_mutex = PTHREAD_MUTEX_INITIALIZER;
  	
    // implementation of topics to publish (command for base and list of relevant obstacles)
//  	topic_pub_command_ = nh_.advertise<geometry_msgs::Twist>("command", 1);
//  	topic_pub_relevant_obstacles_ = nh_.advertise<nav_msgs::GridCells>("relevant_obstacles", 1);
  	
    // subscribe to twist-movement of teleop 
  	joystick_velocity_sub_ = nh_.subscribe<geometry_msgs::Twist>("teleop_twist", 1, boost::bind(&JoystickFilterClass::joystickVelocityCB, this, _1));
  	// subscribe to the costmap to receive inflated cells
  	obstacles_sub_ = nh_.subscribe<nav_msgs::GridCells>("obstacles", 1, boost::bind(&JoystickFilterClass::obstaclesCB, this, _1));
  	// subscribe to the robot footprint published by the costmap
  	robot_footprint_sub_ = nh_.subscribe<geometry_msgs::PolygonStamped>("robot_footprint", 1, boost::bind(&JoystickFilterClass::robotFootprintCB, this, _1));
  }
  
  // joystick_velocityCB reads twist command from joystick
  void joystickVelocityCB(const geometry_msgs::Twist::ConstPtr &twist){
  	ROS_INFO("joystick_velocityCB called");
  }
  
  // obstaclesCB reads obstacles from costmap
  void obstaclesCB(const nav_msgs::GridCells::ConstPtr &obstacles){
  	ROS_INFO("obstaclesCB called");
  }
  
  // robotFootprintCB reads footprint from costmap
  void robotFootprintCB(const geometry_msgs::PolygonStamped::ConstPtr &robot_footprint){
    pthread_mutex_lock(&m_mutex);
    ROS_INFO("robotFootprintCB called");

    robot_footprint_ = robot_footprint->polygon.points;
    
    pthread_mutex_unlock(&m_mutex);
  }

  // Destructor
  ~JoystickFilterClass() 
  {}
  
  private:
  pthread_mutex_t m_mutex;

  //obstacle avoidence
  std::vector<geometry_msgs::Point32> robot_footprint_;
  double footprint_left_, footprint_right_, footprint_front_, footprint_rear_;

  //helper functions:

}; //NodeClass

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

