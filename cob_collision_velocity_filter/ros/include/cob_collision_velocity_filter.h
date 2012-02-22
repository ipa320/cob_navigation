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
 * ROS package name: cob_collision_velocity_filter
 *  							
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *  		
 * Author: Matthias Gruhler, email:Matthias.Gruhler@ipa.fhg.de
 *
 * Date of creation: February 2012
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
#ifndef COB_COLLISION_VELOCITY_FILTER_H
#define COB_COLLISION_VELOCITY_FILTER_H

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

//#########################################
//#### collision velocity filter class ####
class CollisionVelocityFilter
{
  public:
    // Constructor
    CollisionVelocityFilter(std::string name);

    // Destructor
    ~CollisionVelocityFilter();

    // joystick_velocityCB reads twist command from joystick
    void joystickVelocityCB(const geometry_msgs::Twist::ConstPtr &twist);

    // obstaclesCB reads obstacles from costmap
    void obstaclesCB(const nav_msgs::GridCells::ConstPtr &obstacles);

    // create a handle for this node, initialize node
    ros::NodeHandle nh_;

    // declaration of topics
    ros::Publisher topic_pub_command_;
    ros::Publisher topic_pub_relevant_obstacles_;

    ros::Subscriber joystick_velocity_sub_, obstacles_sub_;

  private:
    //core functions
    void performControllerStep();
    void obstacleHandler();

    //helper functions:
    std::vector<geometry_msgs::Point> loadRobotFootprint(ros::NodeHandle node);
    double sign(double x);
    double getDistance2d(geometry_msgs::Point a, geometry_msgs::Point b);
    bool obstacleValid(double x_robot, double y_robot);
    void stopMovement();

    pthread_mutex_t m_mutex;

    //frames
    std::string global_frame_, robot_frame_;

    //velocity
    geometry_msgs::Vector3 robot_twist_linear_, robot_twist_angular_;
    double v_max_, vtheta_max_;  

    //obstacle avoidence
    std::vector<geometry_msgs::Point> robot_footprint_;
    double footprint_left_, footprint_right_, footprint_front_, footprint_rear_;
    bool costmap_received_;
    nav_msgs::GridCells last_costmap_received_, relevant_obstacles_;
    double influence_radius_, stop_threshold_, obstacle_damping_dist_, use_circumscribed_threshold_;
    double closest_obstacle_dist_;

}; //CollisionVelocityFilter

#endif
