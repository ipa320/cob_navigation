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
  ros::Publisher topic_pub_command_;
  ros::Publisher topic_pub_relevant_obstacles_;

  ros::Subscriber joystick_velocity_sub_, obstacles_sub_;
  
  // Constructor
  JoystickFilterClass(std::string name)
  {
    m_mutex = PTHREAD_MUTEX_INITIALIZER;

    // node handle to get footprint from parameter server
    std::string costmap_name_ = "/local_costmap_node/costmap";
    ros::NodeHandle local_costmap_nh_(costmap_name_); 	

    // implementation of topics to publish (command for base and list of relevant obstacles)
  	topic_pub_command_ = nh_.advertise<geometry_msgs::Twist>("command", 1);
    topic_pub_relevant_obstacles_ = nh_.advertise<nav_msgs::GridCells>("relevant_obstacles", 1);
  	
    // subscribe to twist-movement of teleop 
  	joystick_velocity_sub_ = nh_.subscribe<geometry_msgs::Twist>("teleop_twist", 1, boost::bind(&JoystickFilterClass::joystickVelocityCB, this, _1));
  	// subscribe to the costmap to receive inflated cells
  	obstacles_sub_ = nh_.subscribe<nav_msgs::GridCells>("obstacles", 1, boost::bind(&JoystickFilterClass::obstaclesCB, this, _1));
		
		// read parameters from parameter server
    // parameters from costmap
		if(!local_costmap_nh_.hasParam(costmap_name_+"/global_frame")) ROS_WARN("Used default parameter for global_frame");
		local_costmap_nh_.param(costmap_name_+"/global_frame", global_frame_, std::string("/map"));
		
		if(!local_costmap_nh_.hasParam(costmap_name_+"/robot_base_frame")) ROS_WARN("Used default parameter for robot_frame");
		local_costmap_nh_.param(costmap_name_+"/robot_base_frame", robot_frame_, std::string("/base_link"));
		
    if(!nh_.hasParam(name+"/influence_radius")) ROS_WARN("Used default parameter for influence_radius");
		nh_.param(name+"/influence_radius", influence_radius_, 1.5);
		closest_obstacle_dist_ = influence_radius_;
		
    // parameters for obstacle avoidence and velocity adjustment
		if(!nh_.hasParam(name+"/stop_threshold")) ROS_WARN("Used default parameter for stop_threshold");
		nh_.param(name+"/stop_threshold", stop_threshold_, 0.10);
		
		if(!nh_.hasParam(name+"/obstacle_damping_dist")) ROS_WARN("Used default parameter for obstacle_damping_dist");
		nh_.param(name+"/obstacle_damping_dist", obstacle_damping_dist_, 50.0);
		if(obstacle_damping_dist_ < stop_threshold_) {
			ROS_WARN("obstacle_damping_dist < stop_threshold => robot will stop without decceleration!");
		}
		
		if(!nh_.hasParam(name+"/use_circumscribed_threshold")) ROS_WARN("Used default parameter for use_circumscribed_threshold_");
		nh_.param(name+"/use_circumscribed_threshold", use_circumscribed_threshold_, 0.20);
		
		if(!nh_.hasParam(name+"/vmax")) ROS_WARN("Used default parameter for pot_ctrl_vmax");
		nh_.param(name+"/vmax", v_max_, 0.6);
		
		if(!nh_.hasParam(name+"/vtheta_max")) ROS_WARN("Used default parameter for pot_ctrl_vtheta_max");
		nh_.param(name+"/vtheta_max", vtheta_max_, 0.8);
		
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

    // check for relevant obstacles
    obstacleHandler();
    // stop if we are about to run in an obstacle
    performControllerStep();

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

  //core functions
  void performControllerStep();
  void obstacleHandler();

  //helper functions:
  std::vector<geometry_msgs::Point> loadRobotFootprint(ros::NodeHandle node);
	double sign(double x);
  double getDistance2d(geometry_msgs::Point a, geometry_msgs::Point b);
	bool obstacleValid(double x_robot, double y_robot);
  void stopMovement();

}; //JoystickFilterClass

void JoystickFilterClass::performControllerStep() {

  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear = robot_twist_linear_;
  cmd_vel.angular = robot_twist_angular_;
  double v_max_obst = v_max_, vtheta_max_obst = vtheta_max_, v_factor = 1.0f;

	if( closest_obstacle_dist_ < stop_threshold_ ) {
		stopMovement();
		return;
	}

  //Slow down in direction of closest relevant obstacle: 
	if(closest_obstacle_dist_ < influence_radius_) {
		//implementation for linear decrease of v_max:
		double obstacle_linear_slope = v_max_ / (obstacle_damping_dist_ - stop_threshold_);
		v_max_obst = (closest_obstacle_dist_- stop_threshold_ + stop_threshold_/10.0f) * obstacle_linear_slope;

		if(v_max_obst > v_max_) v_max_obst = v_max_;
			else if(v_max_obst < 0.0f) v_max_obst = 0.0f;
		//implementation for linear decrease of vtheta_max:
		obstacle_linear_slope = vtheta_max_ / (obstacle_damping_dist_ - stop_threshold_);
		vtheta_max_obst = (closest_obstacle_dist_- stop_threshold_ + stop_threshold_/10.0f) * obstacle_linear_slope;

		if(vtheta_max_obst > vtheta_max_) vtheta_max_obst = vtheta_max_;
			else if(vtheta_max_obst < 0.0f) vtheta_max_obst = 0.0f;
	
    //Translational movement
    v_factor = v_max_obst / v_max_; // v_factor only dependent on distance of robot to closest relevant obstacle
  	if(v_factor > 1.0) v_factor = 1.0;
	
  	cmd_vel.linear.x = v_factor * robot_twist_linear_.x;
	  cmd_vel.linear.y = v_factor * robot_twist_linear_.y;

  	//Rotational Movement
    v_factor = vtheta_max_obst / vtheta_max_; // v_factor only dependent on distance of robot to closest relevant obstacle
  	if(v_factor > 1.0) v_factor = 1.0;
	
  	cmd_vel.angular.z = v_factor * robot_twist_angular_.z;
  }

  // publish adjusted velocity 
  topic_pub_command_.publish(cmd_vel);  
  return;
}

void JoystickFilterClass::obstacleHandler() {
  if(!costmap_received_) {
		ROS_WARN("No costmap has been received by cob_joystick_filter, the robot will drive without obstacle avoidance!");
		closest_obstacle_dist_ = influence_radius_;
		
		pthread_mutex_unlock(&m_mutex);
		return;
	}
	closest_obstacle_dist_ = influence_radius_;

	double cur_distance_to_center, cur_distance_to_border;
	double obstacle_theta_robot, obstacle_delta_theta_robot, obstacle_dist_vel_dir;
	bool cur_obstacle_relevant;
	geometry_msgs::Point cur_obstacle_robot;
  geometry_msgs::Point zero_position;
  zero_position.x=0.0f;  
  zero_position.y=0.0f;
  zero_position.z=0.0f;
	bool use_circumscribed=true, use_tube=true;

	//Calculate corner angles in robot_frame:
	double corner_front_left, corner_rear_left, corner_rear_right, corner_front_right;
	corner_front_left = atan2(footprint_left_, footprint_front_);
	corner_rear_left = atan2(footprint_left_, footprint_rear_);
	corner_rear_right = atan2(footprint_right_, footprint_rear_);
	corner_front_right = atan2(footprint_right_, footprint_front_);
	
	//Decide, whether circumscribed or tube argument should be usd for filtering:
	if(fabs(robot_twist_linear_.x) <= 0.005f && fabs(robot_twist_linear_.y) <= 0.005f) {
		use_tube = false;
		//disable tube filter at very slow velocities
	}
	if(!use_tube) {
		if( fabs(robot_twist_angular_.z) <= 0.01f) {
			use_circumscribed = false;
		} //when tube filter inactive, start circumscribed filter at very low rot-velocities
	} else {
		if( fabs(robot_twist_angular_.z) <= use_circumscribed_threshold_) {
			use_circumscribed = false;
		} //when tube filter running, disable circum-filter in a wider range of rot-velocities
	} 
	
	//Calculation of tube in driving-dir considered for obstacle avoidence
	double velocity_angle=0.0f, velocity_ortho_angle;
	double corner_angle, delta_corner_angle;
	double ortho_corner_dist;
	double tube_left_border = 0.0f, tube_right_border = 0.0f;
	double tube_left_origin = 0.0f, tube_right_origin = 0.0f;
	double corner_dist, circumscribed_radius = 0.0f;

	for(unsigned i = 0; i<robot_footprint_.size(); i++) {
		corner_dist = sqrt(robot_footprint_[i].x*robot_footprint_[i].x + robot_footprint_[i].y*robot_footprint_[i].y);
		if(corner_dist > circumscribed_radius) circumscribed_radius = corner_dist;
	}

	if(use_tube) {
		//use commanded vel-value for vel-vector direction.. ?
		velocity_angle = atan2(robot_twist_linear_.y, robot_twist_linear_.x);
		velocity_ortho_angle = velocity_angle + M_PI / 2.0f;

		for(unsigned i = 0; i<robot_footprint_.size(); i++) {
			corner_angle = atan2(robot_footprint_[i].y, robot_footprint_[i].x);
			delta_corner_angle = velocity_ortho_angle - corner_angle;
			corner_dist = sqrt(robot_footprint_[i].x*robot_footprint_[i].x + robot_footprint_[i].y*robot_footprint_[i].y);
			if(corner_dist > circumscribed_radius) circumscribed_radius = corner_dist;
			
      ortho_corner_dist = cos(delta_corner_angle) * corner_dist;
	
			if(ortho_corner_dist < tube_right_border) {
				tube_right_border = ortho_corner_dist;
				tube_right_origin = sin(delta_corner_angle) * corner_dist;
			} else if(ortho_corner_dist > tube_left_border) {
				tube_left_border = ortho_corner_dist;
				tube_left_origin = sin(delta_corner_angle) * corner_dist;
			}
		}
	}

	//find relevant obstacles
	relevant_obstacles_.header = last_costmap_received_.header;
	relevant_obstacles_.cell_width = last_costmap_received_.cell_width;
	relevant_obstacles_.cell_height = last_costmap_received_.cell_height;
	relevant_obstacles_.cells.clear();
	
	for(unsigned int i = 0; i < last_costmap_received_.cells.size(); i++) {
    cur_obstacle_relevant = false;
		cur_distance_to_center = getDistance2d(zero_position, last_costmap_received_.cells[i]);
		//check whether current obstacle lies inside the circumscribed_radius of the robot -> prevent collisions while rotating
    if(use_circumscribed && cur_distance_to_center <= circumscribed_radius) {
			cur_obstacle_robot = last_costmap_received_.cells[i];

			if( obstacleValid(cur_obstacle_robot.x, cur_obstacle_robot.y) ) {
				cur_obstacle_relevant = true;
				relevant_obstacles_.cells.push_back(last_costmap_received_.cells[i]);
				obstacle_theta_robot = atan2(cur_obstacle_robot.y, cur_obstacle_robot.x);
			}
			
		//for each obstacle, now check whether it lies in the tube or not:
		} else if(use_tube && cur_distance_to_center < influence_radius_) {
			cur_obstacle_robot = last_costmap_received_.cells[i];

      if( obstacleValid(cur_obstacle_robot.x, cur_obstacle_robot.y) ) {
				obstacle_theta_robot = atan2(cur_obstacle_robot.y, cur_obstacle_robot.x);
				obstacle_delta_theta_robot = obstacle_theta_robot - velocity_angle;
				obstacle_dist_vel_dir = sin(obstacle_delta_theta_robot) * cur_distance_to_center;
		
				if(obstacle_dist_vel_dir <= tube_left_border && obstacle_dist_vel_dir >= tube_right_border) {
					//found obstacle that lies inside of observation tube
					
					if( sign(obstacle_dist_vel_dir) >= 0) { 
						if(cos(obstacle_delta_theta_robot) * cur_distance_to_center >= tube_left_origin) {
							//relevant obstacle in tube found
							cur_obstacle_relevant = true;
							relevant_obstacles_.cells.push_back(last_costmap_received_.cells[i]);
						}
					} else { // obstacle in right part of tube
						if(cos(obstacle_delta_theta_robot) * cur_distance_to_center >= tube_right_origin) {
							//relevant obstacle in tube found
							cur_obstacle_relevant = true;
							relevant_obstacles_.cells.push_back(last_costmap_received_.cells[i]);
						}
					}
				}
			}
		}
	
		if(cur_obstacle_relevant) { //now calculate distance of current, relevant obstacle to robot
			if(obstacle_theta_robot >= corner_front_right && obstacle_theta_robot < corner_front_left) {
        //obstacle in front:
				cur_distance_to_border = cur_distance_to_center - fabs(footprint_front_) / fabs(cos(obstacle_theta_robot));
			} else if(obstacle_theta_robot >= corner_front_left && obstacle_theta_robot < corner_rear_left) {
        //obstacle left:
				cur_distance_to_border = cur_distance_to_center - fabs(footprint_left_) / fabs(sin(obstacle_theta_robot));
			} else if(obstacle_theta_robot >= corner_rear_left || obstacle_theta_robot < corner_rear_right) {
        //obstacle in rear:
				cur_distance_to_border = cur_distance_to_center - fabs(footprint_rear_) / fabs(cos(obstacle_theta_robot));
			} else {
				//obstacle right:
				cur_distance_to_border = cur_distance_to_center - fabs(footprint_right_) / fabs(sin(obstacle_theta_robot));
			}
			
			if(cur_distance_to_border < closest_obstacle_dist_) closest_obstacle_dist_ = cur_distance_to_border;
		}	
	}
	
	topic_pub_relevant_obstacles_.publish(relevant_obstacles_);
}

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
	
double JoystickFilterClass::getDistance2d(geometry_msgs::Point a, geometry_msgs::Point b) {
	return sqrt( pow(a.x - b.x,2) + pow(a.y - b.y,2) );
}

double JoystickFilterClass::sign(double x) {
	if(x >= 0.0f) return 1.0f;
	else return -1.0f;
}

bool JoystickFilterClass::obstacleValid(double x_obstacle, double y_obstacle) {
	if(x_obstacle<footprint_front_ && x_obstacle>footprint_rear_ && y_obstacle>footprint_right_ && y_obstacle<footprint_left_) {
		ROS_WARN("Found an obstacle inside robot_footprint: Skip!");
		return false;
	}
	
	return true;
}

void JoystickFilterClass::stopMovement() {
  geometry_msgs::Twist stop_twist;
  stop_twist.linear.x = 0.0f; stop_twist.linear.y = 0.0f; stop_twist.linear.z = 0.0f;
  stop_twist.angular.x = 0.0f; stop_twist.angular.y = 0.0f; stop_twist.linear.z = 0.0f;
  topic_pub_command_.publish(stop_twist);
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

