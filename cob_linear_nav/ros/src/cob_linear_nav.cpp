/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering	
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: 
 * ROS package name: cob_linear_nav
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Philipp Koehler, email:Philipp.Koehler@ipa.fhg.de
 *
 * Date of creation: Apr 2011
 * ToDo:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *	 * Redistributions of source code must retain the above copyright
 *		 notice, this list of conditions and the following disclaimer.
 *	 * Redistributions in binary form must reproduce the above copyright
 *		 notice, this list of conditions and the following disclaimer in the
 *		 documentation and/or other materials provided with the distribution.
 *	 * Neither the name of the Fraunhofer Institute for Manufacturing 
 *		 Engineering and Automation (IPA) nor the names of its
 *		 contributors may be used to endorse or promote products derived from
 *		 this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
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
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <cob_srvs/SetDefaultVel.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GridCells.h>

#include <tf/transform_listener.h>

//####################
//#### node class ####
class NodeClass
{
	public:
	// create a handle for this node, initialize node
	ros::NodeHandle nh_;
		
	// declaration of topics
	ros::Publisher topic_pub_command_, action_goal_pub_;
	ros::Publisher topic_pub_relevant_obstacles_;

	ros::Subscriber goal_sub_, obstacles_sub_, odometry_sub_;
	
	// declaration of services
	ros::ServiceServer srv_set_stop_threshold_, srv_set_obstacle_damping_dist_;

	// declaration of action server
	actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> as_;
	move_base_msgs::MoveBaseResult result_; //result is of type geometry_msgs/PoseStamped
	
	//declaration of action client to forward move_base_simple messages
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> * action_client_;
	

	// Constructor
	NodeClass(std::string name) :
		as_(nh_, name, boost::bind(&NodeClass::actionCB, this, _1), false)
	{			
		m_mutex = PTHREAD_MUTEX_INITIALIZER;
	
		// implementation of topics to publish
		topic_pub_command_ = nh_.advertise<geometry_msgs::Twist>("command", 1);
		topic_pub_relevant_obstacles_ = nh_.advertise<nav_msgs::GridCells>("relevant_obstacles", 1);
		
		ros::NodeHandle global_nh("~");
		
		//we'll provide a mechanism for some people to send goals as PoseStamped messages over a topic
		//they won't get any useful information back about its status, but this is useful for tools
		//like nav_view and rviz
		ros::NodeHandle simple_nh("move_base_linear_simple");
		goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&NodeClass::topicCB, this, _1));
		
		action_client_ = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(nh_, name);
		
		//subscribe to the costmap to receive inflated cells
		obstacles_sub_ = nh_.subscribe<nav_msgs::GridCells>("obstacles", 1, boost::bind(&NodeClass::obstaclesCB, this, _1));
		costmap_received_ = false;
		
		//subscribe to obstacles
		odometry_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&NodeClass::odometryCB, this, _1));

		//advertise services
		srv_set_stop_threshold_ = nh_.advertiseService(name+"/set_stop_threshold", &NodeClass::setStopThresholdCB, this);
		srv_set_obstacle_damping_dist_ = nh_.advertiseService(name+"/set_obstacle_damping_dist", &NodeClass::setObstacleDampingStrengthCB, this);
		
		// read parameters from parameter server
		if(!nh_.hasParam(name+"/pot_ctrl_kv")) ROS_WARN("Used default parameter for pot_ctrl_kv");
		nh_.param(name+"/pot_ctrl_kv", kv_, 1.0);
		
		if(!nh_.hasParam(name+"/pot_ctrl_kp")) ROS_WARN("Used default parameter for pot_ctrl_kp");
		nh_.param(name+"/pot_ctrl_kp", kp_, 2.0);
		
		if(!nh_.hasParam(name+"/pot_ctrl_virt_mass")) ROS_WARN("Used default parameter for /pot_ctrl_virt_mass");
		nh_.param(name+"/pot_ctrl_virt_mass", virt_mass_, 0.8);
		
		if(!nh_.hasParam(name+"/pot_ctrl_vmax")) ROS_WARN("Used default parameter for pot_ctrl_vmax");
		nh_.param(name+"/pot_ctrl_vmax", v_max_, 0.6);
		
		if(!nh_.hasParam(name+"/pot_ctrl_vtheta_max")) ROS_WARN("Used default parameter for pot_ctrl_vtheta_max");
		nh_.param(name+"/pot_ctrl_vtheta_max", vtheta_max_, 0.8);
		
		if(!nh_.hasParam(name+"/pot_ctrl_goal_threshold")) ROS_WARN("Used default parameter for pot_ctrl_goal_threshold");
		nh_.param(name+"/pot_ctrl_goal_threshold", goal_threshold_, 0.03);
		
		if(!nh_.hasParam(name+"/pot_ctrl_goal_threshold_rot")) ROS_WARN("Used default parameter for pot_ctrl_goal_threshold_rot");
		nh_.param(name+"/pot_ctrl_goal_threshold_rot", goal_threshold_rot_, 0.08);
		
		if(!nh_.hasParam(name+"/pot_ctrl_speed_threshold")) ROS_WARN("Used default parameter for pot_ctrl_speed_threshold");
		nh_.param(name+"/pot_ctrl_speed_threshold", speed_threshold_, 0.05);
		
		if(!nh_.hasParam(name+"/pot_ctrl_speed_threshold_rot")) ROS_WARN("Used default parameter for pot_ctrl_speed_threshold_rot");
		nh_.param(name+"/pot_ctrl_speed_threshold_rot", speed_threshold_rot_, 0.05);
		
		if(!nh_.hasParam(name+"/pot_ctrl_kp_rot")) ROS_WARN("Used default parameter for pot_ctrl_kp_rot");
		nh_.param(name+"/pot_ctrl_kp_rot", kp_rot_, 0.4);
		
		if(!nh_.hasParam(name+"/pot_ctrl_kv_rot")) ROS_WARN("Used default parameter for pot_ctrl_kv_rot");
		nh_.param(name+"/pot_ctrl_kv_rot", kv_rot_, 1.0);
		
		if(!nh_.hasParam(name+"/pot_ctrl_virt_mass_rot")) ROS_WARN("Used default parameter for pot_ctrl_virt_mass_rot");
		nh_.param(name+"/pot_ctrl_virt_mass_rot", virt_mass_rot_, 0.5);
		
		if(!nh_.hasParam(name+"/global_frame")) ROS_WARN("Used default parameter for global_frame");
		nh_.param(name+"/global_frame", global_frame_, std::string("/map"));
		
		if(!nh_.hasParam(name+"/robot_frame")) ROS_WARN("Used default parameter for robot_frame");
		nh_.param(name+"/robot_frame", robot_frame_, std::string("/base_link"));
		
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
		
		
		//load the robot footprint from the parameter server if its available in the global namespace
		robot_footprint_ = loadRobotFootprint(global_nh);
		if(robot_footprint_.size() > 4) 
			ROS_WARN("You have set more than 4 points as robot_footprint, cob_linear nav can deal only with rectangular footprints so far!");
		
		//generate robot zero_pose
		zero_pose_.pose.position.x = 0.0;
		zero_pose_.pose.position.y = 0.0;
		zero_pose_.pose.position.z = 0.0;
		zero_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
		zero_pose_.header.frame_id = robot_frame_;
		
		

		//we need to make sure that the transform between the robot base frame and the global frame is available
		ros::Time last_error = ros::Time::now();
		std::string tf_error;
		while(!tf_listener_.waitForTransform(global_frame_, robot_frame_, ros::Time(), ros::Duration(0.1), ros::Duration(0.01), &tf_error)) {
			ros::spinOnce();
			if(last_error + ros::Duration(5.0) < ros::Time::now()){
				ROS_WARN("Waiting on transform from %s to %s to become available before running cob_linear_nav, tf error: %s", 
				robot_frame_.c_str(), global_frame_.c_str(), tf_error.c_str());
				last_error = ros::Time::now();
			}
		}
		
		//start action server, it holds the main loop while driving
		as_.start();
	}
	
	void topicCB(const geometry_msgs::PoseStamped::ConstPtr& goal){
		ROS_INFO("In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
		move_base_msgs::MoveBaseGoal action_goal;
		action_goal.target_pose = *goal;

		action_client_->sendGoal(action_goal);
		action_client_->stopTrackingGoal();
	}
	
	void actionCB(const move_base_msgs::MoveBaseGoalConstPtr &goal){ // goal is of type geometry_msgs/PoseStamped
		int obstacle_handler_counter = 0;
		
		ROS_INFO("In idle mode, new goal accepted");
		
		getRobotPoseGlobal();
		x_last_ = robot_pose_global_.pose.position.x;
		y_last_ = robot_pose_global_.pose.position.y;
		theta_last_ = tf::getYaw(robot_pose_global_.pose.orientation);
		vtheta_last_ = 0.0f;
		vx_last_ = 0.0f;
		vy_last_ = 0.0f;
		last_time_ = ros::Time::now().toSec();
		
		goal_pose_global_ = transformGoalToMap(goal->target_pose);
		
		obstacleHandler();
		move_ = true;
		
		ros::Rate loop_rate(100);
		while(nh_.ok()) {
			loop_rate.sleep();

			if(as_.isPreemptRequested()) {
				if(as_.isNewGoalAvailable()) {
					ROS_INFO("New goal received, updating movement");
					//if we're active and a new goal is available, we'll accept it
					move_base_msgs::MoveBaseGoal new_goal = * as_.acceptNewGoal();
					goal_pose_global_ = transformGoalToMap(new_goal.target_pose);
					obstacleHandler();
					
					move_ = true;
				} else {
					ROS_INFO("Preempt requested, aborting movement");
					//notify the ActionServer that we've successfully preempted
					as_.setPreempted();
					
					move_ = false;
					stopMovement();
					return;
				}
			}
			
			if(obstacle_handler_counter >= 1) { //calculate obstacles only every n step to safe CPU-load
				obstacleHandler();
				obstacle_handler_counter = 1;
			} else obstacle_handler_counter ++;

			performControllerStep();
			
			if(finished_) {
				as_.setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
				ROS_INFO("Goal reached.");
				return;
			}
		}
		
		
		//if the node is killed then we'll abort and return
		stopMovement();
		as_.setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
		return;
	};
	
	void odometryCB(const nav_msgs::Odometry::ConstPtr &odometry){ 
		/*tf_listener_.waitForTransform(robot_frame_, odometry->header.frame_id, ros::Time::now(), ros::Duration(5.0));
		
		geometry_msgs::Vector3Stamped vec_stamped;
		
		vec_stamped.vector = odometry->twist.twist.linear;
		vec_stamped.header.frame_id = odometry->header.frame_id;
		tf_listener_.transformVector(robot_frame_, vec_stamped, robot_twist_linear_robot_);
		
		vec_stamped.vector = odometry->twist.twist.angular;
		vec_stamped.header.frame_id = odometry->header.frame_id;
		tf_listener_.transformVector(robot_frame_, vec_stamped, robot_twist_angular_robot_); */
	}
	
	void obstaclesCB(const nav_msgs::GridCells::ConstPtr &obstacles){
		pthread_mutex_lock(&m_mutex);
	
		if(obstacles->cells.size()!=0) costmap_received_ = true;
		last_costmap_received_ = * obstacles;
		
		if(stop_threshold_ < obstacles->cell_width / 2.0f || stop_threshold_ < obstacles->cell_height / 2.0f)
			ROS_WARN("You specified a stop_threshold that is smaller than resolution of received costmap!");
		
		pthread_mutex_unlock(&m_mutex);
	}
	
	bool setStopThresholdCB(cob_srvs::SetDefaultVel::Request &req, cob_srvs::SetDefaultVel::Response &resp) {
		pthread_mutex_lock(&m_mutex);
		
		stop_threshold_ = req.default_vel;
		
		if(obstacle_damping_dist_ < stop_threshold_) {
			ROS_WARN("obstacle_damping_dist < stop_threshold => robot will stop without decceleration!");
			resp.error_message.data = "obstacle_damping_dist < stop_threshold => robot will stop without decceleration!";
		}
		resp.success.data = true;
		
		pthread_mutex_unlock(&m_mutex);
		return true;
	}
	
	bool setObstacleDampingStrengthCB(cob_srvs::SetDefaultVel::Request &req, cob_srvs::SetDefaultVel::Response &resp) {
		pthread_mutex_lock(&m_mutex);
		
		obstacle_damping_dist_ = req.default_vel;
		
		if(obstacle_damping_dist_ < stop_threshold_) {
			ROS_WARN("obstacle_damping_dist < stop_threshold => robot will stop without decceleration!");
			resp.error_message.data = "obstacle_damping_dist < stop_threshold => robot will stop without decceleration!";
		}
		resp.success.data = true;
		
		pthread_mutex_unlock(&m_mutex);
		return true;
	}
	
	// Destructor
	~NodeClass() 
	{
	}
	
	private:
	tf::TransformListener tf_listener_;
	std::string global_frame_, robot_frame_;
	geometry_msgs::PoseStamped goal_pose_global_;
	geometry_msgs::PoseStamped zero_pose_;
	geometry_msgs::PoseStamped robot_pose_global_;
	geometry_msgs::Vector3Stamped robot_twist_linear_robot_, robot_twist_angular_robot_;
	
	//obstacle avoidence
	std::vector<geometry_msgs::Point> robot_footprint_;
	double footprint_left_, footprint_right_, footprint_front_, footprint_rear_;
	double influence_radius_;
	bool costmap_received_;
	nav_msgs::GridCells last_costmap_received_, relevant_obstacles_;
	double closest_obstacle_dist_;
	double stop_threshold_, obstacle_damping_dist_, use_circumscribed_threshold_;
	
	bool finished_, move_;
	
	pthread_mutex_t m_mutex;
	
	//core functions:
	void performControllerStep();
	void obstacleHandler();
	void publishVelocitiesGlobal(double vx, double vy, double theta);
	geometry_msgs::PoseStamped transformGoalToMap(geometry_msgs::PoseStamped goal_pose);
	geometry_msgs::PoseStamped getRobotPoseGlobal();
	//helper functions:
	bool obstacleValid(double x_robot, double y_robot);
	double getDistance2d(geometry_msgs::PoseStamped a, geometry_msgs::PoseStamped b);
	double getDistance2d(geometry_msgs::Point a, geometry_msgs::Point b);
	double getThetaDiffRad(double target, double actual);
	std::vector<geometry_msgs::Point> loadRobotFootprint(ros::NodeHandle node);
	double sign(double x);
	void stopMovement();
	
	//Potential field Controller variables
	double vx_last_, vy_last_, x_last_, y_last_, theta_last_, vtheta_last_;
	double goal_threshold_, speed_threshold_;
	double goal_threshold_rot_, speed_threshold_rot_;
	double kp_, kv_, virt_mass_;
	double kp_rot_, kv_rot_, virt_mass_rot_;
	double last_time_;
	double v_max_, vtheta_max_;

}; //NodeClass

geometry_msgs::PoseStamped NodeClass::transformGoalToMap(geometry_msgs::PoseStamped goal_pose) {
	geometry_msgs::PoseStamped goal_global_;
	if(goal_pose.header.frame_id == global_frame_) return goal_pose;
	else if(tf_listener_.canTransform(global_frame_, goal_pose.header.frame_id, ros::Time(0), new std::string)) {
		tf_listener_.transformPose(global_frame_, goal_pose, goal_global_);
		return goal_global_;
	} else {
		ROS_WARN("Can't transform goal to global frame %s", global_frame_.c_str());
		return robot_pose_global_;
	}
}

geometry_msgs::PoseStamped NodeClass::getRobotPoseGlobal() {
	try{
		tf_listener_.transformPose(global_frame_, zero_pose_, robot_pose_global_);
	}
	catch(tf::TransformException& ex){
		ROS_WARN("Failed to find robot pose in global frame %s", global_frame_.c_str());
		return zero_pose_;
	}
	
	return robot_pose_global_;
}

double NodeClass::getDistance2d(geometry_msgs::PoseStamped a, geometry_msgs::PoseStamped b) {
	return sqrt( pow(a.pose.position.x - b.pose.position.x,2) + pow(a.pose.position.y - b.pose.position.y,2) );
}

double NodeClass::getDistance2d(geometry_msgs::Point a, geometry_msgs::Point b) {
	return sqrt( pow(a.x - b.x,2) + pow(a.y - b.y,2) );
}

double NodeClass::sign(double x) {
	if(x >= 0.0f) return 1.0f;
	else return -1.0f;
}

double NodeClass::getThetaDiffRad(double target, double actual) {
	if(fabs(target - actual) <= M_PI) return (target - actual);
	else return sign(target - actual) * -2.0f * M_PI - (target - actual);
}

bool NodeClass::obstacleValid(double x_robot, double y_robot) {
	if(x_robot<footprint_front_ && x_robot>footprint_rear_ && y_robot>footprint_right_ && y_robot<footprint_left_) {
		ROS_WARN("Found an obstacle inside robot_footprint: Skip!");
		return false;
	}
	
	return true;
}

void NodeClass::publishVelocitiesGlobal(double vx, double vy, double theta) {
	//Transform commands from global frame to robot coordinate system
	geometry_msgs::Vector3Stamped cmd_global, cmd_robot;
	geometry_msgs::Twist msg;
	
	cmd_global.header.frame_id = global_frame_;
	cmd_global.vector.x = vx;
	cmd_global.vector.y = vy;
	try { tf_listener_.transformVector(robot_frame_, cmd_global, cmd_robot);
	} catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		cmd_robot.vector.x = 0.0f;
		cmd_robot.vector.y = 0.0f;
	}
	
	
	msg.linear = cmd_robot.vector;
	msg.angular.z = theta;
	msg.linear.z = 0.0; msg.angular.x = 0.0; msg.angular.y = 0.0;
	
	topic_pub_command_.publish(msg);
}

void NodeClass::stopMovement() {
	publishVelocitiesGlobal(0.0f, 0.0f, 0.0f);
	vx_last_ = 0.0f;
	vy_last_ = 0.0f;
	vtheta_last_ = 0.0f;
}

void NodeClass::performControllerStep() {
	pthread_mutex_lock(&m_mutex);

	double dt;;
	double F_x, F_y, F_theta;
	double theta, theta_goal;
	double cmd_vx, cmd_vy, cmd_vtheta;
	double vx_d, vy_d, vtheta_d, v_factor;
	double kv_obst=kv_, kv_rot_obst=kv_rot_, v_max_obst=v_max_;

	if(!move_) {
		pthread_mutex_unlock(&m_mutex);
		return;
	}
	
	getRobotPoseGlobal();
	
	theta = tf::getYaw(robot_pose_global_.pose.orientation);
	theta_goal = tf::getYaw(goal_pose_global_.pose.orientation);
	
	//exit, if positions and velocities lie inside thresholds
	if( getDistance2d(robot_pose_global_, goal_pose_global_) <= goal_threshold_ &&
	 sqrt(vx_last_ * vx_last_ + vy_last_ * vy_last_) <= speed_threshold_ &&
	 fabs(getThetaDiffRad(theta_goal, theta)) <= goal_threshold_rot_ &&
	 fabs(vtheta_last_) <= speed_threshold_rot_ )
	{
		finished_ = true;
		move_ = false;
		stopMovement();
		pthread_mutex_unlock(&m_mutex);
		return;
	} else if( closest_obstacle_dist_ < stop_threshold_ ) {
		finished_ = true;
		move_ = false;
		stopMovement();
		pthread_mutex_unlock(&m_mutex);
		return;
	} else finished_ = false;

	dt = ros::Time::now().toSec() - last_time_;
	last_time_ = ros::Time::now().toSec();

	//Slow down in any way while approximating an obstacle:
	if(closest_obstacle_dist_ < influence_radius_) {
		//relevant obstacles are present
		
		//implementation for linear increase of damping:
		/*
		double obstacle_damping_slope;
		obstacle_damping_slope = (1.0f - obstacle_damping_dist_) / influence_radius_;
		
		kv_obst = (obstacle_damping_slope * closest_obstacle_dist_ + obstacle_damping_dist_) * kv_;
		kv_rot_obst = (obstacle_damping_slope * closest_obstacle_dist_ + obstacle_damping_dist_) * kv_rot_;
		*/
		
		//implementation for quadr. increase of damping:
		/*
		kv_obst = (1/ pow(closest_obstacle_dist_,obstacle_damping_dist_)) * kv_;
		kv_rot_obst = (1/ pow(closest_obstacle_dist_,obstacle_damping_dist_)) * kv_rot_;
		*/
		
		//implementation for exp incr of damping:
		/*
		kv_obst = 10 * exp(- obstacle_damping_dist_ * (closest_obstacle_dist_ - stop_threshold_)) * kv_;
		kv_rot_obst = 10 * exp(- obstacle_damping_dist_ * (closest_obstacle_dist_ - stop_threshold_/2.0f)) * kv_rot_;
		*/
		
		//implementation for linear decrease of v_max:
		if(obstacle_damping_dist_ < stop_threshold_) ROS_WARN("Slow-down-dist is smaller than stop-dist!");
		double obstacle_linear_slope = v_max_ / (obstacle_damping_dist_ - stop_threshold_);
		v_max_obst = (closest_obstacle_dist_- stop_threshold_ + stop_threshold_/10.0f) * obstacle_linear_slope;

		if(v_max_obst > v_max_) v_max_obst = v_max_;
			else if(v_max_obst < 0.0f) v_max_obst = 0.0f;
		if(kv_obst < kv_) kv_obst = kv_;
		if(kv_rot_obst < kv_rot_) kv_rot_obst = kv_rot_;
	}
	
	//Translational movement
	//calculation of v factor to limit maxspeed
	vx_d = kp_/kv_obst * (goal_pose_global_.pose.position.x - robot_pose_global_.pose.position.x);
	vy_d = kp_/kv_obst * (goal_pose_global_.pose.position.y - robot_pose_global_.pose.position.y);
	v_factor = v_max_obst / sqrt(vy_d*vy_d + vx_d*vx_d);

	if(v_factor > 1.0) v_factor = 1.0;
	
	//TODO: use REAL velocities and not last_commanded here? -> No, to guarantee best linear movement
	F_x = - kv_obst * vx_last_ + v_factor * kp_ * (goal_pose_global_.pose.position.x - robot_pose_global_.pose.position.x);
	F_y = - kv_obst * vy_last_ + v_factor * kp_ * (goal_pose_global_.pose.position.y - robot_pose_global_.pose.position.y);
	//F_x = - kv_ * (robot_pose_global_.pose.position.x - x_last_) / dt + v_factor * kp_ * (goal_pose_global_.pose.position.x - robot_pose_global_.pose.position.x);
	//F_y = - kv_ * (robot_pose_global_.pose.position.y - y_last_) / dt + v_factor * kp_ * (goal_pose_global_.pose.position.y - robot_pose_global_.pose.position.y);

	cmd_vx = vx_last_ + F_x / virt_mass_ * dt;
	cmd_vy = vy_last_ + F_y / virt_mass_ * dt;

	//Rotational Movement
	
	//calculation of v factor to limit maxspeed
	vtheta_d = kp_rot_ / kv_rot_obst * getThetaDiffRad(theta_goal, theta);
	v_factor = fabs(vtheta_max_ / vtheta_d);
	if(v_factor > 1.0) v_factor = 1.0;
	
	F_theta = - kv_rot_obst * vtheta_last_ + v_factor * kp_rot_ * getThetaDiffRad(theta_goal, theta);
	//F_theta = - kv_rot_ * (theta - theta_last_) / dt + v_factor * kp_rot_ * (theta_goal - theta);
	cmd_vtheta = vtheta_last_ + F_theta / virt_mass_rot_ * dt;

	//Publish velocities, these calculated forces and velocities are for the global frame, they are transformed to robot_frame later
	x_last_ = robot_pose_global_.pose.position.x;
	y_last_ = robot_pose_global_.pose.position.y;
	theta_last_ = theta;
	vx_last_ = cmd_vx;
	vy_last_ = cmd_vy;
	vtheta_last_ = cmd_vtheta;

	publishVelocitiesGlobal(cmd_vx, cmd_vy, cmd_vtheta);
	pthread_mutex_unlock(&m_mutex);
}

void NodeClass::obstacleHandler() {
	pthread_mutex_lock(&m_mutex);

	if(!costmap_received_) {
		ROS_WARN("No costmap has been received by cob_linear_nav, the robot will drive without obstacle avoidance!");
		closest_obstacle_dist_ = influence_radius_;
		
		pthread_mutex_unlock(&m_mutex);
		return;
	}
	closest_obstacle_dist_ = influence_radius_;

	double cur_distance_to_center, cur_distance_to_border;
	double obstacle_theta_robot, obstacle_delta_theta_robot, obstacle_dist_vel_dir;
	bool cur_obstacle_relevant;
	geometry_msgs::PointStamped obstacle_stamped, cur_obstacle_robot;
	obstacle_stamped.header.frame_id = last_costmap_received_.header.frame_id;
	bool use_circumscribed=true, use_tube=true;

	//Calculate corner angles in robot_frame:
	double corner_front_left, corner_rear_left, corner_rear_right, corner_front_right;
	corner_front_left = atan2(footprint_left_, footprint_front_);
	corner_rear_left = atan2(footprint_left_, footprint_rear_);
	corner_rear_right = atan2(footprint_right_, footprint_rear_);
	corner_front_right = atan2(footprint_right_, footprint_front_);
	
	//Decide, whether circumscribed or tube argument should be usd for filtering:
	if(fabs(vx_last_) <= 0.005f && fabs(vy_last_) <= 0.005f) {
		use_tube = false;
		//disable tube filter at very slow velocities
	}
	if(!use_tube) {
		if( fabs(vtheta_last_) <= 0.01f) {
			use_circumscribed = false;
		} //when tube filter inactive, start circumscribed filter at very low rot-velocities
	} else {
		if( fabs(vtheta_last_) <= use_circumscribed_threshold_) {
			use_circumscribed = false;
		} //when tube filter running, disable circum-filter in a wider range of rot-velocities
	} 
	
	//Calculation of tube in driving-dir considered for obstacle avoidence
	double velocity_angle=0.0f, velocity_ortho_angle;
	double corner_angle, delta_corner_angle;
	geometry_msgs::Vector3Stamped v_last_global, v_last_robot;
	double ortho_corner_dist;
	double tube_left_border = 0.0f, tube_right_border = 0.0f;
	double tube_left_origin = 0.0f, tube_right_origin = 0.0f;
	double corner_dist, circumscribed_radius = 0.0f;

	if(use_tube) {
		//use commanded vel-value for vel-vector direction.. ?
		//velocity_angle = atan2(robot_twist_linear_robot_.vector.y, robot_twist_linear_robot_.vector.x);
		v_last_global.header.frame_id = global_frame_;
		v_last_global.vector.x = vx_last_;
		v_last_global.vector.y = vy_last_;
		
		try { tf_listener_.transformVector(robot_frame_, v_last_global, v_last_robot);
		} catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
		}
		
		
		
		velocity_angle = atan2(v_last_robot.vector.y, v_last_robot.vector.x);
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
		cur_distance_to_center = getDistance2d(robot_pose_global_.pose.position, last_costmap_received_.cells[i]);
		
		//check whether current obstacle lies inside the circumscribed_radius of the robot -> prevent collisions while rotating
		if(use_circumscribed && cur_distance_to_center <= circumscribed_radius) {
			//transform obstacle to robot coord-system
			obstacle_stamped.point = last_costmap_received_.cells[i];

			try { tf_listener_.transformPoint(robot_frame_, obstacle_stamped, cur_obstacle_robot);
			} catch (tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
				cur_obstacle_robot.point.x = 0.0;
				cur_obstacle_robot.point.y = 0.0; //to make cur_obstacle invalid
			}
		
			if( obstacleValid(cur_obstacle_robot.point.x, cur_obstacle_robot.point.y) ) {
				cur_obstacle_relevant = true;
				relevant_obstacles_.cells.push_back(last_costmap_received_.cells[i]);
				obstacle_theta_robot = atan2(cur_obstacle_robot.point.y, cur_obstacle_robot.point.x);
			}
			
		//for each obstacle, now check whether it lies in the tube or not:
		} else if(use_tube && cur_distance_to_center < influence_radius_) {
	
			//transform obstacle to robot coord-system
			obstacle_stamped.point = last_costmap_received_.cells[i];
			
			try { tf_listener_.transformPoint(robot_frame_, obstacle_stamped, cur_obstacle_robot);
			} catch (tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
				cur_obstacle_robot.point.x = 0.0;
				cur_obstacle_robot.point.y = 0.0; //to make cur_obstacle invalid
			}
			
			if( obstacleValid(cur_obstacle_robot.point.x, cur_obstacle_robot.point.y) ) {
			
				obstacle_theta_robot = atan2(cur_obstacle_robot.point.y, cur_obstacle_robot.point.x);
		
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
	
	//if(closest_obstacle_dist_ < influence_radius_) std::cout << "Closest obstacle: " << closest_obstacle_dist_ << " tube/cir: " << use_tube << use_circumscribed << std::endl;
	
	topic_pub_relevant_obstacles_.publish(relevant_obstacles_);
	
	pthread_mutex_unlock(&m_mutex);
}

std::vector<geometry_msgs::Point> NodeClass::loadRobotFootprint(ros::NodeHandle node){ //from costmap_2d_ros to keep same footprint format
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
	ROS_DEBUG("Extracted rectangular footprint cob_potetnial_nav: Front: %f, Rear %f, Left: %f, Right %f", footprint_front_, footprint_rear_, footprint_left_, footprint_right_);

	return footprint;
	}


//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	// initialize ROS, spezify name of node
	ros::init(argc, argv, "cob_linear");
	
	// create nodeClass
	NodeClass nodeClass("move_base_linear");

 	ros::spin();

	return 0;
}

