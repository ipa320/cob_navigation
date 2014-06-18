/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2011 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *   All rightes reserved. \n\n
 *
 *****************************************************************
 *
 * \note
 *   Repository name: cob_navigation
 * \note
 *   ROS package name: ipa_navigation_scan_unifier
 *
 * \author
 *   Author: Florian Mirus, email:florian.mirus@ipa.fhg.de
 * \author
 *   Supervised by: Alexander Bubeck, email:alexander.bubeck@ipa.fhg.de
 *
 * \date Date of creation: January 2011
 * \date Last Modification: June 2014
 *
 * \brief
 *   Takes in several laser scans and publishes them as a single one
 *
 ****************************************************************/

#ifndef SCAN_UNIFIER_NODE_H
#define SCAN_UNIFIER_NODE_H

//##################
//#### includes ####

// standard includes
#include <pthread.h>
#include <boost/lexical_cast.hpp>
#include <XmlRpc.h>
#include <math.h>

// ROS includes
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>

// ROS message includes
#include <sensor_msgs/LaserScan.h>


//####################
//#### node class ####
class scan_unifier_node
{
private:
	/** @struct config_struct
	 *  @brief This structure holds configuration parameters
	 *  @var config_struct::number_input_scans 
	 *  Member 'number_input_scans' contains number of scanners to subscribe to
	 *  @var config_struct::loop_rate 
	 *  Member 'loop_rate' contains the loop rate of the ros node
	 *  @var config_struct::start_delay 
	 *  Member 'start_delay' contains the time delay to start calculations after node-init at the earliest
	 *  @var config_struct::input_scan_topics 
	 *  Member 'input_scan_topics' contains the names of the input scan topics
	 */
	struct config_struct{
		int number_input_scans;
		double loop_rate;
		double start_delay;
		std::vector<std::string> input_scan_topics;
	};

	/** @struct laser_scan_struct
	 *  @brief This structure holds the variables for one laser-scanner to be unified
	 *  @var laser_scan_struct::new_msg_received 
	 *  Member 'new_msg_received' contains the information if a new message of this topic has been received
	 *  @var laser_scan_struct::scan_id 
	 *  Member 'scan_id' contains the scan id to subscibe to
	 *  @var laser_scan_struct::scan_id 
	 *  Member 'scan_topic' contains the topic of this scan
	 *  @var laser_scan_struct::laser_sub
	 *  Member 'laser_sub' contains ros subscriber for this laser-scan
	 *  @var laser_scan_struct::current_scan_msg
	 *  Member 'current_scan_msg' contains the current received laser scan msg
	 */
	struct laser_scan_struct{
		bool new_msg_received;
		int scan_id;
		std::string scan_topic;
		ros::Subscriber laser_sub;
		sensor_msgs::LaserScan current_scan_msg;
	};

	pthread_mutex_t m_mutex;

	config_struct config_;

	std::vector<laser_scan_struct> vec_laser_struct_;
public:

	// constructor
	scan_unifier_node();

	// destructor
	~scan_unifier_node();

	/* ----------------------------------- */
	/* --------- ROS Variables ----------- */
	/* ----------------------------------- */

	// create node handles
	ros::NodeHandle nh_, pnh_;

	// declaration of ros publishers
	ros::Publisher topicPub_LaserUnified_;

	// tf listener
	tf::TransformListener listener_;

	// laser geometry projector
	laser_geometry::LaserProjection projector_;

	/* ----------------------------------- */
	/* ----------- functions ------------- */
	/* ----------------------------------- */

	/**
     * @function getParams
     * @brief function to load parameters from ros parameter server
     *
     * input: - 
     * output: - 
     */ 
	void getParams();

	/**
	 * @function set_new_msg_received
	 * @brief setter function for new_msg_received variable
	 *
	 * input:
	 * @param message received information
	 * output: - 
	 */
	void set_new_msg_received(bool received, int scan_id);

	/**
	 * @function get_new_msg_received
	 * @brief getter function for new_msg_received variable for checking wether a new msg has been received, triggering publishers accordingly
	 *
	 * input: - 
	 * output: - 
	 * @return the new_msg_received variable
	 */
	bool get_new_msg_received(int scan_id);

	/**
	 * @function getLoopRate
	 * @brief getter function for the loop rate
	 *
	 * input: - 
	 * output:
	 * @return the loop rate
	 */
	double getLoopRate();

	/**
	 * @function getStartDelay
	 * @brief getter function for the start delay
	 *
	 * input: - 
	 * output:
	 * @return the start delay
	 */
	double getStartDelay();

	/**
	 * @function initLaserScanStructs
	 * @brief initialize a vector of laser scan structs (member variable vec_laser_struct_) with a given number
	 * (from parameter server, stored in config_ struct) 
	 *
	 * input: - 
	 * output: -
	 */
	void initLaserScanStructs();

	/**
	 * @function topicCallbackLaserScan
	 * @brief callback function to subscribe to laser scan messages and store them in vec_laser_struct_
	 *
	 * input: 
	 * @param: a laser scan msg pointer
	 * @param: integer to trigger the storage in vec_laser_struct_
	 * output: - 
	 */
	void topicCallbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan_in, int scan_id);

	/**
	 * @function checkUnifieCondition
	 * @brief check in every node-loop if the unifieConditions holds. A unified scan is only published if new laser 
	 * messages from all scanners have been received
	 *
	 * input: - 
	 * output: -
	 */
	void checkUnifieCondition();

	/**
	 * @function unifieLaserScans
	 * @brief unifie the scan information from all laser scans in vec_laser_struct_
	 *
	 * input: - 
	 * output:
	 * @param: a laser scan message containing unified information from all scanners
	 */
	sensor_msgs::LaserScan unifieLaserScans();

};
#endif