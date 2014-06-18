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

#include <scan_unifier_node.h>

// constructor
scan_unifier_node::scan_unifier_node()
{
	ROS_DEBUG("init laser unification");
	// create node handles
	nh_ = ros::NodeHandle();
	pnh_ = ros::NodeHandle("~");

	// publisher
	topicPub_LaserUnified_ = nh_.advertise<sensor_msgs::LaserScan>("scan_unified", 1);

	getParams();

	ROS_DEBUG("scan unifier: now init laser structs");

	pthread_mutex_init(&m_mutex, NULL);

	initLaserScanStructs();


}

// destructor
scan_unifier_node::~scan_unifier_node(){}

/**
 * @function getParams
 * @brief function to load parameters from ros parameter server
 *
 * input: - 
 * output: - 
 */ 
void scan_unifier_node::getParams()
{

	if(!pnh_.hasParam("loop_rate"))
	{
	    ROS_WARN("No parameter loop_rate on parameter server. Using default value [100.0].");
	}
	pnh_.param("loop_rate", config_.loop_rate, (double)100.0);

	if(!pnh_.hasParam("start_delay"))
	{
	    ROS_WARN("No parameter start_delay on parameter server. Using default value [2.0].");
	}
	pnh_.param("start_delay", config_.start_delay, (double)2.0);

	XmlRpc::XmlRpcValue topicList;

	if (pnh_.getParam("input_scans", topicList))
	{
		ROS_ASSERT(topicList.getType() == XmlRpc::XmlRpcValue::TypeArray);

		if(topicList.getType() == XmlRpc::XmlRpcValue::TypeArray)
		{
			for (int32_t i = 0; i < topicList.size(); ++i) 
			{
			  ROS_ASSERT(topicList[i].getType() == XmlRpc::XmlRpcValue::TypeString);
			  config_.input_scan_topics.push_back(static_cast<std::string>(topicList[i]));
			
			  ROS_DEBUG_STREAM("Parsed the scan topic: " << config_.input_scan_topics.back());
			}

			config_.number_input_scans = config_.input_scan_topics.size();
		}
	}
	else
	{
		config_.number_input_scans = 0;
		ROS_ERROR("No parameter input_scans on parameter server!! Scan unifier can not subscribe to any scan topic!");
	}
}

/**
 * @function getLoopRate
 * @brief getter function for the loop rate
 *
 * input: - 
 * output:
 * @return the loop rate
 */
double scan_unifier_node::getLoopRate()
{
	return config_.loop_rate;
}

/**
 * @function getStartDelay
 * @brief getter function for the start delay
 *
 * input: - 
 * output:
 * @return the start delay
 */
double scan_unifier_node::getStartDelay()
{
	return config_.start_delay;	
}

/**
 * @function set_new_msg_received
 * @brief setter function for new_msg_received variable
 *
 * input:
 * @param message received information
 * @param scan id to find right scan in struct
 * output: - 
 */
void scan_unifier_node::set_new_msg_received(bool received, int scan_id)
{
  pthread_mutex_lock(&m_mutex);
  vec_laser_struct_.at(scan_id).new_msg_received = received;
  pthread_mutex_unlock(&m_mutex);
}

/**
 * @function get_new_msg_received
 * @brief getter function for new_msg_received variable for checking wether a new msg has been received, triggering publishers accordingly
 *
 * input:
 * @param scan id to find right scan in struct
 * output: - 
 * @return the new_msg_received variable
 */
bool scan_unifier_node::get_new_msg_received(int scan_id)
{
  pthread_mutex_lock(&m_mutex);
  bool ret_val = vec_laser_struct_.at(scan_id).new_msg_received;
  pthread_mutex_unlock(&m_mutex);
  return ret_val;
}

/**
 * @function initLaserScanStructs
 * @brief initialize a vector of laser scan structs (member variable vec_laser_struct_) with a given number
 * (from parameter server, stored in config_ struct) 
 *
 * input: - 
 * output: -
 */
void scan_unifier_node::initLaserScanStructs()
{
	laser_scan_struct dummy_struct;
	vec_laser_struct_.assign(config_.number_input_scans, dummy_struct);

	for(int i = 0; i < config_.number_input_scans; i++)
	{
		set_new_msg_received(false, i);
		vec_laser_struct_.at(i).scan_id = i;
		vec_laser_struct_.at(i).scan_topic = config_.input_scan_topics.at(i);
		vec_laser_struct_.at(i).current_scan_msg = sensor_msgs::LaserScan();
		vec_laser_struct_.at(i).laser_sub = nh_.subscribe<sensor_msgs::LaserScan>
			(vec_laser_struct_.at(i).scan_topic , 1, boost::bind(&scan_unifier_node::topicCallbackLaserScan, this, _1, i));
	}
}

/**
 * @function topicCallbackLaserScan
 * @brief callback function to subscribe to laser scan messages and store them in vec_laser_struct_
 *
 * input: 
 * @param: a laser scan msg pointer
 * @param: integer to trigger the storage in vec_laser_struct_
 * output: - 
 */
void scan_unifier_node::topicCallbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan_in, int scan_id)
{
	ROS_DEBUG_STREAM("scan-unifier: Laser-callback!");
	if(!get_new_msg_received(scan_id))
	{
		vec_laser_struct_.at(scan_id).current_scan_msg = *scan_in;
		ROS_DEBUG_STREAM("received msg from scan_id: " << scan_id);
		set_new_msg_received(true, scan_id);
	}
	else
	{
		ROS_DEBUG_STREAM("received msg from scan_id: " << scan_id << "but storage blocked by mutex");
	}
}

/**
 * @function unifieLaserScans
 * @brief unifie the scan information from all laser scans in vec_laser_struct_
 *
 * input: - 
 * output:
 * @param: a laser scan message containing unified information from all scanners
 */
sensor_msgs::LaserScan scan_unifier_node::unifieLaserScans()
{
	sensor_msgs::LaserScan unified_scan = sensor_msgs::LaserScan();
	std::vector<sensor_msgs::PointCloud> vec_cloud;
	vec_cloud.assign(config_.number_input_scans, sensor_msgs::PointCloud());

	if(!vec_laser_struct_.empty())
	{	
		ROS_DEBUG("start converting");
		for(int i=0; i < config_.number_input_scans; i++)
		{
			vec_cloud.at(i).header.stamp = vec_laser_struct_.at(i).current_scan_msg.header.stamp;
			ROS_DEBUG_STREAM("Converting scans to point clouds at index: " << i << ", at time: " << vec_laser_struct_.at(i).current_scan_msg.header.stamp << " now: " << ros::Time::now());
			listener_.waitForTransform("/base_link", vec_laser_struct_.at(i).current_scan_msg.header.frame_id, 
				vec_laser_struct_.at(i).current_scan_msg.header.stamp, ros::Duration(3.0));
			ROS_DEBUG("now project to point_cloud");
	  		projector_.transformLaserScanToPointCloud("/base_link",vec_laser_struct_.at(i).current_scan_msg, vec_cloud.at(i), listener_);
		}
		ROS_DEBUG("Creating message header");
		unified_scan.header = vec_laser_struct_.at(0).current_scan_msg.header;
	    unified_scan.header.frame_id = "base_link";
		unified_scan.angle_min = -M_PI+0.1;
		unified_scan.angle_max = M_PI-0.1;
		unified_scan.angle_increment = M_PI/180.0/2.0;
		unified_scan.time_increment = 0.0;
		unified_scan.scan_time = vec_laser_struct_.at(0).current_scan_msg.scan_time;
		unified_scan.range_min = vec_laser_struct_.at(0).current_scan_msg.range_min;
		unified_scan.range_max = vec_laser_struct_.at(0).current_scan_msg.range_max;
		unified_scan.ranges.resize(round((unified_scan.angle_max - unified_scan.angle_min) / unified_scan.angle_increment) + 1);

		// now unifie all Scans
		ROS_DEBUG("unifie scans");
		for(int j = 0; j < config_.number_input_scans; j++)
		{
			for (unsigned int i = 0; i < vec_cloud.at(j).points.size(); i++)
	      	{
		    	const float &x = vec_cloud.at(j).points[i].x;
		      	const float &y = vec_cloud.at(j).points[i].y;
		      	const float &z = vec_cloud.at(j).points[i].z;
				//ROS_INFO("Point %f %f %f", x, y, z);
				if ( std::isnan(x) || std::isnan(y) || std::isnan(z) )
		      	{
					ROS_DEBUG("rejected for nan in point(%f, %f, %f)\n", x, y, z);
		        	continue;
				}
				double angle = atan2(y, x);// + M_PI/2;
		      	if (angle < unified_scan.angle_min || angle > unified_scan.angle_max)
		      	{
		        	ROS_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, unified_scan.angle_min, unified_scan.angle_max);
		        	continue;
		      	}
				int index = (angle - unified_scan.angle_min) / unified_scan.angle_increment;
				double range_sq = y*y+x*x;
		        //printf ("index xyz( %f %f %f) angle %f index %d range %f\n", x, y, z, angle, index, sqrt(range_sq));
		        unified_scan.ranges[index] = sqrt(range_sq);
			}
		}

	}

	return unified_scan;
}

/**
 * @function checkUnifieCondition
 * @brief check in every node-loop if the unifieConditions holds. A unified scan is only published if new laser 
 * messages from all scanners have been received
 *
 * input: - 
 * output: -
 */
void scan_unifier_node::checkUnifieCondition()
{
	bool all_scans_received = true;
	if(!vec_laser_struct_.empty())
	{
		int ind = 0;
		while(ind < config_.number_input_scans && all_scans_received)
		{
			// if one scan-struct did not receive a new msg all_scans_received is set to false and we do nothing
			//all_scans_received = vec_laser_struct_.at(ind).new_msg_received;
			all_scans_received = get_new_msg_received(ind);
			ind++;
		}
	}

	if(all_scans_received)
	{
		// all scan-structs received a new msg so now unifie all of them
		ROS_DEBUG("all_scans_received");
		sensor_msgs::LaserScan unified_scan = unifieLaserScans();
		ROS_DEBUG("now publish");
		topicPub_LaserUnified_.publish(unified_scan);
		for(int i=0; i < config_.number_input_scans; i++)
		{
			set_new_msg_received(false, i);
		}
	}
}

int main(int argc, char** argv)
{
	ROS_DEBUG("scan unifier: start scan unifier node");
	ros::init(argc, argv, "cob_unified_scan_publisher");

	scan_unifier_node my_scan_unifier_node;

	// store initialization time of the node 
	ros::Time start = ros::Time::now();

	ros::Duration delay(my_scan_unifier_node.getStartDelay());

	ros::Rate rate(my_scan_unifier_node.getLoopRate());

	// start the calculation at the earliest after a given time interval to avoid tf-crash
	ROS_DEBUG_STREAM("scan unifier: sleep for " << my_scan_unifier_node.getStartDelay() << " seconds before start of scan unification");
	ros::Duration(my_scan_unifier_node.getStartDelay()).sleep();
	// actual calculation step with given frequency
	while(my_scan_unifier_node.nh_.ok()){
		// start the calculation at the earliest after a given time interval to avoid tf-crash
		
		if(ros::Time::now().toSec() > (start.toSec() + delay.toSec()))
		{
			my_scan_unifier_node.checkUnifieCondition();
		}
		
		ros::spinOnce();
		rate.sleep();

	}

	return 0;
}