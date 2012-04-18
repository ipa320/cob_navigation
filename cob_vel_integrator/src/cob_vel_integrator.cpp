#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <ros/console.h>
#include <deque>
#include <sstream>
#include <iostream>
#include <boost/circular_buffer.hpp>
#include <boost/bind.hpp>
//#include "publlisher.h"

using namespace std;

class cob_vel_integrator
{
private:
	int buffer_capacity;
	//maximal time-delay in seconds for stored messages in Circular Buffer
	double store_delay;
public:	
	
	ros::NodeHandle n;
	boost::circular_buffer<geometry_msgs::Twist> cb;
	boost::circular_buffer<ros::Time> cb_time;

	void setBufferCapacity(int cap, double delay);
	void geometryCallback(const geometry_msgs::Twist& cmd_vel);
	void reviseCircBuff(ros::Time now, geometry_msgs::Twist cmd_vel);
	bool CircBuffOutOfDate(ros::Time now);

	geometry_msgs::Twist meanValue(geometry_msgs::Twist cmd_vel);

};

void cob_vel_integrator::setBufferCapacity(int cap, double delay)
{
	buffer_capacity = cap;
	store_delay = delay;
	cb.set_capacity(buffer_capacity);
	cb_time.set_capacity(buffer_capacity);
};

//returns true if all messages in cb are out of date in consideration of store_delay
bool cob_vel_integrator::CircBuffOutOfDate(ros::Time now)
{
	bool result=true;

	long unsigned int count=0;

	while( (count < cb.size()) && (result == true) ){
		
		double delay=(now.toSec() - cb_time[count].toSec());
		//cout << "CircBuffOutOfDate: " << delay << endl;
		if(delay < store_delay){
			result = false;
		}
		count++;
	}

	return result;

};

void cob_vel_integrator::reviseCircBuff(ros::Time now, geometry_msgs::Twist cmd_vel)
{
	if(this->CircBuffOutOfDate(now) == true){

		cb.clear();
		cb_time.clear();
		//cout << "revise CircBuff: if-case!" << endl;

		//add new command velocity message to circular buffer
		cb.push_front(cmd_vel);
		//add new timestamp for subscribed command velocity message
		cb_time.push_front(ros::Time::now());

	}
	else{
		//cout << "revise CircBuff: else-case!" << endl;
		double delay=(now.toSec() - cb_time.back().toSec());

		while( delay >= store_delay ){
			/*
			cout << "I'm in the while-loop" << endl;

			cout << "buffer-size: " << cb.size() << " " << endl;
			cout << "buffer-back: " << cb.back() << endl;
			cout << "buffer-back-time-delay: " << delay << endl;*/

			//remove out-dated messages
			cb.pop_back();
			cb_time.pop_back();
	
			delay=(now.toSec() - cb_time.back().toSec());
		}

		//add new command velocity message to circular buffer
		cb.push_front(cmd_vel);
		//add new timestamp for subscribed command velocity message
		cb_time.push_front(ros::Time::now());

	}
};

//calculates the mean values of all twist geometry messages contained in the circular buffer
geometry_msgs::Twist cob_vel_integrator::meanValue(geometry_msgs::Twist cmd_vel)
{
	geometry_msgs::Twist result;

	result.linear.x=0;
	result.linear.y=0;
	result.linear.z=0;

	result.angular.x=0;
	result.angular.y=0;
	result.angular.z=0;
	
	//some pre-conditions
	this->reviseCircBuff(ros::Time::now(), cmd_vel);

	//test-print
	//cout << "front-time-delay till now: " << ros::Time::now() - cb_time.back() << " " << endl;
	//cout << "buffer-front: " << cb.front() << endl;
	//cout << "buffer-back: " << cb.back() << endl;
	//cout << "buffer-back-time: " << cb_time.back() << endl;

	long unsigned int size=cb.size();

	//calculate sum
	for(long unsigned int i=0; i<size; i++){

		result.linear.x = result.linear.x + cb[i].linear.x;
		result.linear.y = result.linear.y + cb[i].linear.y;
		result.linear.z = result.linear.z + cb[i].linear.z;

		result.angular.x = result.angular.x + cb[i].angular.x;
		result.angular.y = result.angular.y + cb[i].angular.y;
		result.angular.z = result.angular.z + cb[i].angular.z;

	}
	//calculate mean values
	result.linear.x = result.linear.x / size;
	result.linear.y = result.linear.y / size;
	result.linear.z = result.linear.z / size;

	result.angular.x = result.angular.x / size;
	result.angular.y = result.angular.y / size;
	result.angular.z = result.angular.z / size;
	
	return result;
}

void cob_vel_integrator::geometryCallback(const geometry_msgs::Twist& cmd_vel)
{

	ROS_INFO("%s","I heard something, so here's the callBack-Function!");

	cout << "subscribed-message: " << cmd_vel << endl;
	
	//calculation of meanValues of all relevant elements in the circular buffer
	geometry_msgs::Twist result = this->meanValue(cmd_vel);

	//print result
	cout << "result message: " << result << endl;
	//publish result
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("output", 1);
	pub.publish(result);

};

int main(int argc, char **argv)
{

	ros::init(argc, argv, "cob_vel_integrator");

	cob_vel_integrator my_cvi;

	my_cvi.setBufferCapacity(10,4);
	
	ros::Subscriber sub=my_cvi.n.subscribe("input", 1, &cob_vel_integrator::geometryCallback, &my_cvi);

	ros::spin();

	return 0;
}
