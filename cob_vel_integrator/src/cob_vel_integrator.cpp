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
	//capacity for circular buffers
	int buffer_capacity;
	//maximal time-delay in seconds for stored messages in Circular Buffer
	double store_delay;
	//geometry message filled with zero values
	geometry_msgs::Twist zero_values;
public:
	//constructor	
	cob_vel_integrator(int cap, double delay);

	ros::NodeHandle n;

	//circular buffers for velocity, acceleration and time
	boost::circular_buffer<geometry_msgs::Twist> cb;
	boost::circular_buffer<geometry_msgs::Twist> cb_in;
	boost::circular_buffer<ros::Time> cb_time;

	ros::Publisher pub;
	//void setBufferCapacity(int cap, double delay);
	void geometryCallback(const geometry_msgs::Twist& cmd_vel);
	void reviseCircBuff(ros::Time now, geometry_msgs::Twist cmd_vel);
	bool CircBuffOutOfDate(ros::Time now);
	bool IsCircBuffInZero();
	bool IsEqual(geometry_msgs::Twist msg1, geometry_msgs::Twist msg2);

	geometry_msgs::Twist meanValue(geometry_msgs::Twist cmd_vel);

};

//constructor
cob_vel_integrator::cob_vel_integrator(int cap, double delay)
{
	//set variables
	buffer_capacity = cap;
	store_delay = delay;

	zero_values.linear.x=0;
	zero_values.linear.y=0;
	zero_values.linear.z=0;

	zero_values.angular.x=0;
	zero_values.angular.y=0;
	zero_values.angular.z=0;

	//initialize crcular buffers
	cb.set_capacity(buffer_capacity);
	cb_in.set_capacity(buffer_capacity);
	cb_time.set_capacity(buffer_capacity);
	
	//set actual ros::Time
	ros::Time now=ros::Time::now();

	//fill circular buffer with zero values
	while(cb.full() == false){

		cb.push_front(zero_values);
		//cb_in.push_back(zero_values);
		cb_time.push_front(now);

	}

	pub = n.advertise<geometry_msgs::Twist>("output", 1);

};

//returns true if all messages in cb are out of date in consideration of store_delay
bool cob_vel_integrator::CircBuffOutOfDate(ros::Time now)
{
	bool result=true;

	long unsigned int count=0;

	while( (count < cb.size()) && (result == true) ){
		
		double delay=(now.toSec() - cb_time[count].toSec());

		if(delay < store_delay){
			result = false;
		}
		count++;
	}

	return result;

};

//returns true if the input geometry messages are equal
bool cob_vel_integrator::IsEqual(geometry_msgs::Twist msg1, geometry_msgs::Twist msg2)
{
	if( (msg1.linear.x == msg2.linear.x) && (msg1.linear.y == msg2.linear.y) && (msg1.linear.z == msg2.linear.z) && (msg1.angular.x == msg2.angular.x) && (msg1.angular.y == msg2.angular.y) && (msg1.angular.z == msg2.angular.z)){
		return true;
	}
	else{	
		return false;
	}
};

bool cob_vel_integrator::IsCircBuffInZero()
{
	bool result=true;
	long unsigned int count=0;
	long unsigned int size = cb_in.size();

	while( (count < size) && (result == true) ){

		if(this->IsEqual(zero_values, cb_in[count]) == false){
			result=false;
		}
		count++;
	}

	return result;

};

void cob_vel_integrator::reviseCircBuff(ros::Time now, geometry_msgs::Twist cmd_vel)
{
	if(this->CircBuffOutOfDate(now) == true){

		//clear buffers
		cb.clear();
		cb_time.clear();

		//fill circular buffer with zero_values and time buffer with actual time-stamp
		while(cb.full() == false){

			cb.push_front(zero_values);
			cb_time.push_front(now);

		}

		//add new command velocity message to circular buffer
		cb.push_front(cmd_vel);
		cb_in.push_front(cmd_vel);
		//add new timestamp for subscribed command velocity message
		cb_time.push_front(now);

	}
	else{
		//cout << "revise CircBuff: else-case!" << endl;
		double delay=(now.toSec() - cb_time.back().toSec());

		while( delay >= store_delay ){
			//remove out-dated messages
			cb.pop_back();
			cb_time.pop_back();
	
			delay=(now.toSec() - cb_time.back().toSec());
		}

		//add new command velocity message to circular buffer
		cb.push_front(cmd_vel);
		cb_in.push_front(cmd_vel);
		//add new timestamp for subscribed command velocity message
		cb_time.push_front(now);

	}
};

//calculates the mean values of all twist geometry messages contained in the circular buffer
geometry_msgs::Twist cob_vel_integrator::meanValue(geometry_msgs::Twist cmd_vel)
{
	geometry_msgs::Twist result = zero_values;
	
	//set actual ros::Time
	ros::Time now=ros::Time::now();
	//some pre-conditions
	this->reviseCircBuff(now, cmd_vel);

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
	result.linear.x = result.linear.x / buffer_capacity;
	result.linear.y = result.linear.y / buffer_capacity;
	result.linear.z = result.linear.z / buffer_capacity;

	result.angular.x = result.angular.x / buffer_capacity;
	result.angular.y = result.angular.y / buffer_capacity;
	result.angular.z = result.angular.z / buffer_capacity;

	//return result;

	//delete last incoming geometry message from circular buffer
	//cb.pop_front();
	//add calculated result message to circular buffer
	//cb.push_front(result);
		
	if(this->IsCircBuffInZero() == true){
		//cb.pop_front();
		//cb.push_front(zero_values);
		return result;
	}
	else{
		cb.pop_front();
		//add calculated result message to circular buffer
		cb.push_front(result);

		return result;
	}
	
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
	pub.publish(result);

};

int main(int argc, char **argv)
{

	ros::init(argc, argv, "cob_vel_integrator");

	cob_vel_integrator my_cvi = cob_vel_integrator(12,4);
	
	ros::Subscriber sub=my_cvi.n.subscribe("input", 1, &cob_vel_integrator::geometryCallback, &my_cvi);

	ros::spin();

	return 0;
}
