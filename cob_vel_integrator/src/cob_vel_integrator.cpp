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
	//threshhold for allowed distance from zero-value
	double thresh;
	//geometry message filled with zero values
	geometry_msgs::Twist zero_values;

public:
	
	//constructor	
	cob_vel_integrator(int cap, double delay, double zero_thresh);

	ros::NodeHandle n;

	//circular buffers for velocity, acceleration and time
	boost::circular_buffer<geometry_msgs::Twist> cb;
	boost::circular_buffer<geometry_msgs::Twist> cb_out;
	boost::circular_buffer<ros::Time> cb_time;

	ros::Publisher pub;
	//void setBufferCapacity(int cap, double delay);
	void geometryCallback(const geometry_msgs::Twist& cmd_vel);
	void reviseCircBuff(ros::Time now, geometry_msgs::Twist cmd_vel);
	bool CircBuffOutOfDate(ros::Time now);
	bool IsCircBuffZero();
	bool IsEqual(geometry_msgs::Twist msg1, geometry_msgs::Twist msg2);

	double meanValueX();
	double meanValueY();
	double meanValueZ();


	geometry_msgs::Twist setOutput(geometry_msgs::Twist cmd_vel);

};

//constructor
cob_vel_integrator::cob_vel_integrator(int cap, double delay, double zero_thresh)
{
	//set variables
	buffer_capacity = cap;
	store_delay = delay;
	thresh = zero_thresh;

	zero_values.linear.x=0;
	zero_values.linear.y=0;
	zero_values.linear.z=0;

	zero_values.angular.x=0;
	zero_values.angular.y=0;
	zero_values.angular.z=0;

	//initialize crcular buffers
	cb.set_capacity(buffer_capacity);
	cb_out.set_capacity(buffer_capacity);
	cb_time.set_capacity(buffer_capacity);
	
	//set actual ros::Time
	ros::Time now=ros::Time::now();

	//fill circular buffer with zero values
	while(cb.full() == false){

		cb.push_front(zero_values);
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

double cob_vel_integrator::meanValueX()
{
	double result = 0;
	long unsigned int size = cb.size();

	//calculate sum
	for(long unsigned int i=0; i<size; i++){

		result = result + cb[i].linear.x;

	}
	result = result / size;

	if(size > 1){
		double max = cb[0].linear.x;
		long unsigned int max_ind = 0;
		for(long unsigned int i=0; i<size; i++){

			if(abs(result-cb[i].linear.x) > abs(result-max)){
				max = cb[i].linear.x;
				max_ind = i;
			}

		}

		//calculate sum
		for(long unsigned int i=0; i<size; i++){
		
			if(i != max_ind){
				result = result + cb[i].linear.x;
			}
		}
		result = result / (size - 1);
	}
	
	return result;
	
};

double cob_vel_integrator::meanValueY()
{
	double result = 0;
	long unsigned int size = cb.size();

	//calculate sum
	for(long unsigned int i=0; i<size; i++){

		result = result + cb[i].linear.y;

	}
	result = result / size;

	if(size > 1){

		double max = cb[0].linear.y;
		long unsigned int max_ind = 0;
		for(long unsigned int i=0; i<size; i++){

			if(abs(result-cb[i].linear.y) > abs(result-max)){
				max = cb[i].linear.y;
				max_ind = i;
			}

		}

		//calculate sum
		for(long unsigned int i=0; i<size; i++){
		
			if(i != max_ind){
				result = result + cb[i].linear.y;
			}
		}
		result = result / (size - 1);
	}

	return result;
	
};

double cob_vel_integrator::meanValueZ()
{
	double result = 0;
	long unsigned int size = cb.size();

	//calculate sum
	for(long unsigned int i=0; i<size; i++){

		result = result + cb[i].angular.z;

	}
	result = result / size;

	if(size > 1){

		double max = cb[0].angular.z;
		long unsigned int max_ind = 0;
		for(long unsigned int i=0; i<size; i++){

			if(abs(result-cb[i].angular.z) > abs(result-max)){
				max = cb[i].angular.z;
				max_ind = i;
			}

		}

		//calculate sum
		for(long unsigned int i=0; i<size; i++){
		
			if(i != max_ind){
				result = result + cb[i].angular.z;
			}
		}
		result = result / (size - 1);

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

bool cob_vel_integrator::IsCircBuffZero()
{
	bool result=true;
	long unsigned int count=0;
	long unsigned int size = cb.size();

	while( (count < size) && (result == true) ){

		if(this->IsEqual(zero_values, cb[count]) == false){
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
		//add new timestamp for subscribed command velocity message
		cb_time.push_front(now);

	}
};

//calculates the mean values of all twist geometry messages contained in the circular buffer
geometry_msgs::Twist cob_vel_integrator::setOutput(geometry_msgs::Twist cmd_vel)
{
	geometry_msgs::Twist result = zero_values;
	
	//set actual ros::Time
	ros::Time now=ros::Time::now();
	//some pre-conditions
	this->reviseCircBuff(now, cmd_vel);
	
	result.linear.x = meanValueX();
	result.linear.y = meanValueY();
	result.angular.z = meanValueZ();

	if( cb_out.size() > 1){
	
		//set delty velocity and acceleration values
		double deltaX = result.linear.x - cb_out.front().linear.x;
		double accX = deltaX / ( now.toSec() - cb_time.front().toSec() );

		double deltaY = result.linear.y - cb_out.front().linear.y;
		double accY = deltaY / ( now.toSec() - cb_time.front().toSec() );

		double deltaZ = result.angular.z - cb_out.front().linear.y;
		double accZ = deltaZ /  ( now.toSec() - cb_time.front().toSec() );

		if( abs(accX) > thresh){
			result.linear.x = cb_out.front().linear.x + ( thresh *  ( now.toSec() - cb_time.front().toSec() ) );
		}
		if( abs(accY) > thresh){
			result.linear.y = cb_out.front().linear.y + ( thresh *  ( now.toSec() - cb_time.front().toSec() ));
		}
		if( abs(accZ) > thresh){
			result.angular.z = cb_out.front().angular.z + ( thresh *  ( now.toSec() - cb_time.front().toSec() ) );
		}

		cb_out.push_front(result);
	}

	return result;

}

void cob_vel_integrator::geometryCallback(const geometry_msgs::Twist& cmd_vel)
{

	ROS_INFO("%s","I heard something, so here's the callBack-Function!");

	cout << "subscribed-message: " << cmd_vel << endl;
	
	//generate Output messages
	geometry_msgs::Twist result = this->setOutput(cmd_vel);

	//print result
	cout << "result message: " << result << endl;
	//publish result
	pub.publish(result);

};

int main(int argc, char **argv)
{

	ros::init(argc, argv, "cob_vel_integrator");

	cob_vel_integrator my_cvi = cob_vel_integrator(12,4,0.02);
	
	ros::Subscriber sub=my_cvi.n.subscribe("input", 1, &cob_vel_integrator::geometryCallback, &my_cvi);

	ros::spin();

	return 0;
}
