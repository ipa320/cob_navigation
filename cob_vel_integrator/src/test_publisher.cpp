#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <sstream>


using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_publisher");
	
	ros::NodeHandle n;

	ros::Publisher geometry_pub = n.advertise<geometry_msgs::Twist>("input", 1);

	geometry_msgs::Twist cmd_vel;	
	std_msgs::String msg;
	
	ros::Rate loop_rate(3);
	int count = 0;
	//while (ros::ok())
	while(count <= 10)
	{		
		
		cmd_vel.linear.x = 0+count;
		cmd_vel.linear.y = 0+count;
		cmd_vel.linear.z=0+count;
		cmd_vel.angular.x=0+count;
		cmd_vel.angular.y=0+count;
		cmd_vel.angular.z = 0+count;

		//std::stringstream ss;
		//ss << "I think this should work!" << cmd_vel.linear.x << " " << cmd_vel.linear.y << " " << cmd_vel.angular.z;
		//msg.data=ss.str();

		//publish message
		geometry_pub.publish(cmd_vel);

		ROS_INFO("%s","I published something:");
		//ROS_INFO("%s",msg.data.c_str());
		
		cout << cmd_vel << endl;

		//ROS_INFO("%d",cmd_vel.linear.x);
	
		
		ros::spinOnce();
		loop_rate.sleep();
		count++;
	}

	return 0;
}
