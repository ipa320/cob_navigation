#include "ros/ros.h"
#include <cob_3d_mapping_msgs/GetApproachPoseForPolygon.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <cob_3d_mapping_common/polygon.h>
#include <cob_3d_mapping_common/ros_msg_conversions.h>
#include <tf/tf.h>

int main(int argc, char **argv)
{
	cob_3d_mapping::Polygon p1;
	Eigen::Vector2f v;
	std::vector<Eigen::Vector2f> vv;
	p1.id_ = 1;
	p1.normal_ << 0.0,0.0,1.0;
	p1.d_ = -1;
	v << 1,-2;//,1;
	vv.push_back(v);
	v << 1,-3;//,1;
	vv.push_back(v);
	v << 2,-3;//,1;
	vv.push_back(v);
	v << 2,-2;//,1;
	vv.push_back(v);
	p1.contours_.push_back(vv);
	p1.holes_.push_back(false);
	cob_3d_mapping_msgs::Shape p_msg;
	toROSMsg(p1, p_msg);

	ros::init (argc, argv, "client");
	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<cob_3d_mapping_msgs::GetApproachPoseForPolygon> ("compute_approach_pose");
	cob_3d_mapping_msgs::GetApproachPoseForPolygonRequest req;
	req.polygon = p_msg;
	cob_3d_mapping_msgs::GetApproachPoseForPolygonResponse res;
	client.call (req, res);

	for (unsigned int i=0; i<res.approach_poses.poses.size(); i++)
	{
		tf::Quaternion q;
		tf::quaternionMsgToTF(res.approach_poses.poses[i].orientation, q);
		std::cout << "i=" << i << "  x=" << res.approach_poses.poses[i].position.x << "  y=" << res.approach_poses.poses[i].position.y << "  ang=" << tf::getYaw(q) << std::endl;
	}

	return 0;
}
