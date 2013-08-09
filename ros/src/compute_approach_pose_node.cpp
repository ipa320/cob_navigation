#include "ros/ros.h"
#include <vector>
#include <math.h>

#include <nav_msgs/OccupancyGrid.h>
#include <cob_3d_mapping_msgs/GetApproachPoseForPolygon.h>

// opencv
#include <opencv/cv.h>
#include <opencv/highgui.h>
//#include <opencv2/ml/ml.hpp>
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>

#include <pcl/ros/conversions.h>
#include <pcl/point_types.h>

#include <tf/transform_listener.h>


struct Pose
{
    float x;
    float y;
    float orientation; //in degree

    Pose()
    {
    	x = 0;
    	y = 0;
    	orientation = 0;
    }

    Pose(float x_, float y_, float orientation_)
    {
    	x = x_;
    	y = y_;
    	orientation = orientation_;
    }
};

///** from here
class ComputeApproachPoseNode
{
public:

	void init(ros::NodeHandle nh)
	{
		node_handle_ = nh;
		map_resolution_ = 0;

		robot_radius_ = 0.8;	// in [m]

		static_map_sub_ = node_handle_.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &ComputeApproachPoseNode::updateMapCallback, this);

		compute_approach_pose_server_ = node_handle_.advertiseService("compute_approach_pose", &ComputeApproachPoseNode::computeApproachPose, this);
	}

	// this function just copies the received map into opencv's format
	// and creates an inflated version of the map
	void updateMapCallback(const nav_msgs::OccupancyGridConstPtr& map_msg)
	{
		// copy properties
		map_resolution_ = map_msg->info.resolution;
		map_origin_ = cv::Point2d(map_msg->info.origin.position.x, map_msg->info.origin.position.y);

		// create empty copy of map
		map_ = 255*cv::Mat::ones(map_msg->info.height, map_msg->info.width, CV_8UC1);

		// copy real static map into cv::Mat element-wise
		for (unsigned int v=0, i=0; v<map_msg->info.height; v++)
		{
			for (unsigned int u=0; u<map_msg->info.width; u++, i++)
			{
				if (map_msg->data[i] != 0)
					map_.at<unsigned char>(v,u) = 0;
			}
		}

//		// create the inflated map
//		int iterations = (int)(robot_radius_/map_resolution_);
//		//std::cout << "iterations=" << iterations << std::endl;
//		cv::erode(map_, expanded_map_, cv::Mat(), cv::Point(-1,-1), iterations);

		// display maps
//		cv::imshow("blown up map", expanded_map_);
//		cv::imshow("map", map_);
//		cv::waitKey(10);


		ROS_INFO("ComputeApproachPoseNode: Map received.");
	}

	// pose_p and closest_point_on_polygon in pixel coordinates!
	void computeClosestPointOnPolygon(const cv::Mat& map_with_polygon, const Pose& pose_p, Pose& closest_point_on_polygon)
	{
		double closest_pixel_distance_squared = 1e10;
		for (int v=0; v<map_with_polygon.rows; v++)
		{
			for (int u=0; u<map_with_polygon.cols; u++)
			{
				double dist_squared = 0;
				if (map_with_polygon.at<uchar>(v,u) == 128 && (dist_squared=(pose_p.x-u)*(pose_p.x-u)+(pose_p.y-v)*(pose_p.y-v))<closest_pixel_distance_squared)
				{
					closest_pixel_distance_squared = dist_squared;
					closest_point_on_polygon.x = u;
					closest_point_on_polygon.y = v;
				}
			}
		}
	}

	bool computeApproachPose(cob_3d_mapping_msgs::GetApproachPoseForPolygon::Request& req, cob_3d_mapping_msgs::GetApproachPoseForPolygon::Response& res)
	{
		// determine robot pose
		tf::StampedTransform transform;
		try
		{
			tf_listener_.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(1));
			tf_listener_.lookupTransform("/map", "/base_link", ros::Time(0), transform);
		}
	    catch (tf::TransformException ex)
	    {
	      ROS_ERROR("[registration] : %s",ex.what());
	      return false;
	    }
	    tf::Vector3 pose = transform.getOrigin();
	    cv::Point robot_location = convertFromMeterToPixelCoordinates<cv::Point>(Pose(pose.x(), pose.y(), 0));

		// copy contours
		std::vector< std::vector<cv::Point> > polygon_contours;
		for (unsigned int i=0; i<req.polygon.points.size(); i++)
		{
			if(req.polygon.holes[i] == false)
			{
				pcl::PointCloud<pcl::PointXYZ> pc;
				pcl::fromROSMsg(req.polygon.points[i], pc);
				std::vector<cv::Point> p_vec(pc.size());
				for( unsigned int j=0; j<pc.size(); j++)
				{
					p_vec[j] = convertFromMeterToPixelCoordinates<cv::Point>(Pose(pc.points[j].x, pc.points[j].y, 0));
				}
				polygon_contours.push_back(p_vec);
			}
		}

		// add contours to map
		cv::Mat map_expanded = map_.clone();
		cv::drawContours(map_expanded, polygon_contours, -1, cv::Scalar(128), CV_FILLED);
		cv::Mat map_with_polygon = map_expanded.clone();

		//cv::imshow("map", map_expanded);
		//cv::waitKey();

		// create the inflated map
		int iterations = (int)(robot_radius_/map_resolution_);
		cv::erode(map_expanded, map_expanded, cv::Mat(), cv::Point(-1,-1), iterations);
		//cv::imshow("expanded map", map_expanded);
		//cv::waitKey();

		// compute gradients
//		cv::Mat dx, dy;
//		cv::Sobel(map_expanded, dx, CV_32F, 1, 0, 3);
//		cv::Sobel(map_expanded, dy, CV_32F, 0, 1, 3);

		// find the individual connected areas
		std::vector< std::vector<cv::Point> > area_contours;		// first index=contour index;  second index=point index within contour
		cv::Mat map_expanded_copy = map_expanded.clone();
		cv::findContours(map_expanded_copy, area_contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

		// iterate through all white points and consider those as potential approach poses that have an expanded table pixel in their neighborhood
		for (int y=1; y<map_expanded.rows-1; y++)
		{
			for (int x=1; x<map_expanded.cols-1; x++)
			{
				if (map_expanded.at<uchar>(y,x)==255)
				{
					bool close_to_table = false;
					for (int ky=-1; ky<=1; ky++)
						for(int kx=-1; kx<=1; kx++)
						{
							if (map_expanded.at<uchar>(y+ky,x+kx) == 128)
							{
								close_to_table = true;
								break;
							}
						}
					if (close_to_table == true)
					{
						std::cout << "true\n";
						// check if robot can approach this position
						if (isApproachPositionAccessible(robot_location, cv::Point(x,y), area_contours)==true)
						{
							geometry_msgs::Pose pose;
							Pose pose_p(x,y,0);
							Pose pose_m = convertFromPixelCoordinatesToMeter<Pose>(pose_p);
							pose.position.x = pose_m.x;
							pose.position.y = pose_m.y;
							pose.position.z = 0;

							Pose closest_point_on_polygon;
							computeClosestPointOnPolygon(map_with_polygon, pose_p, closest_point_on_polygon);

							tf::quaternionTFToMsg(tf::createQuaternionFromYaw(atan2(closest_point_on_polygon.y-pose_p.y, closest_point_on_polygon.x-pose_p.x)), pose.orientation);
							//tf::quaternionTFToMsg(tf::createQuaternionFromYaw(atan2(-dy.at<float>(y,x),-dx.at<float>(y,x))), pose.orientation);
							res.approach_poses.poses.push_back(pose);
						}

						// display found contours
//						cv::Mat map_expanded_copy = map_expanded.clone();
//						cv::drawContours(map_expanded_copy, area_contours, -1, cv::Scalar(128,128,128,128), 2);
//						cv::circle(map_expanded_copy, robot_location, 3, cv::Scalar(200,200,200,200), -1);
//						cv::circle(map_expanded_copy, cv::Point(x,y), 3, cv::Scalar(200,200,200,200), -1);
//						std::cout << " x=" << x << "  y=" << y << "\n";
//						cv::imshow("contour areas", map_expanded_copy);
//						cv::waitKey();
					}
				}
			}
		}

		return true;
	}

	template <class T>
	T convertFromMeterToPixelCoordinates(const Pose& pose)
	{
		T val;
		val.x = (pose.x-map_origin_.x)/map_resolution_;
		val.y = (pose.y-map_origin_.y)/map_resolution_;
		return val;
	}

	template <class T>
	T convertFromPixelCoordinatesToMeter(const Pose& pose)
	{
		T val;
		val.x = map_resolution_*pose.x + map_origin_.x;
		val.y = map_resolution_*pose.y + map_origin_.y;
		return val;
	}

	// this function computes whether a given point (potentialApproachPose) is accessible by the robot at location robotLocation
	bool isApproachPositionAccessible(const cv::Point& robotLocation, const cv::Point& potentialApproachPose, std::vector< std::vector<cv::Point> > contours)
	{
		// check whether potentialApproachPose and robotLocation are in the same area (=same contour)
		int contourIndexRobot = -1;
		int contourIndexPotentialApproachPose = -1;
		for (unsigned int i=0; i<contours.size(); i++)
		{
			if (0 <= cv::pointPolygonTest(contours[i], potentialApproachPose, false))
				contourIndexPotentialApproachPose = i;
			if (0 <= cv::pointPolygonTest(contours[i], robotLocation, false))
				contourIndexRobot = i;
		}
		std::cout << "contourIndexPotentialApproachPose=" << contourIndexPotentialApproachPose << "  contourIndexRobot=" << contourIndexRobot << std::endl;
		if (contourIndexRobot != contourIndexPotentialApproachPose || (contourIndexRobot==-1 && contourIndexPotentialApproachPose==-1))
			return false;

		return true;
	}

protected:

	ros::NodeHandle node_handle_;
	ros::Subscriber static_map_sub_;

	ros::ServiceServer compute_approach_pose_server_; 	// Service server providing proxemics locations

	tf::TransformListener tf_listener_;

	double robot_radius_;		// in [m]

	cv::Mat map_;
//	cv::Mat expanded_map_;
	double map_resolution_;		// in [m/cell]
	cv::Point2d map_origin_;	// in [m]
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "compute_approach_pose");
	ros::NodeHandle nh;
	
	ComputeApproachPoseNode capn;
	capn.init(nh);

	ros::spin();

	return 0;
}
