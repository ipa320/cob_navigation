#include <cob_map_accessibility_analysis/map_accessibility_analysis_server.h>

#define __DEBUG_DISPLAYS__

MapAccessibilityAnalysis::MapAccessibilityAnalysis(ros::NodeHandle nh)
: node_handle_(nh)
{
	// todo: read in parameters
	robot_radius_ = 0.4;
	// 	ros::param::get("/move_base/local_costmap/robot_radius", robot_radius_);
	approach_path_accessibility_check_ = true;
	map_topic_name_ = "/map";
	obstacles_topic_name_ = "/move_base/local_costmap/obstacles";
	inflated_obstacles_topic_name_ = "/move_base/local_costmap/inflated_obstacles";
	map_link_name_ = "/map";
	robot_base_link_name_ = "/base_link";

	// receive ground floor map once
	mapInit(node_handle_);

	// then set up dynamic obstacle callbacks
	inflationInit(node_handle_);

	// advertise services
	map_points_accessibility_check_server_ = node_handle_.advertiseService("map_points_accessibility_check", &MapAccessibilityAnalysis::checkPose2DArrayCallback, this);
	map_perimeter_accessibility_check_server_ = node_handle_.advertiseService("map_perimeter_accessibility_check", &MapAccessibilityAnalysis::checkPerimeterCallback, this);
	map_polygon_accessibility_check_server_ = node_handle_.advertiseService("map_polygon_accessibility_check", &MapAccessibilityAnalysis::checkPolygonCallback, this);

	ROS_INFO("MapPointAccessibilityCheck initialized.");
}

void MapAccessibilityAnalysis::mapInit(ros::NodeHandle& nh)
{
	map_data_recieved_ = false;
	map_msg_sub_ = nh.subscribe<nav_msgs::OccupancyGrid>(map_topic_name_, 1, &MapAccessibilityAnalysis::mapDataCallback, this);
	ROS_INFO("MapPointAccessibilityCheck: Waiting to receive map...");
	while (map_data_recieved_ == false)
		ros::spinOnce();
	ROS_INFO("MapPointAccessibilityCheck: Map received.");
}

void MapAccessibilityAnalysis::inflationInit(ros::NodeHandle& nh)
{
	obstacles_sub_.subscribe(nh, obstacles_topic_name_, 1);
	inflated_obstacles_sub_.subscribe(nh, inflated_obstacles_topic_name_, 1);

	inflated_obstacles_sub_sync_ = boost::shared_ptr<message_filters::Synchronizer<InflatedObstaclesSyncPolicy> >(new message_filters::Synchronizer<InflatedObstaclesSyncPolicy>(InflatedObstaclesSyncPolicy(5)));
	inflated_obstacles_sub_sync_->connectInput(obstacles_sub_, inflated_obstacles_sub_);
	inflated_obstacles_sub_sync_->registerCallback(boost::bind(&MapAccessibilityAnalysis::obstacleDataCallback, this, _1, _2));
}

void MapAccessibilityAnalysis::mapDataCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg_data)
{
	map_resolution_ = map_msg_data->info.resolution;
	inverse_map_resolution_ = 1. / map_resolution_;
	map_origin_ = cv::Point2d(map_msg_data->info.origin.position.x, map_msg_data->info.origin.position.y);
	std::cout << "map resolution: " << map_msg_data->info.resolution << std::endl;

	// create original map
	original_map_ = 255 * cv::Mat::ones(map_msg_data->info.height, map_msg_data->info.width, CV_8UC1);
	for (unsigned int v = 0, i = 0; v < map_msg_data->info.height; v++)
	{
		for (unsigned int u = 0; u < map_msg_data->info.width; u++, i++)
		{
			if (map_msg_data->data[i] != 0)
				original_map_.at<unsigned char>(v, u) = 0;
		}
	}

	// compute inflated static map
	std::cout << "inflation thickness: " << robot_radius_ << std::endl;
	cv::erode(original_map_, inflated_original_map_, cv::Mat(), cv::Point(-1,-1), cvRound(robot_radius_*inverse_map_resolution_));

	// todo: you can comment the following two lines to not pop up the inflated original map
//	cv::imshow("Inflated Original Map", inflated_original_map_);
//	cv::waitKey();

	map_data_recieved_ = true;
	map_msg_sub_.shutdown();
}

void MapAccessibilityAnalysis::obstacleDataCallback(const nav_msgs::GridCells::ConstPtr& obstacles_data, const nav_msgs::GridCells::ConstPtr& inflated_obstacles_data)
{
	{
		boost::mutex::scoped_lock lock(mutex_inflated_map_);

		inflated_map_ = inflated_original_map_.clone();
		for (unsigned int i=0; i<obstacles_data->cells.size(); ++i)
			inflated_map_.at<uchar>((obstacles_data->cells[i].y - map_origin_.y) * inverse_map_resolution_, (obstacles_data->cells[i].x - map_origin_.x) * inverse_map_resolution_) = 0;

		for (unsigned int i = 0; i < inflated_obstacles_data->cells.size(); i++)
			inflated_map_.at<uchar>((inflated_obstacles_data->cells[i].y - map_origin_.y) * inverse_map_resolution_, (inflated_obstacles_data->cells[i].x - map_origin_.x) * inverse_map_resolution_) = 0;
	}

	//TODO: you can comment the following two lines to not pop up the inflated original map
//	cv::imshow("Inflated Map", inflated_map_);
//	cv::waitKey(10);
}

bool MapAccessibilityAnalysis::checkPose2DArrayCallback(cob_map_accessibility_analysis::CheckPointAccessibility::Request &req, cob_map_accessibility_analysis::CheckPointAccessibility::Response &res)
{
	ROS_INFO("Received request to check accessibility of %i points.",req.points_to_check.size());

	res.accessibility_flags.resize(req.points_to_check.size(), true);
	{
		boost::mutex::scoped_lock lock(mutex_inflated_map_);

#ifdef __DEBUG_DISPLAYS__
		cv::Mat display_map = inflated_map_.clone();
#endif
		for (unsigned int i=0; i<req.points_to_check.size(); ++i)
		{
			if (inflated_map_.at<uchar>((req.points_to_check[i].y-map_origin_.y)*inverse_map_resolution_, (req.points_to_check[i].x-map_origin_.x)*inverse_map_resolution_) == 0)
				res.accessibility_flags[i] = false;

#ifdef __DEBUG_DISPLAYS__
			if (res.accessibility_flags[i] == false)
				cv::circle(display_map, cv::Point((req.points_to_check[i].x-map_origin_.x)*inverse_map_resolution_, (req.points_to_check[i].y-map_origin_.y)*inverse_map_resolution_), 2, cv::Scalar(64), 2);
			else
				cv::circle(display_map, cv::Point((req.points_to_check[i].x-map_origin_.x)*inverse_map_resolution_, (req.points_to_check[i].y-map_origin_.y)*inverse_map_resolution_), 2, cv::Scalar(192), 2);
#endif
		}

#ifdef __DEBUG_DISPLAYS__
		cv::imshow("points", display_map);
		cv::waitKey(10);
#endif
	}

	return true;
}

bool MapAccessibilityAnalysis::checkPerimeterCallback(cob_map_accessibility_analysis::CheckPerimeterAccessibility::Request &req, cob_map_accessibility_analysis::CheckPerimeterAccessibility::Response &res)
{
	ROS_INFO("Received request to check accessibility of point (%f,%f).",req.center.x, req.center.y);

	{
		boost::mutex::scoped_lock lock(mutex_inflated_map_);

#ifdef __DEBUG_DISPLAYS__
		cv::Mat display_map = inflated_map_.clone();
#endif
		for (double angle=req.center.theta; angle<req.center.theta+2*CV_PI; angle+=req.rotational_sampling_step)
		{
			double x = req.center.x + req.radius * cos(angle);
			double y = req.center.y + req.radius * sin(angle);
			int u = (x-map_origin_.x)*inverse_map_resolution_;
			int v = (y-map_origin_.y)*inverse_map_resolution_;
			if (inflated_map_.at<uchar>(v, u) == 255)
			{
				// add accessible point to results
				geometry_msgs::Pose2D pose;
				pose.x = x;
				pose.y = y;
				pose.theta = angle + CV_PI;
				while (pose.theta > 2*CV_PI)
					pose.theta -= 2*CV_PI;
				while (pose.theta < 0.)
					pose.theta += 2*CV_PI;
				res.accessible_poses_on_perimeter.push_back(pose);

#ifdef __DEBUG_DISPLAYS__
				cv::circle(display_map, cv::Point(u, v), 2, cv::Scalar(192), 2);
#endif
			}
		}

#ifdef __DEBUG_DISPLAYS__
		cv::imshow("perimeter", display_map);
		cv::waitKey(10);
#endif
	}

	return true;
}

bool MapAccessibilityAnalysis::checkPolygonCallback(cob_3d_mapping_msgs::GetApproachPoseForPolygon::Request& req, cob_3d_mapping_msgs::GetApproachPoseForPolygon::Response& res)
{
	// determine robot pose
	tf::StampedTransform transform;
	try
	{
		tf_listener_.waitForTransform(map_link_name_, robot_base_link_name_, ros::Time(0), ros::Duration(1));
		tf_listener_.lookupTransform(map_link_name_, robot_base_link_name_, ros::Time(0), transform);
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

	// compute inflated polygon
	cv::Mat polygon_expanded = 255 * cv::Mat::ones(original_map_.rows, original_map_.cols, original_map_.type());
	cv::drawContours(polygon_expanded, polygon_contours, -1, cv::Scalar(128), CV_FILLED);
	int iterations = cvRound(robot_radius_*inverse_map_resolution_);
	cv::erode(polygon_expanded, polygon_expanded, cv::Mat(), cv::Point(-1,-1), iterations);

	// combine inflated polygon with inflated map
	cv::Mat inflated_map;
	{
		boost::mutex::scoped_lock lock(mutex_inflated_map_);
		inflated_map = cv::min(polygon_expanded, inflated_map_);
	}
#ifdef __DEBUG_DISPLAYS__
	cv::imshow("inflated polygon map", inflated_map);
	cv::waitKey(10);
#endif

	// find the individual connected areas
	std::vector< std::vector<cv::Point> > area_contours;		// first index=contour index;  second index=point index within contour
	cv::Mat inflated_map_copy = inflated_map.clone();
	cv::findContours(inflated_map_copy, area_contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

	// iterate through all white points and consider those as potential approach poses that have an expanded table pixel in their neighborhood
	for (int y=1; y<inflated_map.rows-1; y++)
	{
		for (int x=1; x<inflated_map.cols-1; x++)
		{
			if (inflated_map.at<uchar>(y,x)==255)
			{
				bool close_to_polygon = false;
				for (int ky=-1; ky<=1; ky++)
					for(int kx=-1; kx<=1; kx++)
					{
						if (inflated_map.at<uchar>(y+ky,x+kx) == 128)
						{
							close_to_polygon = true;
							break;
						}
					}
				if (close_to_polygon == true)
				{
					// check if robot can approach this position
					if (approach_path_accessibility_check_==false || isApproachPositionAccessible(robot_location, cv::Point(x,y), area_contours)==true)
					{
						geometry_msgs::Pose pose;
						Pose pose_p(x,y,0);
						Pose pose_m = convertFromPixelCoordinatesToMeter<Pose>(pose_p);
						pose.position.x = pose_m.x;
						pose.position.y = pose_m.y;
						pose.position.z = 0;

						Pose closest_point_on_polygon;
						computeClosestPointOnPolygon(inflated_map, pose_p, closest_point_on_polygon);

						tf::quaternionTFToMsg(tf::createQuaternionFromYaw(atan2(closest_point_on_polygon.y-pose_p.y, closest_point_on_polygon.x-pose_p.x)), pose.orientation);
						//tf::quaternionTFToMsg(tf::createQuaternionFromYaw(atan2(-dy.at<float>(y,x),-dx.at<float>(y,x))), pose.orientation);
						res.approach_poses.poses.push_back(pose);
					}

#ifdef __DEBUG_DISPLAYS__
					// display found contours
					cv::Mat map_expanded_copy = inflated_map.clone();
					cv::drawContours(map_expanded_copy, area_contours, -1, cv::Scalar(128,128,128,128), 2);
					cv::circle(map_expanded_copy, robot_location, 3, cv::Scalar(200,200,200,200), -1);
					cv::circle(map_expanded_copy, cv::Point(x,y), 3, cv::Scalar(200,200,200,200), -1);
					std::cout << " x=" << x << "  y=" << y << "\n";
					cv::imshow("contour areas", map_expanded_copy);
					cv::waitKey(10);
#endif
				}
			}
		}
	}

	return true;
}

// this function computes whether a given point (potentialApproachPose) is accessible by the robot at location robotLocation
bool MapAccessibilityAnalysis::isApproachPositionAccessible(const cv::Point& robotLocation, const cv::Point& potentialApproachPose, std::vector< std::vector<cv::Point> > contours)
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


// pose_p and closest_point_on_polygon in pixel coordinates!
void MapAccessibilityAnalysis::computeClosestPointOnPolygon(const cv::Mat& map_with_polygon, const Pose& pose_p, Pose& closest_point_on_polygon)
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


template <class T>
T MapAccessibilityAnalysis::convertFromMeterToPixelCoordinates(const Pose& pose)
{
	T val;
	val.x = (pose.x-map_origin_.x)*inverse_map_resolution_;
	val.y = (pose.y-map_origin_.y)*inverse_map_resolution_;
	return val;
}

template <class T>
T MapAccessibilityAnalysis::convertFromPixelCoordinatesToMeter(const Pose& pose)
{
	T val;
	val.x = map_resolution_*pose.x + map_origin_.x;
	val.y = map_resolution_*pose.y + map_origin_.y;
	return val;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "map_accessibility_analysis_server");

	ros::NodeHandle nh;

	MapAccessibilityAnalysis map_accessibility_analysis(nh);

	ros::spin();

	return 0;
}

