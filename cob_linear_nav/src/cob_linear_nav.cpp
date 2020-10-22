/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 

//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <ros/ros.h>
#include <angles/angles.h>

#include <pthread.h>

// ROS message includes
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <cob_srvs/SetString.h>

#include <tf/transform_listener.h>

//####################
//#### node class ####
class NodeClass
{
  public:
  // create a handle for this node, initialize node
  ros::NodeHandle nh_;

  // declaration of topics
  ros::Publisher topic_pub_command_;

  ros::Subscriber goal_sub_, odometry_sub_;

  ros::ServiceServer ss_;

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

    ros::NodeHandle private_nh("~");

    // implementation of topics to publish
    topic_pub_command_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    //we'll provide a mechanism for some people to send goals as PoseStamped messages over a topic
    //they won't get any useful information back about its status, but this is useful for tools
    //like nav_view and rviz
    ros::NodeHandle simple_nh("move_base_linear_simple");
    goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&NodeClass::topicCB, this, _1));

    action_client_ = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(nh_, name);

    // subscribe to odometry
    odometry_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&NodeClass::odometryCB, this, _1));

    ss_ = private_nh.advertiseService("set_global_frame", &NodeClass::serviceCB, this);

    // read parameters from parameter server
    if(!private_nh.hasParam("kv")) ROS_WARN("Used default parameter for kv [2.5]");
    private_nh.param("kv", kv_, 2.5);

    if(!private_nh.hasParam("kp")) ROS_WARN("Used default parameter for kp [3.0]");
    private_nh.param("kp", kp_, 3.0);

    if(!private_nh.hasParam("virt_mass")) ROS_WARN("Used default parameter for virt_mass [0.5]");
    private_nh.param("virt_mass", virt_mass_, 0.5);

    if(!private_nh.hasParam("vmax")) ROS_WARN("Used default parameter for vmax [0.3 m/s]");
    private_nh.param("vmax", v_max_, 0.3);

    if(!private_nh.hasParam("goal_threshold")) ROS_WARN("Used default parameter for goal_threshold [0.03 cm]");
    private_nh.param("goal_threshold", goal_threshold_, 0.03);

    if(!private_nh.hasParam("speed_threshold")) ROS_WARN("Used default parameter for speed_threshold [0.08 m/s]");
    private_nh.param("speed_threshold", speed_threshold_, 0.08);

    if(!private_nh.hasParam("kv_rot")) ROS_WARN("Used default parameter for kv_rot [2.0]");
    private_nh.param("kv_rot", kv_rot_, 2.0);

    if(!private_nh.hasParam("kp_rot")) ROS_WARN("Used default parameter for kp_rot [2.0]");
    private_nh.param("kp_rot", kp_rot_, 2.0);

    if(!private_nh.hasParam("virt_mass_rot")) ROS_WARN("Used default parameter for virt_mass_rot [0.5]");
    private_nh.param("virt_mass_rot", virt_mass_rot_, 0.5);

    if(!private_nh.hasParam("vtheta_max")) ROS_WARN("Used default parameter for vtheta_max [0.3 rad/s]");
    private_nh.param("vtheta_max", vtheta_max_, 0.3);

    if(!private_nh.hasParam("goal_threshold_rot")) ROS_WARN("Used default parameter for goal_threshold_rot [0.08 rad]");
    private_nh.param("goal_threshold_rot", goal_threshold_rot_, 0.08);

    if(!private_nh.hasParam("speed_threshold_rot")) ROS_WARN("Used default parameter for speed_threshold_rot [0.08 rad/s]");
    private_nh.param("speed_threshold_rot", speed_threshold_rot_, 0.08);

    if(!private_nh.hasParam("goal_abortion_speed")) ROS_WARN("Used default parameter for goal_abortion_speed [0.01 m/s]");
    private_nh.param("goal_abortion_speed", goal_abortion_speed_, 0.01);

    if(!private_nh.hasParam("goal_abortion_speed_rot")) ROS_WARN("Used default parameter for goal_abortion_speed_rot [0.01 rad/s]");
    private_nh.param("goal_abortion_speed_rot", goal_abortion_speed_rot_, 0.01);

    if(!private_nh.hasParam("global_frame")) ROS_WARN("Used default parameter for global_frame [/map]");
    private_nh.param("global_frame", global_frame_, std::string("map"));

    if(!private_nh.hasParam("robot_frame")) ROS_WARN("Used default parameter for robot_frame [/base_link]");
    private_nh.param("robot_frame", robot_frame_, std::string("base_link"));

    if(!private_nh.hasParam("robot_footprint_frame")) ROS_WARN("Used default parameter for robot_footprint_frame [/base_footprint]");
    private_nh.param("robot_footprint_frame", robot_footprint_frame_, std::string("base_footprint"));

    if(!private_nh.hasParam("slow_down_distance")) ROS_WARN("Used default parameter for slow_down_distance [0.5m]");
    private_nh.param("slow_down_distance", slow_down_distance_, 0.5);

    if(!private_nh.hasParam("goal_abortion_time")) ROS_WARN("Used default parameter for goal_abortion_time [5.0s]");
    private_nh.param("goal_abortion_time", goal_abortion_time_, 5.0);

    if(!private_nh.hasParam("use_move_action")) ROS_WARN("Used default parameter for use_move_action [true]");
    private_nh.param("use_move_action", use_move_action_, true);

    //generate robot zero_pose
    zero_pose_.pose.position.x = 0.0;
    zero_pose_.pose.position.y = 0.0;
    zero_pose_.pose.position.z = 0.0;
    zero_pose_.pose.orientation.x = 0.0;
    zero_pose_.pose.orientation.y = 0.0;
    zero_pose_.pose.orientation.z = 0.0;
    zero_pose_.pose.orientation.w = 1.0;
    zero_pose_.header.frame_id = robot_frame_;
    zero_pose_.header.stamp = ros::Time::now();
    robot_pose_global_ = zero_pose_;
    robot_pose_global_.header.frame_id = global_frame_;
    goal_pose_global_ = robot_pose_global_;
    // at startup, the robot is should not move
    move_ = false;

    // also initialize variables that are used later on!
    last_time_ = ros::Time::now().toSec();
    vtheta_last_ = 0.0;

    //start action server, it holds the main loop while driving
    as_.start();
  }

  void topicCB(const geometry_msgs::PoseStamped::ConstPtr& goal)
  {

    if(!goalValid(*goal))
      return;

    if(use_move_action_)
    {
      ROS_INFO("In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
      move_base_msgs::MoveBaseGoal action_goal;

      action_goal.target_pose = transformGoalToMap(*goal);

      action_client_->sendGoal(action_goal);
      action_client_->stopTrackingGoal();
    }
    else
    {
      ROS_DEBUG("In ROS goal callback, using the PoseStamped as error and start control step.");

      last_time_moving_ = ros::Time::now().toSec();

      getRobotPoseGlobal();

      if(last_time_ < 0)
      {
        vtheta_last_ = 0.0f;
        vx_last_ = 0.0f;
        vy_last_ = 0.0f;
        last_time_ = ros::Time::now().toSec();
      }

      x_last_ = robot_pose_global_.pose.position.x;
      y_last_ = robot_pose_global_.pose.position.y;
      theta_last_ = tf::getYaw(robot_pose_global_.pose.orientation);

      goal_pose_global_ = transformGoalToMap(*goal);

      move_ = true;

    }

  }

  void actionCB(const move_base_msgs::MoveBaseGoalConstPtr &goal)
  {
    if(!goalValid(goal->target_pose))
    {
      as_.setAborted(move_base_msgs::MoveBaseResult(), "Aborting because a transformation could not be found");
      return;
    }
    // goal is of type geometry_msgs/PoseStamped
    ROS_INFO("In idle mode, new goal accepted");

    last_time_moving_ = ros::Time::now().toSec();

    getRobotPoseGlobal();
    x_last_ = robot_pose_global_.pose.position.x;
    y_last_ = robot_pose_global_.pose.position.y;
    theta_last_ = tf::getYaw(robot_pose_global_.pose.orientation);
    vtheta_last_ = 0.0f;
    vx_last_ = 0.0f;
    vy_last_ = 0.0f;
    last_time_ = ros::Time::now().toSec();

    goal_pose_global_ = transformGoalToMap(goal->target_pose);

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

          last_time_moving_ = ros::Time::now().toSec();
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

      performControllerStep();

      if(finished_) {
        as_.setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
        ROS_INFO("Goal reached.");
        return;
      }

      if(!as_.isActive()) {
        ROS_INFO("Goal not active anymore. Stop!");
        return;
      }
    }


    //if the node is killed then we'll abort and return
    stopMovement();
    as_.setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
    return;
  };


  void odometryCB(const nav_msgs::Odometry::ConstPtr &odometry){
    geometry_msgs::Vector3Stamped vec_stamped;

    vec_stamped.vector = odometry->twist.twist.linear;
    vec_stamped.header.frame_id =  robot_footprint_frame_;
    try
    {
      tf_listener_.waitForTransform(robot_frame_, vec_stamped.header.frame_id, ros::Time(0), ros::Duration(1.0));
      tf_listener_.transformVector(robot_frame_, vec_stamped, robot_twist_linear_robot_);
    }
    catch(tf::TransformException& ex){ROS_ERROR("%s",ex.what());}

    vec_stamped.vector = odometry->twist.twist.angular;
    vec_stamped.header.frame_id =  robot_footprint_frame_;
    try
    {
      tf_listener_.waitForTransform(robot_frame_, vec_stamped.header.frame_id, ros::Time(0), ros::Duration(1.0));
      tf_listener_.transformVector(robot_frame_, vec_stamped, robot_twist_angular_robot_);
    }
    catch(tf::TransformException& ex){ROS_ERROR("%s",ex.what());}
  }

  bool serviceCB(cob_srvs::SetString::Request &req, cob_srvs::SetString::Response &res)
  {
    global_frame_ = req.data;
    ros::NodeHandle private_nh("~");
    private_nh.setParam("global_frame", req.data);
    ROS_INFO_STREAM("Set global frame to: " << req.data);
    res.success = true;
    res.message = "";

    return true;
  }

  // Destructor
  ~NodeClass()
  {
  }

  void performControllerStep();
  bool getUseMoveAction(void);

private:
  tf::TransformListener tf_listener_;
  std::string global_frame_, robot_frame_, robot_footprint_frame_;
  geometry_msgs::PoseStamped goal_pose_global_;
  geometry_msgs::PoseStamped zero_pose_;
  geometry_msgs::PoseStamped robot_pose_global_;
  geometry_msgs::Vector3Stamped robot_twist_linear_robot_, robot_twist_angular_robot_;

  double slow_down_distance_, goal_abortion_time_;

  bool finished_, move_;

  bool use_move_action_;

  pthread_mutex_t m_mutex;

  //core functions:
  void publishVelocitiesGlobal(double vx, double vy, double theta);
  geometry_msgs::PoseStamped transformGoalToMap(geometry_msgs::PoseStamped goal_pose);
  geometry_msgs::PoseStamped getRobotPoseGlobal();

  //helper functions:
  double getDistance2d(geometry_msgs::PoseStamped a, geometry_msgs::PoseStamped b);
  double getDistance2d(geometry_msgs::Point a, geometry_msgs::Point b);
  double getThetaDiffRad(double target, double actual);
  double sign(double x);
  void stopMovement();
  bool notMovingDueToObstacle();
  bool goalValid(const geometry_msgs::PoseStamped& goal_pose);

  //Potential field Controller variables
  double vx_last_, vy_last_, x_last_, y_last_, theta_last_, vtheta_last_;
  double goal_threshold_, speed_threshold_, goal_abortion_speed_;
  double goal_threshold_rot_, speed_threshold_rot_, goal_abortion_speed_rot_;
  double kp_, kv_, virt_mass_;
  double kp_rot_, kv_rot_, virt_mass_rot_;
  double last_time_;
  double v_max_, vtheta_max_;
  double last_time_moving_;

}; //NodeClass

bool NodeClass::getUseMoveAction(void)
{
  return use_move_action_;
}

geometry_msgs::PoseStamped NodeClass::transformGoalToMap(geometry_msgs::PoseStamped goal_pose) {
  geometry_msgs::PoseStamped goal_global_;
  if(goal_pose.header.frame_id == global_frame_) return goal_pose;
  else if(tf_listener_.canTransform(global_frame_, goal_pose.header.frame_id, ros::Time(0), new std::string)) {
    tf_listener_.transformPose(global_frame_, ros::Time(0), goal_pose, "base_link", goal_global_);
    return goal_global_;
  } else {
    ROS_WARN("Can't transform goal to global frame %s", global_frame_.c_str());
    return robot_pose_global_;
  }
}

geometry_msgs::PoseStamped NodeClass::getRobotPoseGlobal() {
  try
  {
    ros::Time now = ros::Time::now();
    tf_listener_.waitForTransform(global_frame_, robot_frame_, now, ros::Duration(5.0));
    tf_listener_.transformPose(global_frame_, now, zero_pose_, robot_frame_, robot_pose_global_);
  }
  catch(tf::TransformException& ex){
    ROS_WARN("Failed to find robot pose in global frame %s", global_frame_.c_str());
    robot_pose_global_ = zero_pose_;
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
  return angles::shortest_angular_distance(actual, target);
}

void NodeClass::publishVelocitiesGlobal(double vx, double vy, double theta) {
  //Transform commands from global frame to robot coordinate system
  geometry_msgs::Vector3Stamped cmd_global, cmd_robot;
  geometry_msgs::Twist msg;

  cmd_global.header.frame_id = global_frame_;
  cmd_global.header.stamp = ros::Time::now();
  cmd_global.vector.x = vx;
  cmd_global.vector.y = vy;
  try
  {
    tf_listener_.waitForTransform(robot_frame_, cmd_global.header.frame_id, cmd_global.header.stamp, ros::Duration(1.0));
    tf_listener_.transformVector(robot_frame_, cmd_global, cmd_robot);
  } catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    cmd_robot.vector.x = 0.0f;
    cmd_robot.vector.y = 0.0f;
  }

  // make sure that the published velocities are in an at least somewhat reasonable range
  try
  {
    if ( std::isnan(cmd_robot.vector.x) || std::isnan(cmd_robot.vector.y) || std::isnan(theta) )
    {
      std::string err = "linear_nav: Output velocity contains NaN!";
      throw err;
    }
    if ( ! ( fabs(cmd_robot.vector.x) < 5.0 && fabs(cmd_robot.vector.y) < 5.0 && fabs(theta) < M_PI ) )
    {
      std::string err = "linear_nav: Output velocity too high (x="+std::to_string(cmd_robot.vector.x)+" y="+std::to_string(cmd_robot.vector.y)+" theta="+std::to_string(theta)+")";
      throw err;
    }
  }
  catch ( std::string err )
  {
    ROS_ERROR("%s", err.c_str());
    topic_pub_command_.publish(geometry_msgs::Twist());
    return;
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

bool NodeClass::notMovingDueToObstacle() {
  if (move_ == true && // should move
      finished_ == false && // has not reached goal
      fabs(robot_twist_linear_robot_.vector.x) <= goal_abortion_speed_ && // velocity components are small
      fabs(robot_twist_linear_robot_.vector.y) <= goal_abortion_speed_ &&
      fabs(robot_twist_angular_robot_.vector.z) <= goal_abortion_speed_rot_ &&
      ros::Time::now().toSec() - last_time_moving_ > goal_abortion_time_ ) // has not been moving for x seconds
  {
    return true;
  } else if ( fabs(robot_twist_linear_robot_.vector.x) > goal_abortion_speed_ ||
              fabs(robot_twist_linear_robot_.vector.y) > goal_abortion_speed_ ||
              fabs(robot_twist_angular_robot_.vector.z) > goal_abortion_speed_rot_ )
  { // still moving, then update last_time_moving_
    last_time_moving_ = ros::Time::now().toSec();
  }

  return false;
}

bool NodeClass::goalValid(const geometry_msgs::PoseStamped& goal_pose)
{
  if( goal_pose.pose.orientation.x == 0.0 &&
      goal_pose.pose.orientation.y == 0.0 &&
      goal_pose.pose.orientation.z == 0.0 &&
      goal_pose.pose.orientation.w == 0.0 )
  {
    ROS_WARN("Goal invalid! Received Quaternion with all values 0.0!");
    return false;
  }
  else if (!tf_listener_.waitForTransform(global_frame_, goal_pose.header.frame_id, goal_pose.header.stamp, ros::Duration(1.0)))
  {
    ROS_WARN_STREAM("Can not transform goal which is given in /"
                    << goal_pose.header.frame_id << " into global frame /" << global_frame_);
    return false;
  }
  else if (!tf_listener_.waitForTransform(robot_frame_, goal_pose.header.frame_id, goal_pose.header.stamp, ros::Duration(1.0)))
  {
     ROS_WARN_STREAM("Can not transform goal which is given in /"
                      << goal_pose.header.frame_id << " into robot frame /" << robot_frame_);
     return false;
  }
  else
  {
    return true;
  }
}

void NodeClass::performControllerStep() {
  pthread_mutex_lock(&m_mutex);
  ROS_DEBUG_STREAM_NAMED("mutex", "performControllerStep: locked mutex");

  double dt;
  double F_x, F_y, F_theta;
  double distance_to_goal;
  double theta, theta_goal;
  double cmd_vx, cmd_vy, cmd_vtheta;
  double vx_d, vy_d, vtheta_d, v_factor;
  double v_max_goal = v_max_;

  if(!move_) {
    if(!use_move_action_)
      last_time_ = ros::Time::now().toSec();
    pthread_mutex_unlock(&m_mutex);
    ROS_DEBUG_STREAM_NAMED("mutex", "performControllerStep: released mutex");
    return;
  }

  getRobotPoseGlobal();

  distance_to_goal = getDistance2d(robot_pose_global_, goal_pose_global_);
  theta = tf::getYaw(robot_pose_global_.pose.orientation);
  theta_goal = tf::getYaw(goal_pose_global_.pose.orientation);

  //exit, if positions and velocities lie inside thresholds
  if( distance_to_goal <= goal_threshold_ &&
      sqrt(vx_last_ * vx_last_ + vy_last_ * vy_last_) <= speed_threshold_ &&
      fabs(getThetaDiffRad(theta_goal, theta)) <= goal_threshold_rot_ &&
      fabs(vtheta_last_) <= speed_threshold_rot_ )
  {
    finished_ = true;
    move_ = false;
    stopMovement();
    if(!use_move_action_)
      last_time_ = ros::Time::now().toSec();
    pthread_mutex_unlock(&m_mutex);
    ROS_DEBUG_STREAM_NAMED("mutex", "performControllerStep: released mutex");
    return;
  } else if( notMovingDueToObstacle() == true ) {
    finished_ = false;
    move_ = false;
    stopMovement();
    if(use_move_action_)
    {
      as_.setAborted(move_base_msgs::MoveBaseResult(), "Cancel the goal because an obstacle is blocking the path.");
    }
    else
    {
      last_time_ = ros::Time::now().toSec();
    }
    ROS_INFO("Cancel the goal because an obstacle is blocking the path.");
    pthread_mutex_unlock(&m_mutex);
    ROS_DEBUG_STREAM_NAMED("mutex", "performControllerStep: released mutex");
    return;
  } else finished_ = false;

  dt = ros::Time::now().toSec() - last_time_;
  last_time_ = ros::Time::now().toSec();

  //Slow down while approaching goal
  if(distance_to_goal < slow_down_distance_) {
    //implementation for linear decrease of v_max:
    double goal_linear_slope = v_max_ / slow_down_distance_;
    v_max_goal = distance_to_goal * goal_linear_slope;

    if(v_max_goal > v_max_) v_max_goal = v_max_;
      else if(v_max_goal < 0.0f) v_max_goal = 0.0f;
  }

  //Translational movement
  //calculation of v factor to limit maxspeed
  vx_d = kp_/kv_ * (goal_pose_global_.pose.position.x - robot_pose_global_.pose.position.x);
  vy_d = kp_/kv_ * (goal_pose_global_.pose.position.y - robot_pose_global_.pose.position.y);
  v_factor = v_max_goal / sqrt(vy_d*vy_d + vx_d*vx_d);

  if(v_factor > 1.0) v_factor = 1.0;

  F_x = - kv_ * vx_last_ + v_factor * kp_ * (goal_pose_global_.pose.position.x - robot_pose_global_.pose.position.x);
  F_y = - kv_ * vy_last_ + v_factor * kp_ * (goal_pose_global_.pose.position.y - robot_pose_global_.pose.position.y);

  cmd_vx = vx_last_ + F_x / virt_mass_ * dt;
  cmd_vy = vy_last_ + F_y / virt_mass_ * dt;

  //Rotational Movement
  //calculation of v factor to limit maxspeed
  vtheta_d = kp_rot_ / kv_rot_ * getThetaDiffRad(theta_goal, theta);
  v_factor = fabs(vtheta_max_ / vtheta_d);
  if(v_factor > 1.0) v_factor = 1.0;

  F_theta = - kv_rot_ * vtheta_last_ + v_factor * kp_rot_ * getThetaDiffRad(theta_goal, theta);
  cmd_vtheta = vtheta_last_ + F_theta / virt_mass_rot_ * dt;

  //Publish velocities, these calculated forces and velocities are for the global frame
  //they are transformed to robot_frame later
  x_last_ = robot_pose_global_.pose.position.x;
  y_last_ = robot_pose_global_.pose.position.y;
  theta_last_ = theta;
  vx_last_ = cmd_vx;
  vy_last_ = cmd_vy;
  vtheta_last_ = cmd_vtheta;

  publishVelocitiesGlobal(cmd_vx, cmd_vy, cmd_vtheta);
  pthread_mutex_unlock(&m_mutex);
  ROS_DEBUG_STREAM_NAMED("mutex", "performControllerStep: released mutex");
}



//#######################
//#### main programm ####
int main(int argc, char** argv)
{
  // initialize ROS, spezify name of node
  ros::init(argc, argv, "cob_linear");

  // create nodeClass
  NodeClass nodeClass("move_base_linear");

  if(nodeClass.getUseMoveAction())
  {
    ros::spin();
  }
  else
  {
    ros::Rate loop_rate(100);
    while(nodeClass.nh_.ok())
    {
      nodeClass.performControllerStep();
      loop_rate.sleep();
      ros::spinOnce();
    }
  }

  return 0;
}

