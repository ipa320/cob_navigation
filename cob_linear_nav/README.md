The cob_linear_nav package provides a
configurable node for simple navigation. It drives the robot on a linear
path from its current position to its goal position **without any
obstacle avoidance**. This should be carried out in another package. If
the robot doesn't move anymore (e.g. due to an obstacle that stays on
the robot path), the robot stops trying to reach the goal after a
certain period of time (to be specified over a parameter).

## ROS API

### cob_linear_nav

The `cob_linear_nav` node takes in
[geometry_msgs/PoseStamped](http://ros.org/doc/api/geometry_msgs/html/msg/PoseStamped.html)
messages as a new goal, as well as
[nav_msgs/Odometry](http://ros.org/doc/api/nav_msgs/html/msg/Odometry.html)
messages specifying the odometry of the robot. The movement commands are
published as a
[geometry_msgs/Twist](http://ros.org/doc/api/geometry_msgs/html/msg/Twist.html)
[topic](Topics). It also provides the ability to send a goal using
[rviz](https://index.ros.org/p/rviz) on the
*/move_base_linear_simple/goal* topic.

#### Subscribed Topics

- `/move_baser_linear_simple/goal`
  ([geometry_msgs/msg/PoseStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/PoseStamped.html)):
  new goal for the robot
- `odom`
  ([nav_msgs/msg/Odometry](https://docs.ros.org/en/rolling/p/nav_msgs/msg/Odometry.html)):
  odometry of the robot

#### Published Topics

- `cmd_vel`
  ([geometry_msgs/msg/Twist](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/Twist.html)):
  movement command to robot base

#### Parameters

- `~kv` (`double`, default: `2.5`): Damping term for potential field
  like controller for translational movement.
- `~kp` (`double`, default: `3.0`): Stiffness term for potential field
  like controller for translational movement.
- `~virt_mass` (`double`, default: `0.5`): Virtual mass term for
  potential field like controller for translational movement.
- `~vmax` (`double`, default: `0.3`): Maximum allowed velocity for
  translational movement.
- `~goal_threshold` (`double`, default: `0.03 [m]`): Distance to goal at
  which robot considers to have reached the goal (translational movement
  only).
- `~speed_threshold` (`double`, default: `0.08`): Speed at which the
  robot considers to have stopped (translational movement only).
- `~kv_rot` (`double`, default: `2.5`): Damping term for potential field
  like controller for rotational movement.
- `~kp_rot` (`double`, default: `3.0`): Stiffness term for potential
  field like controller for rotational movement.
- `~virt_mass_rot` (`double`, default: `0.5`): Virtual mass term for
  potential field like controller for rotational movement.
- `~vtheta_max` (`double`, default: `0.3`): Maximum allowed rotational
  velocity for movement.
- `~goal_threshold_rot` (`double`, default: `0.08`): Distance to goal at
  which robot considers to have reached the goal (rotational movement
  only).
- `~speed_threshold_rot` (`double`, default: `0.08`): Speed at which the
  robot considers to have stopped (rotational movement only).
- `~global_frame` (`string`, default: `"/map"`): Map frame.
- `~robot_frame` (`string`, default: `"/base_link"`): Frame of robot
  which is considered to be the frame that should reach the goal
  position.
- `~slow_down_distance` (`double`, default: `0.5 [m]`): Distance to goal
  at which the robot starts slowing down.
- `~goal_abortion_time` (`double`, default: `5.0 [s]`): Time after which
  the robot stops trying to reach the goal, if it does not move anymore
  (as indicated by the odometry). This happens when the movement is
  stopped by an external collision avoidance function due to e.g.
  obstacles on the linear path.

## Usage/Examples

For the usage of cob_linear_nav, have a
look at
[cob_navigation_global](https://index.ros.org/p/cob_navigation_global)
and the `2dnav_linear.launch` and `2dnav_linear.xml` files therein.

A sample parameter file (currently in
[cob_navigation_config](https://index.ros.org/p/cob_navigation_config)`/common/linear/ctrl_params.yaml`)
can look like this: 

```
#Parameters designing the potential field controller
#Translational part
kv: 2.50 #damping 2.50
kp: 3.00 #stiffness 3.0
virt_mass: 0.50 #0.50
vmax: 0.3 #0.3
goal_threshold: 0.03 #0.03
speed_threshold: 0.08 #0.08

#Rotational part
kv_rot: 2.0 #damping 2.0
kp_rot: 2.0 #stiffness 2.0
virt_mass_rot: 0.50 #0.5
vtheta_max: 0.3 # 0.3
goal_threshold_rot: 0.08 #0.08
speed_threshold_rot: 0.08 #0.08

#tf parameters
global_frame: /map #/map
robot_frame: /base_link #/base_link

slow_down_distance: 0.5 #0.5[m] distance from robot_center to goal at which robot starts slowing down
goal_abortion_time: 5.0 #5.0[s] time until robot aborts goal due to obstacle
```
