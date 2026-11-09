## Overview

This package holds a number of common configuration files for the
[move_base](https://index.ros.org/p/move_base) node on the
[Care-O-bot](care-o-bot). In particular, it holds parameter settings for
the [base_local_planner](https://index.ros.org/p/base_local_planner),
[dwa_local_planner](https://index.ros.org/p/dwa_local_planner),
[costmap_2d](https://index.ros.org/p/costmap_2d), and
[move_base](https://index.ros.org/p/move_base) components of the
[move_base](https://index.ros.org/p/move_base) node that are shared
between many different configurations of the
[navigation](https://index.ros.org/p/navigation) stack run on the
Care-O-bot.

It holds a `common` folder with a default RVIZ configuration as well as
a folder with configuration files for each of the
[different robots built at IPA](Robots/Care-O-bot/distribution).

## Configuration Files

- **`common/rviz.vcg`**: Default [rviz](https://index.ros.org/p/rviz)
  configuration for the cob_navigation stack.

<!-- -->

- robot\* is the folder of the specific robot (e.g. `cob3-2`, `raw3-1`).
- **`\*robot\*/costmap_common_params.yaml`**: Holds the common
  configuration for the [costmap's](https://index.ros.org/p/costmap_2d)
  run on the robot. Those can differ between the different robot
  versions concerning e.g. the footprint.
- **`\*robot\*/base_local_planner_params.yaml`**: Parameters for the
  [base_local_planner](https://index.ros.org/p/base_local_planner) for
  the respective robot.
- **`\*robot\*/dwa_local_planner_params.yaml`**: Parameters for the
  [dwa_local_planner](https://index.ros.org/p/dwa_local_planner) for the
  respective robot.
- **`\*robot\*/linear/ctrl_params.yaml`**: Parameters for the
  [cob_linear_nav](https://index.ros.org/p/cob_linear_nav) node.
