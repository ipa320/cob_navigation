## Overview

This package contains launch files for the
[gmapping](https://index.ros.org/p/gmapping) node for SLAM without
navigation to create a map.

## Applications

- **`launch/2dslam.launch`**: Starts the gmapping node.

## Building Blocks

- **`launch/2dslam.xml`**: Holds a specific configuration of the
  [gmapping](https://index.ros.org/p/gmapping) node. Designed to be
  included by applications that wish to run
  [gmapping](https://index.ros.org/p/gmapping) on the
  [Care-O-bot](care-o-bot).

## Tutorials

see [Tutorials](#tutorials-1)

## Tutorials

- [Creating a map](#creating-a-map):
  This tutorials shows how to build a map in a new environment

## Creating a map

- **Description:** This tutorials shows how to build a map in a new
  environment
- **Tutorial Level:** INTERMEDIATE

### Prerequisists

- Prepare environment to be "robot capable" (no chairs, \...)
- Start up the bringup layer, see
  [cob_bringup](https://index.ros.org/p/cob_bringup).

Note: The origin of the map will be where you initialise the base.

### Start the mapping process



```
roslaunch cob_mapping_slam 2dslam.launch
```



- drive around with joystick
- keep dynamic obstacles (people) out of front scan
- while driving let the front scan see wide parts of the already created
  map

### Safe map

To safe the map run 

```
rosrun map_server map_saver
```

 in a **new** terminal while
the mapping process is **still running**. Otherwise your map will be
lost. Then copy `map.yaml` and `map.pgm` to your scenario or environment
package.
