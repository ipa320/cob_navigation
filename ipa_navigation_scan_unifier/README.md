cob_unified scan publisher
====================

General description
---------------------
This package implements a node that unifies scan messages from a given numer of laser scanners

Node: cob\_unified\_scan\_publisher\_node
---------------------

The actual node that unifies a given number of laser scans
#### Parameters
**number\_scans** *(int, default: 2)*   
 The number of scans to be unified.
 
**loop\_rate** *(double, default: 100.0 [hz])*   
 The loop rate of the ros node.

**start\_delay** *(double, default: 2 [sec])*   
 A time delay in seconds. The node will start publishing messages at the earliest after this time delay (to avoid tf-bug).

#### Published Topics
**scan\_unified** *(sensor_msgs::LaserScan)*   
 Publishes the unified scans.

#### Subscribed Topics
**scan\_i\_in** *(sensor_msgs::LaserScan)*   
 The current scan message from the i-th laser scanner


#### Services


#### Services called
