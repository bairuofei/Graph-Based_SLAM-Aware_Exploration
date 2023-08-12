
# Graph-based SLAM-Aware Exploration
Source code for the paper "*Graph-based SLAM-Aware Exploration with Prior Topo-Metric Information*".
It includes a ROS plugin for the `nav2d` package for autonomous exploration using the prior topo-metric graph of the environment.

<div style="display:flex; justify-content:center;">
<figure>
    <img src="./images/frontier.gif" alt="Alt Text" width="400" height="400">
    <!-- <figcaption style="text-align:center;">Frontier-based Method</figcaption> -->
</figure>
<figure>
    <img src="./images/active_tsp.gif" alt="Alt Text" width="400" height="400">
    <!-- <figcaption style="text-align:center;">Active TSP-based Method</figcaption> -->
</figure>
</div>

## Requirements

The code has been developed and tested on Ubuntu 20.04, ROS noetic.

Install required:
- [Eigen](https://eigen.tuxfamily.org/dox/GettingStarted.html) REQUIRED
- ROS noetic
- [pyconcorde](https://github.com/jvkersch/pyconcorde) (Traveling Salesman Problem Solver, detailed usage refers to [here](https://github.com/jvkersch/pyconcorde/issues/28))
- [p2os_urdf](https://github.com/allenh1/p2os) (Robot model)

No need to install:
- [Nav2d](http://wiki.ros.org/nav2d) (has been included in this repo, no need to install)
- [astar-gridmap-2d](https://github.com/Eurecat/astar-gridmap-2d) (has been included in this repo, no need to install)



## Usage

### Quick start
Open terminal 1
```
$ cd Graph-Based_SLAM-Aware_Exploration/
$ catkin_make
$ source ./devel/setup.bash

$ roslaunch cpp_solver exploration.launch
```
Open anthoer terminal 2
```
$ cd Graph-Based_SLAM-Aware_Exploration/
$ source ./devel/setup.bash

$ rosservice call /StartMapping
$ rosservice call /StartExploration
```
The robot will start to exploration the default environment.

### Change exploration strategy



## Explore a new environment

#### Create a 2D environment
1. Create a `my_environment.png` file representing the top view of the environment;
2. Define `my_environment.world` file to be used in Stage.
    1. Set *bitmap* parameter as  `my_environment.png` 
    1. The actual size of the map should be set, which actually defines `meter / pixel`.
    2. The position will the map **center** be placed in Stage simulator. 
    3. The initial position of the robot in the Stage simulator.
3. Define `my_environment.yaml` file to be used in map server to provide ground truth map for visualization.
    1. Define the position of the left-bottom pixel of the map. 
    To keep the map published by map_server aligned with SLAM map, here the position should be set by: `[-0.5 * map_size_x - robot_x, -0.5 * map_size_y - robot_y, 0]`.
#### Prepare prior topo-metric graph of the 2D environment
4. Use [draw.io](https://app.diagrams.net/) to draw a topo-metric graph of the environment. 
    1. Only use `circle` element to represent vertices, and use `line` to connect these circles with each other in [draw.io](https://app.diagrams.net/), which represents edges in the prior graph.
    2. Put three isolated `circle` in the graph: the left bottom corner of the environment, the right bottom corner of the environment and the top boundary of the environment.
    3. Select the above graph and export the selected region into a `my_environment.xml` file.

#### Specify the new environment in launch file





4. In ROS launch file, set the `.yaml` file for `map_server` node, set the `.world` file for `Stage` node.

5. In ROS launch file, set the `robot_position` parameter as in `.world` file in Step 2. Two node will use this parameter:
    1. In `path_planner.py`, define prior_map for the new map. The positions of vertices in prior_map are defined in a coordinate frame with its origin being the center of map. And then call the function `self.align_prior_map_with_robot()`, redefine the position of vertices in prior_map in robot's coordinate frame.

    2. In `pubPathGT.cpp`, use `robot_position` parameter to corrent the `odom` coordinate.

6. In ROS launch file, set the `use_drawio` parameter to `true / false`.

7. In ROS launch file, set the `map_width` parameter to the map actual width as in `.world` file in Step 2.





###

#### Quick Check
The following parameters in `.launch` file should be carefully defined.

```xml
	<arg name="suffix" default="_7_06_frontier3"/>
	<arg name="strategy" default="MyPlanner" />
	<arg name="map_name" default="map3/grid_world" />
	<arg name="robot_position" default="-28 0 0" />
	<arg name="use_drawio" default="true" />
	<arg name="map_width" default="74" />
```


#### Detailed steps




