
# Graph-based SLAM-Aware Exploration (will release soon)
Source code for the paper "*Graph-based SLAM-Aware Exploration with Prior Topo-Metric Information*".
The paper proposes a SLAM-aware exploration method that exploits prior topo-metric information of the environment for fast exploration and enhanced pose graph reliability in the SLAM process.
It works as an exploration plugin to the [nav2d](http://wiki.ros.org/nav2d) package for autonomous exploration .

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

Frontier-based method (Left)  V.S.   the SLAM-aware exploration (right)

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

Three types of exploration strategies are provided. You can simply change to one of them by specifying in the roslaunch file.
- frontier-based (no need for prior graph)
```xml
<arg name="strategy" default="NearestFrontierPlanner" />
```
- TSP-based (prior graph required)
```xml
<arg name="strategy" default="MyPlanner" />
<arg name="only_use_tsp" default="true" />
```
- the SLAM-Aware path planner (prior graph required)
```xml
<arg name="strategy" default="MyPlanner" />
<arg name="only_use_tsp" default="false" />
```







## Explore a new environment

By default, we provide several environment models under the `world/` folder. 
You can set your own environment by the following steps.

#### Create new directory
1. Create a new folder named `my_environment` under the `world` folder. 
You are recommended to copy from an existing folder and then modify each file.
The files should be organized as follow:
```
my_environment
├── floorplan.inc
├── hokuyo.inc
├── p3at.inc
├── my_environment.png         // See step 2
├── my_environment.world       // See step 3
├── my_environment.yaml        // See step 4
├── my_environment.drawio      // See step 5
└── my_environment.xml         // See step 5
```


#### Create a 2D environment
2. Create a `my_environment.png` file representing the top view of the environment;
3. Define `my_environment.world` file to be used in Stage.
    1. Set *bitmap* parameter as  `my_environment.png` 
    1. The actual size of the map should be set, which actually defines `meter / pixel`.
    2. The position will the map **center** be placed in Stage simulator. 
    3. The initial position of the robot in the Stage simulator.
4. Define `my_environment.yaml` file to be used in map server to provide ground truth map for visualization.
    1. Define the position of the left-bottom pixel of the map. 
    To keep the map published by map_server aligned with SLAM map, here the position should be set by: `[-0.5 * map_size_x - robot_x, -0.5 * map_size_y - robot_y, 0]`.
#### Prepare prior topo-metric graph of the 2D environment
5. Use [draw.io](https://app.diagrams.net/) to draw a topo-metric graph of the environment. 
    1. Only use `circle` element to represent vertices, and use `line` to connect these circles with each other in [draw.io](https://app.diagrams.net/), which represents edges in the prior graph.
    2. Put three isolated `circle` in the graph: the left bottom corner of the environment, the right bottom corner of the environment and the top boundary of the environment.
    3. Select the above graph and export the selected region into a `my_environment.xml` file.

#### Specify the new environment in launch file

6. The following parameters in `.launch` file should be modified accordingly.

```xml
<arg name="suffix" default="_MyPlanner"/>
<arg name="strategy" default="MyPlanner" />
<arg name="map_name" default="my_environment/my_environment" />

<!-- same as in step 2 -->
<arg name="robot_position" default="0 0 0" />   
<!-- same as in step 2 -->
<arg name="map_width" default="74" />
```

#### Explore the new environment

7. Launch the roslaunch file as before. 
The new environment will be loaded into the Stage simulator.
The prior map will automatically be read and transformed into the robot's local coordinate frame.
The robot now starts to explore the new environment.






