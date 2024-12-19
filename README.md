
# Graph-based SLAM-Aware Exploration

## Update (07/2024): Extension to multi-robot graph exploration

Extension of this repo to multi-robot graph exploration is [open-source here](https://github.com/bairuofei/CGE). 
The [related paper](https://arxiv.org/abs/2407.01013) has been accpeted by 2024 IEEE/RSJ IROS.


## Update (06/2024)

Our paper has been accpeted by IEEE RA-L for publication. Please follow [this link](https://ieeexplore-ieee-org.remotexs.ntu.edu.sg/document/10577228) to the preprint version.

Please consider citing our paper if you find it helpful.
```
@ARTICLE{10577228,
  author={Bai, Ruofei and Guo, Hongliang and Yau, Wei-Yun and Xie, Lihua},
  journal={IEEE Robotics and Automation Letters}, 
  title={Graph-Based SLAM-Aware Exploration With Prior Topo-Metric Information}, 
  year={2024},
  volume={},
  number={},
  pages={1-8},
  keywords={Planning under uncertainty;SLAM;autonomous exploration},
  doi={10.1109/LRA.2024.3420817}}
```

## Quick Overview
Source code for the paper ["*Graph-based SLAM-Aware Exploration with Prior Topo-Metric Information*](https://arxiv.org/abs/2308.16522)".
The paper proposes a SLAM-aware exploration method that exploits prior topo-metric information of the environment for fast exploration and enhanced pose graph reliability in the SLAM process.

Our method tends to form "**informative**" loop closures that can globally stablize the SLAM pose-graph based on the graph spectral analysis, thus balancing exploration efficiency and pose-graph reliability, as shown by the following results.


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

<figure>
    <img src="./images/map4.gif" alt="Alt Text" width="800" height="400">
    <!-- <figcaption style="text-align:center;">Active TSP-based Method</figcaption> -->
</figure>
</div>

## Requirements

The code has been developed and tested on Ubuntu 20.04, ROS noetic.

Install required:
- [Eigen](https://eigen.tuxfamily.org/dox/GettingStarted.html) REQUIRED
    - `sudo apt update & sudo apt install libeigen3-dev`
- ROS noetic
- [pyconcorde](https://github.com/jvkersch/pyconcorde) (A python library for Traveling Salesman Problem Solver, detailed usage refers to [here](https://github.com/jvkersch/pyconcorde/issues/28#issuecomment-770354999))
    - To install pyconcorde, you need to first install `concorde`, and set the PATH in the environment variable. You can simply download `concorde` and `linkern` [here](https://www.math.uwaterloo.ca/tsp/concorde/downloads/downloads.htm), unzip the executable programs, and set the path in `~/.bashrc` as `export PATH=$PATH:{your path to the executable programs}`.
- [p2os_urdf](https://github.com/allenh1/p2os) (A ros package that provides `Pionner3at` robot model in the simulation.)
    - Only one file `p2os_urdf/defs/pioneer3at.xacro` from this package is used, so you can simply download this file, and update the path to it in the launch file. 
    - By default you can install `p2os_urdf` globally, and then use `$(find p2os_urdf)` to get the path automatically.


Source code from following repo is also used:
- [astar-gridmap-2d](https://github.com/Eurecat/astar-gridmap-2d) (has been included in this repo, no need to install)



## Installation
There are two ros packages to install: `cpp_solver` (provided in this repo) and `navigation_2d` (provided [here](https://github.com/bairuofei/navigation_2d)).

```bash
$ mkdir -r catkin_ws/src
$ cd catkin_ws && catkin_make
$ source devel/setup.bash
$ cd src

# download navigation_2d package
$ git clone git@github.com:bairuofei/navigation_2d.git

# download Graph-Based_SLAM-Aware_Exploration package
$ git clone git@github.com:bairuofei/Graph-Based_SLAM-Aware_Exploration.git
$ mv Graph-Based_SLAM-Aware_Exploration cpp_solver

$ cd ..
$ catkin_make
$ source ./devel/setup.bash

```


## Usage

### Quick start
Open terminal 1
```
$ cd catkin_ws/
$ source ./devel/setup.bash

$ roslaunch cpp_solver exploration.launch
```
Open anthoer terminal 2
```
$ cd catkin_ws/
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



## Using Real-robot (Pioneer3AT by default)
1. Download and build Aria and RosAria
2. Download urg_node for Hokuyo laser.
3. Connect Pioneer robot, Hokuyo laser, and Joystick to PC, and change the permission properties of the ports;
4. Specify exploration strategy and prior graph;
5. Launch the `real_exploration_with_pioneer.launch` file to start exploration.



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

## Known Issues
1. Frontier allocation to vertices in the prior graph may be incorrect, becuase we only evaluate the Euclidean distance between fronters and vertices. 
Exact distance evaluation is also possible by using A* algorithm to explicitly evaluate the distance between each frontier to vertices in the prior graph, but would be time inefficient in our testing.

2. Frontier detection module sometimes misses one or two frontiers. More advanced implementation can be found [here](https://github.com/hasauino/rrt_exploration).






