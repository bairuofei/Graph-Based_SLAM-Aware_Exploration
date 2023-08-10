
# Graph-based SLAM-Aware Exploration
This repo includes a ROS plugin for the `nav2d` package for autonomous exploration based on the prior topo-metric graph of the environment.

<div style="display:flex; justify-content:center;">
<figure>
    <img src="./images/frontier.gif" alt="Alt Text" width="250" height="250">
    <figcaption style="text-align:center;">Frontier-based Method</figcaption>
</figure>
<figure>
    <img src="./images/active_tsp.gif" alt="Alt Text" width="250" height="250">
    <figcaption style="text-align:center;">Active TSP-based Method</figcaption>
</figure>
    
    
</div>


## Usage




##### ROS build

Run the command in a catkin workspace.

    $ catkin_make

##### ROS exploration

Run the commands in a catkin workspace.

    $ source devel/setup.bash
    $ roslaunch cpp_solver exploration.launch

Then it's possible to draw the graph in Rviz following the prompted instructions.
Hints: use the `publish point` tool from the Rviz toolbar to place vertices; write vertices pairs in the terminal to connect them with an edge. 

Once the graph is complete, start the exploration by running in another terminal

    $ rosservice call \StartMapping
    $ rosservice call \StartExploration

If you have a joystick connected, you can follow the Nav2d instructions for starting the mapping procedure instead of this last step (http://wiki.ros.org/nav2d/Tutorials/MultiMapper#Autonomous_Exploration).
