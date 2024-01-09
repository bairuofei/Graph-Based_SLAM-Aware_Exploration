#include "ros/ros.h"
#include "MyPlanner.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Int16.h>

#include <eigen3/Eigen/Dense>

#include <algorithm>
#include <map>
#include <fstream>
#include <numeric>
#include <limits>
#include <iomanip>  // accuracy of float point numbers


int removeSmallUnknownCells(GridMap* map);
bool isGoalInMap(GridMap* map, double x, double y);
std::pair<int, int> getPointCoordinate(GridMap* map, double x, double y);
int removeSmallFrontiers(std::vector<std::vector<unsigned int>>& mFrontiers);
std::pair<double, double> getMapIndexPosition(GridMap* map, const unsigned int& index_x, const unsigned int& index_y);


double maxDoubleValue = std::numeric_limits<double>::max();

typedef std::multimap<double,unsigned int> Queue;
typedef std::pair<double,unsigned int> Entry;

int mid_size = -1;


MyPlanner::MyPlanner()
{
	ros::NodeHandle robotNode;
	robotNode.param("robot_id", mRobotID, 1);
	
	ros::NodeHandle navigatorNode("~/");
	navigatorNode.param("min_target_area_size", mMinTargetAreaSize, 10.0);
	navigatorNode.param("visualize_frontiers", mVisualizeFrontiers, false);
	navigatorNode.param("use_local_planner", useLocalReplanning, false);
    std::string defaultPath = " ";
    navigatorNode.param("save_path_plan_time", mSavePlanTimePath, defaultPath);

    std::ofstream outfile(mSavePlanTimePath, std::ios_base::trunc); // append
    if (outfile.is_open()){
        outfile.close(); // Close file
    } else {
        ROS_WARN("Unable to open file at mSavePlanTimePath.");
    }
	
	if(mVisualizeFrontiers)
	{
		mFrontierPublisher = navigatorNode.advertise<visualization_msgs::Marker>("frontiers", 1, true);
		mFrontierToVertexPublisher = navigatorNode.advertise<visualization_msgs::Marker>("frontier_to_vertex", 1, true);
	}
	// 3rd parameter is latch
	this->mGoalPublisher = navigatorNode.advertise<visualization_msgs::Marker>("nav_goal", 2, true); 
	this->pubPriorMap = navigatorNode.advertise<visualization_msgs::MarkerArray>("prior_map", 10, true);  
	this->mPubTspPath = navigatorNode.advertise<visualization_msgs::Marker>("visual_tsp_path", 2, true);
	this->mPubTspIndex = navigatorNode.advertise<visualization_msgs::MarkerArray>("tsp_index", 2, true);
	this->mDebugPathFinder = navigatorNode.advertise<visualization_msgs::Marker>("path_finder", 2, true);
	this->pubDistanceMap = navigatorNode.advertise<nav_msgs::OccupancyGrid>("dist_map", 1, true);

	finished_pub = robotNode.advertise<std_msgs::Bool>("/finished_topic", 10);

	// Send out replan request
	this->mRequestReplan = robotNode.advertise<std_msgs::Int16>("request_replan", 10, true);

	// Get tsp plan from planner node
	// this->mSubTspPlan = robotNode.subscribe("tsp_path", 1, &MyPlanner::handleTspPlan, this);

	// Stop exploration
	this->SubStopExploration = robotNode.subscribe("stop_exploration", 1, &MyPlanner::handleStopExploration, this);

	// Prior graph request test
	ROS_INFO("Waiting for prior_graph_service ...");
	bool priorGraphServiceAvailable = ros::service::waitForService("prior_graph_service", ros::Duration(30.0));
    if (!priorGraphServiceAvailable)
    {
        ROS_ERROR("prior_graph_service not available.");
    }
	else{
		this->clientPriorGraph = robotNode.serviceClient<cpp_solver::RequestGraph>("prior_graph_service");
		cpp_solver::RequestGraph clientMessage2;
		// No parameter needed for this service
		if (this->clientPriorGraph.call(clientMessage2))
		{	
			cpp_solver::RequestGraph::Response res2 = clientMessage2.response;
			std::vector<int32_t> vertex_list = res2.vertices;
			std::vector<float> x_coords = res2.x_coords;
			std::vector<float> y_coords = res2.y_coords;
			std::vector<int32_t> edges_start = res2.edges_start;
			std::vector<int32_t> edges_end = res2.edges_end;
			ROS_INFO("Receive response of service prior_graph_service.");
			// Set mNodeList and mEdgeList
			this->readPriorMap(vertex_list, x_coords, y_coords, edges_start, edges_end);
			ROS_INFO("Load prior_map.");
		}
		else
		{
			std::cout << "Failed to call service: prior_graph_service" << std::endl;
		}
	}

	this->unique_markers_id = 0;
    this->drawPriorGraph();
	ROS_INFO("Draw prior_map in RVIZ.");


	// Path planning service test
	ROS_INFO("Waiting for path_plan_service ...");
	bool pathPlanServiceAvailable = ros::service::waitForService("path_plan_service", ros::Duration(30.0));
    if (!pathPlanServiceAvailable)
    {
        ROS_ERROR("path_plan_service not available.");
    }
	else{
		this->clientTspPlan = robotNode.serviceClient<cpp_solver::TspPathList>("path_plan_service");
		cpp_solver::TspPathList clientMessage1;   // Create a service object
		clientMessage1.request.curr_vertex_idx = 0;
		clientMessage1.request.covered_vertices = std::vector<int32_t>();
		// Send request and wait response
		if (this->clientTspPlan.call(clientMessage1)) {
			cpp_solver::TspPathList::Response res = clientMessage1.response;
			this->tsp_path = res.response;
			this->mIsLoop.clear();
			for(bool val: res.isLoop){
				this->mIsLoop.push_back(val);
			}
		}
		else {
			std::cout << "Failed to call service: path_plan_service" << std::endl;
		}
	}

	std::stringstream ss;
    for (size_t i = 0; i < this->tsp_path.size(); ++i) {
        ss << this->tsp_path[i]; 
        if (i != this->tsp_path.size() - 1) {
            ss << " "; 
        }
    }
    ROS_INFO("Receive tsp path: %s", ss.str().c_str());

	ss.str("");  // clear string in ss
	ss.clear();  // clear flags
	for (size_t i = 0; i < this->mIsLoop.size(); ++i) {
        ss << this->mIsLoop[i]; 
        if (i != this->mIsLoop.size() - 1) {
            ss << " "; 
        }
    }
	ROS_INFO("Receive loop index: %s", ss.str().c_str());


	// Reliable loop service test
	ROS_INFO("Waiting for reliable_loop_service ...");
	bool reliableLoopServiceAvailable = ros::service::waitForService("reliable_loop_service", ros::Duration(30.0));
    if (!reliableLoopServiceAvailable)
    {
        ROS_ERROR("reliable_loop_service not available.");
    }
	else{
		ROS_INFO("reliable_loop_service is available.");
		this->clientReliableLoop = robotNode.serviceClient<cpp_solver::ReliableLoop>("reliable_loop_service");
	}


	this->PublishTspPath();

	this->currGoalIdx = 0;
	this->mExplorationButton = false;

	// Active loop closing
	this->mLoopClosing = false;  // Indicator whether loop closing finished
	this->mClosingPath.clear();
	this->mCurrClosingIdx = 0;

	this->mReachGoal = false;
	this->mPlan = NULL;
    
    this->mPlanTimeRecord = 0;

	ROS_INFO("MyPlanner is initialized and running!!");

}

MyPlanner::~MyPlanner()
{
	if(mPlan)
		delete[] mPlan;
}


void MyPlanner::handleStopExploration(const std_msgs::Bool::ConstPtr& msg){
	this->mExplorationButton = msg->data;
	return;
}


/**
 * @brief Set mClosingPath for reliable loop closing
 * 
 * @param currPosition 
 */
void MyPlanner::setReliableLoopPath(const int& currGoal){
	// DOWN: Find closest poses in current vertex for loop closing
	cpp_solver::ReliableLoop loopRequestMsg;   
	loopRequestMsg.request.goal_vertex = currGoal;
	// Call service server
	bool getReliableLoop = false;
	if (this->clientReliableLoop.call(loopRequestMsg)) {
		cpp_solver::ReliableLoop::Response res = loopRequestMsg.response;
		std::vector<float> x_pos = res.loop_x_coords;
		std::vector<float> y_pos = res.loop_y_coords;
		if(res.loop_x_coords.size() > 0){
			this->mClosingPath.clear();
			for(int i = 0; i < x_pos.size(); i++){
				this->mClosingPath.push_back(std::make_pair(x_pos[i], y_pos[i]));
			}
			ROS_INFO("Set reliable loop closing path from reliable_loop_service.");
			getReliableLoop = true;
		}
	}
	if(!getReliableLoop){
		ROS_ERROR("Failed to call service: reliable_loop_service");
		// Get absolute position of current loop point
		if(std::find(this->mNodeList.begin(), this->mNodeList.end(), currGoal) == this->mNodeList.end()){
			ROS_ERROR("currGoal is not a vertex.");
		}
		double x = this->mPriorGraph.vertex(currGoal)->position().x();
		double y = this->mPriorGraph.vertex(currGoal)->position().y();
		std::pair<double, double> currPosition = std::make_pair(x, y);
		
		this->mClosingPath.clear();
		std::vector<double> delta_x{1.5, 0, -1.5, 0};
		std::vector<double> delta_y{0, 1.5, 0, -1.5};
		for(int i = 0; i < 4; i++){
			this->mClosingPath.push_back(std::make_pair(currPosition.first+delta_x[i], currPosition.second+delta_y[i]));
		}
		ROS_INFO("Set reliable loop closing path.");
	}
	this->mCurrClosingIdx = 0;
	return;
}

/**
 * @brief Check distance from start to mClosingPath[mCurrClosingIdx]
 * 
 * @param map 
 * @param start 
 * @return true 
 * @return false 
 */
bool MyPlanner::loopVertexReached(GridMap* map, unsigned int start){
	unsigned int start_x, start_y;
	map->getCoordinates(start_x, start_y, start);
	double curr_x = map->getOriginX() + start_x * map->getResolution();
	double curr_y = map->getOriginY() + start_y * map->getResolution();
	std::pair<double, double> target = this->mClosingPath[this->mCurrClosingIdx];
	if((target.first - curr_x)*(target.first - curr_x) + (target.second - curr_y)*(target.second - curr_y) < 0.3){
		return true;
	}
	return false;
}

/**
 * @brief Skip vertex that: (1) have been visited and not a closing vertex; 
 * 							(2) not visited but have been scanned and have no frontier
 * 		  Given currGoalIdx being the first index to check
 * @return true: Set valid value of currGoalIdx
 * @return false: Exploration finished
 */
bool MyPlanner::getUsefulGoalIndex(){
	// Skip vertices have been visited and not loop closure points.
	int prev_idx = this->currGoalIdx;
	std::set<int> visited_vertices;
	std::stringstream ss;
	for (int i = 0; i < this->currGoalIdx; i++){
		visited_vertices.insert(this->tsp_path[i]);
		ss << this->tsp_path[i] << " ";
	}
	// Check whether currGoal is valid
	ROS_DEBUG("visited_vertices: %s", ss.str().c_str());
	// (1) Not loop closurel; (2) have been visited or have been covered (means no frontier).
	while ((!this->mIsLoop[this->currGoalIdx]) &&
			((visited_vertices.find(this->tsp_path[this->currGoalIdx]) != visited_vertices.end()) || 
				this->mCoveredVertices.find(this->tsp_path[this->currGoalIdx]) != this->mCoveredVertices.end())){
		ROS_INFO("Skip repeated visit to vertex %d", this->tsp_path[this->currGoalIdx]);
		this->currGoalIdx++;
		if (this->currGoalIdx >= (int)this->tsp_path.size()){
			return false;
		}
	}
	ROS_INFO("Set newGoal %d", this->tsp_path[this->currGoalIdx]);
	return true;
}


/**
 * @brief Handler for reliable loop closing, set the value for "goal"
 * 
 * @param map 
 * @param start 
 * @param goal 
 * @return true 
 * @return false 
 */
bool MyPlanner::performReliableLooping(GridMap* map, unsigned int start, unsigned int &goal){
	// Down: Find reliable looping around the pose graph vertices
	if(this->loopVertexReached(map, start)){
		this->mCurrClosingIdx++;   // Traverse a list of poses for reliable loop closing
	}
	if(this->mCurrClosingIdx < this->mClosingPath.size()){  // Whether loop closing finished
		// Set next sub-waypoint in current loop closing
		double closing_goal_x = this->mClosingPath[this->mCurrClosingIdx].first;
		double closing_goal_y = this->mClosingPath[this->mCurrClosingIdx].second;
		std::pair<int, int> goal_coordinate = getPointCoordinate(map, closing_goal_x, closing_goal_y);
		map->getIndex(goal_coordinate.first, goal_coordinate.second, goal);
		return true;
	}
	return false;
}


bool MyPlanner::callReplanning(){
	// Replanning evaluation
	cpp_solver::TspPathList tspRequestMsg;   // 创建一个服务请求对象
	// currGoalIdx has been visited. So starting from currentGoalIdx do the planning
	tspRequestMsg.request.curr_vertex_idx = this->currGoalIdx;
	tspRequestMsg.request.covered_vertices = std::vector<int>();
	// 发送服务请求，并等待响应
	if (this->clientTspPlan.call(tspRequestMsg)) {
		cpp_solver::TspPathList::Response res = tspRequestMsg.response;
		this->tsp_path = res.response;
		this->mIsLoop.clear();
		for(bool val: res.isLoop){
			this->mIsLoop.push_back(val);
		}
		this->PublishTspPath();
		return true;
	}
	else {
		ROS_ERROR("Failed to call service: path_plan_service");
		// The tsp_path remain unchanged if fail to call the service
		return false;
	}
}


/**
 * @brief 
 * 
 * @param map Map cell index start from left-bottom corner, and increase to right-top corner.
 * @param start 
 * @param goal 
 * @return int 
 */
int MyPlanner::findExplorationTarget(GridMap* map, unsigned int start, unsigned int &goal)
{	
	// The state machine is controlled by two variables: this->getTspPlan + this->reach_goal
	/* State indicator: mStopExploration -- wait 
						mLoopClosing     -- reliable loop closing
	*/

	ros::spinOnce();
    bool saveTime = this->savePlanTime();
    if (!saveTime) 
        ROS_ERROR("save time doesnot run. path: %s", mSavePlanTimePath.c_str());
    ros::Time startPlanTime = ros::Time::now();

	// Frontier Calculation
    removeSmallUnknownCells(map);
	this->mFrontiers.clear();  
	this->mFrontierCells = 0;
	this->findFrontiers(map, start);   // Update mFrontiers, mFrontierCells, mPlan
	int numFrontier = removeSmallFrontiers(this->mFrontiers);
	this->mFrontierCells = 0;
	for(int k = 0; k < (int)this->mFrontiers.size(); k++){
		this->mFrontierCells += this->mFrontiers[k].size();
	}
	std::vector<int> frontier_to_vertex(mFrontiers.size(), -1);  // frontier: belonged node index
	std::map<int, std::vector<int>> vertex_to_frontiers;  // vertex -> frontiers
	// Use A* distance to allocate fronteirs
	// ROS_INFO("Run allocateFrontierToVertex");
	this->allocateFrontierToVertex(map, frontier_to_vertex, vertex_to_frontiers);
	// O for free space, 100 for obstacle, -1 for unknow
	this->publishFrontier(map, mFrontierCells, mFrontiers, frontier_to_vertex);

	// Record vertices that have no frontier and can skip if not a loop vertex
	this->mCoveredVertices.clear();
	for(const int& id: this->mNodeList){
		if(vertex_to_frontiers.find(id) == vertex_to_frontiers.end() || vertex_to_frontiers[id].size() == 0){
			Eigen::Vector2f vPos = this->mPriorGraph.vertex(id)->position();
			if(isGoalInMap(map, vPos.x(), vPos.y())){   // Whether unvisited vertices is located in current map
				std::pair<int, int> vPosInMap = getPointCoordinate(map, vPos.x(), vPos.y());
				unsigned int vIndexInMap;
				map->getIndex(vPosInMap.first, vPosInMap.second, vIndexInMap);
				if(map->isExplored(vIndexInMap)){    // Whether unvisited vertices have been scanned
					this->mCoveredVertices.insert(id);
				}
			}			
		}
	}

	static bool stopFlag = false;  // Only execute once
	if(this->mExplorationButton){
		if(!stopFlag){
			ROS_WARN("Waiting to start...");
			goal = start;
			stopFlag = true;
		}
        ros::Duration planDuration = ros::Time::now() - startPlanTime;
        mPlanTimeRecord = planDuration.toSec();
		return EXPL_WAITING;
	}
	stopFlag = false;

	if(this->mLoopClosing){  
		bool findLoopGoal = this->performReliableLooping(map, start, goal);
		if(findLoopGoal){
			this->publishGoalToRviz(map, goal);
            ros::Duration planDuration = ros::Time::now() - startPlanTime;
            mPlanTimeRecord = planDuration.toSec();
			return EXPL_TARGET_SET;
		}
	}

	if(this->mLoopClosing){  // Finish reliable loop closing
		this->mLoopClosing = false;
		// Replanning evaluation
		bool replanIsSuccess = this->callReplanning();
		this->currGoalIdx++;
		if(this->currGoalIdx >= this->tsp_path.size()){
			std_msgs::Bool msg;
			msg.data = true;
			finished_pub.publish(msg);
            ros::Duration planDuration = ros::Time::now() - startPlanTime;
            mPlanTimeRecord = planDuration.toSec();
			return EXPL_FINISHED;
		}

		if(!this->mIsLoop[this->currGoalIdx]){
			bool hasSetIndex = this->getUsefulGoalIndex();
			if(!hasSetIndex){
				std_msgs::Bool msg;
				msg.data = true;
				finished_pub.publish(msg);
                ros::Duration planDuration = ros::Time::now() - startPlanTime;
                mPlanTimeRecord = planDuration.toSec();
				return EXPL_FINISHED;
			}
		}
		this->mReachGoal = false;
	}else if(this->useLocalReplanning && !this->mIsLoop[this->currGoalIdx]){
		// TODO: Control the frequency
		ROS_INFO("Call local replanning...");
		bool replanIsSuccess = this->callReplanning();
	}

	if(this->currGoalIdx >= (int)this->tsp_path.size()){
		std_msgs::Bool msg;
		msg.data = true;
		finished_pub.publish(msg);
        ros::Duration planDuration = ros::Time::now() - startPlanTime;
        mPlanTimeRecord = planDuration.toSec();
		return EXPL_FINISHED;
	}
	// Moving to current_goal...
	int currGoal = this->tsp_path[this->currGoalIdx];
	if(this->mIsLoop[this->currGoalIdx]){  // Reach currGoal and is a loop closing point
		ROS_INFO("Next target is loop vertex %d(L)", currGoal);
		this->setReliableLoopPath(currGoal);   
		this->mLoopClosing = true;   // Indicator
        ros::Duration planDuration = ros::Time::now() - startPlanTime;
        mPlanTimeRecord = planDuration.toSec();
		return EXPL_WAITING;
	}

	// currGoal is not a loop vertex
	if (!this->mReachGoal){
		// DOWN: Start to explore frontier earlier
		if(this->hasReachVertexRegion(map, start, currGoal)){
			this->mReachGoal = true;
			ROS_INFO("Reach vertex %d", currGoal);	
		}else{
			ROS_DEBUG("Moving to goal %d ...", currGoal);
			// If currGoal has no frontier, then find next valid vertex
			// FIXMED: Vertex will be escaped if it is free and has no frontier
			Eigen::Vector2f vPos = this->mPriorGraph.vertex(currGoal)->position();
			bool goal_in_map = isGoalInMap(map, vPos.x(), vPos.y());
			bool goal_is_free = false;
			if(goal_in_map){
				std::pair<int, int> goal_coordinate = getPointCoordinate(map, vPos.x(), vPos.y());
				unsigned int goal_index_in_map;
				map->getIndex(goal_coordinate.first, goal_coordinate.second, goal_index_in_map);
				goal_is_free = map->isFree(goal_index_in_map);
			}
			if(goal_is_free){   // Goal vertex is free, and has no frontiers
				if(!this->mIsLoop[this->currGoalIdx] && vertex_to_frontiers.find(currGoal) == vertex_to_frontiers.end()){
					ROS_INFO("Skip currGoal %d that has no frontier and seen by robot", currGoal);
					this->currGoalIdx++;
					if(this->currGoalIdx >= this->tsp_path.size()){
						std_msgs::Bool msg;
						msg.data = true;
						finished_pub.publish(msg);
                        ros::Duration planDuration = ros::Time::now() - startPlanTime;
                        mPlanTimeRecord = planDuration.toSec();
						return EXPL_FINISHED;
					}
					bool hasSetIndex = this->getUsefulGoalIndex();
					if(!hasSetIndex){
						std_msgs::Bool msg;
						msg.data = true;
						finished_pub.publish(msg);
                        ros::Duration planDuration = ros::Time::now() - startPlanTime;
                        mPlanTimeRecord = planDuration.toSec();
						return EXPL_FINISHED;
					}
				}
			}
			currGoal = this->tsp_path[this->currGoalIdx];
			goal = this->getWaypointToGoal(map, currGoal, start);  
			this->publishGoalToRviz(map, goal);
            ros::Duration planDuration = ros::Time::now() - startPlanTime;
            mPlanTimeRecord = planDuration.toSec();
			return EXPL_TARGET_SET;
		}
	}

	// Check whether frontiers of currGoal is reachable
	bool hasFrontier = false;
	int best_frontier = -1;
	if(vertex_to_frontiers.find(currGoal) != vertex_to_frontiers.end()){  // currGoal has frontiers
		best_frontier = this->findDistanceToFrontiers(map, start, vertex_to_frontiers[currGoal]);
		if(best_frontier >= 0){
			hasFrontier = true;
		}
		if(!hasFrontier){
			ROS_WARN("No frontier reachable at vertex %d. #Frontier: %d", currGoal, vertex_to_frontiers[currGoal].size());
			for(const unsigned int& v: vertex_to_frontiers[currGoal]){
				ROS_WARN("Size: %d. Center: %d, %d", this->mFrontiers[v].size(), this->mCenterOfFrontier[v].first, this->mCenterOfFrontier[v].second);
			}
		}
	}else{
		ROS_WARN("No frontier at vertex %d. #Frontier: 0", currGoal);
	}
	// No frontier reachable then move to next goal
	if(!hasFrontier){
		if(this->currGoalIdx + 1 >= (int)this->tsp_path.size()){
			std_msgs::Bool msg;
			msg.data = true;
			finished_pub.publish(msg);
            ros::Duration planDuration = ros::Time::now() - startPlanTime;
            mPlanTimeRecord = planDuration.toSec();
			return EXPL_FINISHED;
		}
		// Skip vertices have been visited and not loop closure points.
		this->currGoalIdx++;
		if(this->currGoalIdx >= this->tsp_path.size()){
			std_msgs::Bool msg;
			msg.data = true;
			finished_pub.publish(msg);
            ros::Duration planDuration = ros::Time::now() - startPlanTime;
            mPlanTimeRecord = planDuration.toSec();
			return EXPL_FINISHED;
		}
		bool hasSetIndex = this->getUsefulGoalIndex();
		if(!hasSetIndex){
			std_msgs::Bool msg;
			msg.data = true;
			finished_pub.publish(msg);
            ros::Duration planDuration = ros::Time::now() - startPlanTime;
            mPlanTimeRecord = planDuration.toSec();
			return EXPL_FINISHED;
		}
		int nextGoal = this->tsp_path[this->currGoalIdx];
		// FIXMED: here cannot get valid waypoint
		goal = this->getWaypointToGoal(map, nextGoal, start);  
		this->mReachGoal = false;
		this->publishGoalToRviz(map, goal);
		// ROS_INFO("Set newGoal %d", nextGoal);
        ros::Duration planDuration = ros::Time::now() - startPlanTime;
        mPlanTimeRecord = planDuration.toSec();
		return EXPL_TARGET_SET;
	}

	// Explore currGoal's frontiers
	ROS_INFO("Explore Goal %d 's frontiers...", currGoal);
	std::pair<unsigned int, unsigned int> res = this->mCenterOfFrontier[best_frontier];
	map->getIndex(res.first, res.second, goal);
	this->publishGoalToRviz(map, goal);
    ros::Duration planDuration = ros::Time::now() - startPlanTime;
    mPlanTimeRecord = planDuration.toSec();
	return EXPL_TARGET_SET;
}


bool MyPlanner::savePlanTime() {
    if (mSavePlanTimePath == " ") {
        ROS_WARN("Do not set mSavePlanTimePath.");
        return false;
    }

    std::ofstream outfile(mSavePlanTimePath, std::ios_base::app); // append
    if (outfile.is_open()){
        outfile << std::fixed << std::setprecision(2) << mPlanTimeRecord << std::endl;
        outfile.close(); // Close file
        return true;
    } else {
        ROS_WARN("Unable to open file at mSavePlanTimePath.");
        return false;
    }
}


/**
 * @brief 1. Compute distances from prior map vertexes to frontiers, allocate each frontier to vertexes.
 * 		  2. Store the distance information;
 * 		  3. Publish vertex -> frontier relationship. 
 * @param map 
 * @param frontier_to_vertex frontier_idx to priorgraph_vertex_idx
 * @param vertex_to_frontiers vertex_idx to list-of-frontier_idx
 */
void MyPlanner::allocateFrontierToVertex(GridMap* map, std::vector<int>& frontier_to_vertex, std::map<int, std::vector<int>>& vertex_to_frontiers){
	if(this->mFrontiers.size() == 0){
		return;
	}
	this->unique_markers_id = 1000;
	// Note line_list type is Marker rather than MarkerArray
	visualization_msgs::Marker line_list;
    line_list.header.frame_id = "map";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "frontier_to_vertex";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = this->unique_markers_id;
	this->unique_markers_id++;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_list.scale.x = 0.2;
    // Line list is red
    line_list.color.r = 0.0;
	line_list.color.g = 1.0;
    line_list.color.a = 0.9;

	this->mCenterOfFrontier.clear();
	for(int i = 0; i < this->mFrontiers.size(); i++){
		// Find center index of current frontier
		unsigned int center_x = 0, center_y = 0;
		for(unsigned int cell = 0; cell < this->mFrontiers[i].size(); cell++)
		{	
			unsigned int getX, getY;
			map->getCoordinates(getX, getY, this->mFrontiers[i][cell]);
			center_x += getX;
			center_y += getY;
		}
		center_x /= this->mFrontiers[i].size();
		center_y /= this->mFrontiers[i].size();

		// Find the closest cell to the center of current frontier
		double closest_index_distance;
		unsigned int closest_cell_idx;
		for(unsigned int cell = 0; cell < this->mFrontiers[i].size(); cell++)
		{	
			// Check if current cell is easy for trajectory planning
			unsigned int numFreeNeighbor = map->getNumFreeNeighbors(this->mFrontiers[i][cell]);
			if(numFreeNeighbor < 4){  // Adjacent frontier cells are also free
				continue;
			}
			unsigned int getX, getY;
			map->getCoordinates(getX, getY, this->mFrontiers[i][cell]);
			double index_distance = (getX - center_x) * (getX - center_x) + (getY - center_y) * (getY - center_y);
			if(cell == 0 || index_distance < closest_index_distance){
				closest_cell_idx = cell;
				closest_index_distance = index_distance;
			}
		}
		unsigned int getX, getY;
		map->getCoordinates(getX, getY, this->mFrontiers[i][closest_cell_idx]);
		this->mCenterOfFrontier.push_back(std::make_pair(getX, getY));

		// Use the closest cell to the centroid of each frontier to compute its distance to each vertex
		double map_x = map->getOriginX() + getX * map->getResolution();
		double map_y = map->getOriginY() + getY * map->getResolution();
		double best_distance = maxDoubleValue;
		double distance = 0;
		for(const int& id: this->mNodeList){
			// If current vertex has been visited, directly allocate to the next vertex
			if(this->currGoalIdx > 0){
				auto endIterator = this->tsp_path.begin() + this->currGoalIdx;
				auto result = std::find(this->tsp_path.begin(), endIterator, id);
				if(result != endIterator){  // this vertex has been visited
					continue;
				}
			}
			Eigen::Vector2f vPos = this->mPriorGraph.vertex(id)->position();
			// If vertex in current, then use A*; If not, then use manhattan distance
			int vertex_x_idx = (int)((vPos.x() - map->getOriginX()) / map->getResolution());
			int vertex_y_idx = (int)((vPos.y() - map->getOriginY()) / map->getResolution());
			distance = std::abs<int>(vertex_x_idx - static_cast<int>(getX)) + std::abs<int>(vertex_y_idx - static_cast<int>(getY));
			if(distance > best_distance){
				continue;
			}
			// TODO: If this vertex is unknown, then enlarge its distance because the estimation may be inaccurate
			if(vertex_x_idx >= map->getWidth() || vertex_y_idx >= map->getHeight() || vertex_x_idx < 0 || vertex_y_idx < 0 || 
												  !map->isFree(vertex_x_idx, vertex_y_idx)){
				distance *= 1.3;
			}

			if(distance < best_distance){
				best_distance = distance;
				frontier_to_vertex[i] = id;
			}
		}

		// Skip some vertices that have small frotiers and have been visited before
		bool belongToPrevVertex = false;
		for(const int& id: this->mNodeList){
			if(this->currGoalIdx > 0){
				auto endIterator = this->tsp_path.begin() + this->currGoalIdx;
				auto result = std::find(this->tsp_path.begin(), endIterator, id);
				if(result != endIterator){  // this vertex has been visited
					Eigen::Vector2f vPos = this->mPriorGraph.vertex(id)->position();
					int vertex_x_idx = (int)((vPos.x() - map->getOriginX()) / map->getResolution());
					int vertex_y_idx = (int)((vPos.y() - map->getOriginY()) / map->getResolution());
					distance = std::abs<int>(vertex_x_idx - static_cast<int>(getX)) + std::abs<int>(vertex_y_idx - static_cast<int>(getY));
					if(distance < best_distance){
						belongToPrevVertex = true;
						break;
					}
				}	
			}
		}
		// Skip such small frontiers that belong to previous vertices
		// Most such frontiers are caused by the mapping error, rather than exploration coverage.
		if(belongToPrevVertex){  
			frontier_to_vertex[i] = -1;  // just omit
			continue;
			// if(this->mFrontiers[i].size() < 25){
			// 	frontier_to_vertex[i] = -1;
			// 	continue;
			// }
		}

		vertex_to_frontiers[frontier_to_vertex[i]].push_back(i);
		// Publish this connectioon
		geometry_msgs::Point p1;
		p1.x = map_x;
		p1.y = map_y;
		p1.z = 0;
		line_list.points.push_back(p1);
		geometry_msgs::Point p2;
		int id = frontier_to_vertex[i];
		Eigen::Vector2f vPos = this->mPriorGraph.vertex(id)->position();
		p2.x = vPos.x();
		p2.y = vPos.y();
		p2.z = 0;
		line_list.points.push_back(p2);
	}

	// delete[] map_arr;  // Delete array to store current map

	this->mFrontierToVertexPublisher.publish(line_list);
	return;
}


/**
 * @brief Send current goal index to TSP planner.
 * 
 * @return
 */
void MyPlanner::requestReplan(){
	// Request replanning after reaching each vertex
	std_msgs::Int16 replan;
	replan.data = this->currGoalIdx;
	this->mRequestReplan.publish(replan);
}


bool MyPlanner::hasReached(GridMap* map, unsigned int start, const int& current_goal){
	double x1 = this->mPriorGraph.vertex(current_goal)->position().x();
	double y1 = this->mPriorGraph.vertex(current_goal)->position().y();
	unsigned int start_x, start_y;
	map->getCoordinates(start_x, start_y, start);
	double x2 = map->getOriginX() + start_x * map->getResolution();
	double y2 = map->getOriginY() + start_y * map->getResolution();
	double threshold = 1;
	if(sqrt(x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) < threshold)
		return true;
	return false;
}

/**
 * @brief Check whether the robot is the closest to currentGoal than to other vertices
 * 
 * @param map 
 * @param start 
 * @param current_goal 
 * @return true 
 * @return false 
 */
bool MyPlanner::hasReachVertexRegion(GridMap* map, unsigned int start, const int& current_goal){
	unsigned int start_x_idx, start_y_idx;
	map->getCoordinates(start_x_idx, start_y_idx, start);
	double start_x = map->getOriginX() + start_x_idx * map->getResolution();
	double start_y = map->getOriginY() + start_y_idx * map->getResolution();

	double goal_x = this->mPriorGraph.vertex(current_goal)->position().x();
	double goal_y = this->mPriorGraph.vertex(current_goal)->position().y();
	double goal_dist = std::pow(goal_x - start_x, 2) + std::pow(goal_y - start_y, 2);

	for(const int& id: this->mNodeList){
		if(id == current_goal)
			continue;
		Eigen::Vector2f vPos = this->mPriorGraph.vertex(id)->position();
		double other_dist = (vPos.x() - start_x)*(vPos.x() - start_x) + (vPos.y() - start_y)*(vPos.y() - start_y);
		if(other_dist < goal_dist){
			return false;
		}
	}
	return true;
}

/**
 * @brief Return closest frontier index. We use BFS propogation from start position, once reach the center
 * of one frontier, the search stops. If no frontier center can be reachable, the function also record the first encountered frontier 
 * cell, and the corresponding frontier will be selected as the target frontier. 
 * 
 * @param map 
 * @param start 
 * @param currFrontiers 
 * @param dist_to_frontiers 
 * @return int 
 */
int MyPlanner::findDistanceToFrontiers(GridMap* map, unsigned int start, const std::vector<int>& currFrontiers){
	// Fixed: Double free or corruption (out)

	// Build a dict for currFrontiers
	std::map<unsigned int, int> frontierCenterToFrontierIndex;
	for(int k = 0; k < currFrontiers.size(); k++){
		std::pair<unsigned int, unsigned int> centerCoordinate = this->mCenterOfFrontier[currFrontiers[k]];
		unsigned int centerIndex;
		map->getIndex(centerCoordinate.first, centerCoordinate.second, centerIndex);
		frontierCenterToFrontierIndex[centerIndex] = currFrontiers[k];
	}

	// Cell index to frontier map. Use frontier index in this->mFrontiers
	std::map<unsigned int, int> cellToFrontierIndex;
	for(int k = 0; k < currFrontiers.size(); k++){
		for(int p = 0; p < this->mFrontiers[currFrontiers[k]].size(); p++){
			unsigned int cellIndex = this->mFrontiers[currFrontiers[k]][p];
			cellToFrontierIndex[cellIndex] = currFrontiers[k];
		}
	}

	double* plan = new double[map->getSize()];
	for(unsigned int i = 0; i < map->getSize(); i++){
		plan[i] = -1;
	} 
	Queue frontQueue;
	// Start from the "start" position
	frontQueue.insert(Entry(0.0, start)); 
	int result = -1;
	double bestDistance = -1;
	bool flag = false; // Flag to reach first frontier
	double maxDistance = 0;
	while(!frontQueue.empty()){
		Queue::iterator next = frontQueue.begin();
		double distance = next->first;
		unsigned int index = next->second;
		frontQueue.erase(next);
		// Reach frontier
		if(!flag && cellToFrontierIndex.find(index) != cellToFrontierIndex.end()){
			flag = true;
			result = cellToFrontierIndex[index];
			bestDistance = distance;
		}
		// Reach center of one frontier
		if(frontierCenterToFrontierIndex.find(index) != frontierCenterToFrontierIndex.end()){
			result = frontierCenterToFrontierIndex[index];
			bestDistance = distance;
			break;
		}
		for(unsigned int it = 0; it < 8; it++) {
			int ind = index + this->mOffset[it];
			// Note here if want to judge whether the cell is free, you should compare with getLethalCost(), rather than 0
			if(ind >= 0 && ind < (int)map->getSize() && map->getData(ind) < map->getLethalCost() && plan[ind] < 0) { // This cell has not been visited
				if(it < 4){
					plan[ind] = distance + map->getResolution();
				}else{
					plan[ind] = distance + 1.414 * map->getResolution();
				}
				frontQueue.insert(Entry(plan[ind], ind));
				if(plan[ind] > maxDistance){
					maxDistance = plan[ind];
				}
			}
		}
	}
	
	// FIXMED: Why still cannot find? Because whether cell is free should use getLethalCost(), rather than 0
	if(result < 0){  // Not find valid frontier
		ROS_ERROR("Can not find path to %d local frontiers.", currFrontiers.size());
		std::vector<std::pair<double, double>> debugPoints;
		for (auto it = cellToFrontierIndex.begin(); it != cellToFrontierIndex.end(); ++it) {
			unsigned int cell = it->first;
			unsigned int cell_x, cell_y;
			map->getCoordinates(cell_x, cell_y, cell);
			double x = map->getOriginX() + cell_x * map->getResolution();
			double y = map->getOriginY() + cell_y * map->getResolution();
			debugPoints.push_back(std::make_pair(x, y));
		}
		this->debugMarker(debugPoints);
		ROS_WARN("Publish these frontiers in Red to rviz.");

		// Publish distance map
		// Define the width, height, and resolution of the gridmap
		int width = map->getWidth();  // Number of cells in the x-axis
		int height = map->getHeight(); // Number of cells in the y-axis
		double resolution = map->getResolution(); // The size of each cell in meters

		// Prepare the occupancy grid data
		nav_msgs::OccupancyGrid gridmap;
		gridmap.header.stamp = ros::Time::now();
		gridmap.header.frame_id = "map"; // Change this to your desired frame_id
		gridmap.info.width = width;
		gridmap.info.height = height;
		gridmap.info.resolution = resolution;

		gridmap.info.origin.position.x = map->getOriginX();
		gridmap.info.origin.position.y = map->getOriginY();
		gridmap.info.origin.orientation.w = 1.0; // Default orientation

		// Assuming you have a 1D double array containing the cost values of each cell
		std::vector<int8_t> occupancy_data;
		occupancy_data.reserve(width * height);

		for (int i = 0; i < width * height; ++i) {
			// Scale the double cost values to the range [0, 100]
			double cost = plan[i];
			int8_t occupancy;
			// occupancy = map->getData(i);
			if(cost < 0){
				occupancy = -1;
			}else{
				occupancy = static_cast<int8_t>(std::max(0.0, std::min(100.0, (cost / maxDistance) * 100.0)));
			}
			occupancy_data.push_back(occupancy);
		}
		gridmap.data = occupancy_data;
		this->pubDistanceMap.publish(gridmap);

	}

	delete[] plan;

	return result;
}


void MyPlanner::debugMarker(const std::vector<std::pair<double, double>>& points_coordinates){
	visualization_msgs::Marker points;
	points.header.frame_id = "map";
	// points.header.stamp = ros::Time();
	points.type = visualization_msgs::Marker::POINTS;
	points.id = 2;
	// POINTS markers use x and y scale for width/height respectively
	points.scale.x = 0.1;
	points.scale.y = 0.1;
	// Points are red
	points.color.r = 1.0f;
	points.color.a = 0.8;

	for(const std::pair<double, double>& p: points_coordinates){
		geometry_msgs::Point point;
		point.x = p.first;
		point.y = p.second;
		points.points.push_back(point);
	}
	this->mDebugPathFinder.publish(points);
}

/**
 * @brief Update mFrontiers and mPlan
 * 
 * @param map 
 * @param start 
 */
void MyPlanner::findFrontiers(GridMap* map, unsigned int start){
	unsigned int mapSize = map->getSize();
	if(this->mPlan)
		delete[] this->mPlan;
	this->mPlan = new double[mapSize];  // Store the distance from current cell to each cell

	// Initialize map to be unknown
	for(unsigned int i = 0; i < mapSize; i++){
		this->mPlan[i] = -1;
	}

	this->mOffset[0] = -1;					// left
	this->mOffset[1] =  1;					// right
	this->mOffset[2] = -map->getWidth();	// up
	this->mOffset[3] =  map->getWidth();	// down
	this->mOffset[4] = -map->getWidth() - 1;
	this->mOffset[5] = -map->getWidth() + 1;
	this->mOffset[6] =  map->getWidth() - 1;
	this->mOffset[7] =  map->getWidth() + 1;
	
	// Initialize the queue with the robot position
	Queue queue;
	Entry startPoint(0.0, start);
	queue.insert(startPoint);
	this->mPlan[start] = 0;
	
	Queue::iterator next;
	unsigned int index;
	double linear = map->getResolution();
	int cellCount = 0;  // Count of cells in the queue

	// Search for frontiers with wavefront propagation
	while(!queue.empty()) {
		cellCount++;
		// Get the nearest cell from the queue
		next = queue.begin();
		double distance = next->first;
		index = next->second;
		queue.erase(next);
		
		// Now continue 1st level WPA
		for(unsigned int it = 0; it < 4; it++) {
			unsigned int i = index + mOffset[it];
			if(i >= mapSize){
				continue;
			}
			if(this->mPlan[i] == -1 && map->isFree(i)){  // Only free cell will check whether is frontier or not
				// Check if it is a frontier cell
				if(map->isFrontier(i)){
					this->findCluster(map, i);   // Frontier will be added in this function
				}else{
					queue.insert(Entry(distance+linear, i));  // Add into queue to search for frontier
				}
				this->mPlan[i] = distance+linear;
			}
		}
	}
	return;
}


/**
 * @brief Find frontier cell between start and current_goal, in case the current goal is not in SLAM map.
 * 
 * @param map 
 * @param current_goal 
 * @param start 
 */
unsigned int MyPlanner::getWaypointToGoal(GridMap* map, const int& current_goal, unsigned int start){
	// FIXMED: the robot may be stucked in front of a wall
	double vertex_x = this->mPriorGraph.vertex(current_goal)->position().x();
	double vertex_y = this->mPriorGraph.vertex(current_goal)->position().y();
	std::pair<int, int> coordinate = getPointCoordinate(map, vertex_x, vertex_y);
	// Note the index may be negative
	unsigned int goal_x_index = coordinate.first;
	unsigned int goal_y_index = coordinate.second;
	unsigned int start_x_index, start_y_index;
	map->getCoordinates(start_x_index, start_y_index, start);
	int dx_index = goal_x_index - start_x_index;
	int dy_index = goal_y_index - start_y_index;
	double initial_slope = -1.0;
	if(goal_y_index != start_y_index){
		initial_slope = std::abs<int>(goal_x_index - start_x_index) / std::abs<int>(goal_y_index - start_y_index);
	}
	int delta_x_index = 1, delta_y_index = 1;
	if(dx_index > 0)
		delta_x_index = -1;
	if(dy_index > 0)
		delta_y_index = -1;

	// If goal is not covered in current map, then find a goal that is within current map scope
	if(!isGoalInMap(map, vertex_x, vertex_y)){  // If x and y are larger than current bound of map index
		int height = map->getSize() / map->getWidth();
		// FIXMED: this goal may be stucked in a corner
		// ROS_INFO("While loop 1.");
		while(goal_x_index < 0 || goal_y_index < 0 || 
			  goal_x_index > map->getWidth() || goal_y_index > height){
			double curr_slope = -1.0;
			if(goal_y_index != start_y_index){
				curr_slope = std::abs<int>(goal_x_index - start_x_index) / std::abs<int>(goal_y_index - start_y_index);
			}
			if(goal_y_index == start_y_index || curr_slope >= initial_slope){
				goal_x_index += delta_x_index;
			}else if(goal_x_index == start_x_index || curr_slope < initial_slope){
				goal_y_index += delta_y_index;
			}
		}
	}
	ROS_DEBUG("Debug: goal index in map: %d, %d", goal_x_index, goal_y_index);

	// Up to now, new_x and new_y are within the current map scope
	unsigned int cell_idx;
	// FIXED
	map->getIndex(goal_x_index, goal_y_index, cell_idx);
	// If mPlan has value larger than 0, then this cell is reachable
	if(this->mPlan[cell_idx] >= 0){  // There is one way from start to current_goal in map
		return cell_idx;
	}else{ // Search on that direction to get one free cell, so that the robot can plan path
		// ROS_INFO("While loop 2.");
		// Directly assign the cloest frontier as current goal
		// Get postion of cell_idx
		double goal_x = map->getOriginX() + goal_x_index * map->getResolution();
		double goal_y = map->getOriginY() + goal_y_index * map->getResolution();
		int best_frontier_idx = -1;
		double best_dist;
		// Find the index of closest frontier
		for(int i = 0; i < this->mCenterOfFrontier.size(); i++){
			double frontier_x = map->getOriginX() + this->mCenterOfFrontier[i].first * map->getResolution();
			double frontier_y = map->getOriginY() + this->mCenterOfFrontier[i].second * map->getResolution();
			double dist = (goal_x - frontier_x)*(goal_x - frontier_x) + (goal_y - frontier_y)*(goal_y - frontier_y);
			if(i == 0 || dist < best_dist){
				best_dist = dist;
				best_frontier_idx = i;
			}
		}

		std::pair<unsigned int, unsigned int> res = this->mCenterOfFrontier[best_frontier_idx];
		map->getIndex(res.first, res.second, cell_idx);
		ROS_INFO("Find middle goal point.");
		
		return cell_idx;
	}
}


/**
 * @brief Given a frontier cell, expand it into a frontier cluster
 * 
 * @param map 
 * @param startCell 
 */
void MyPlanner::findCluster(GridMap* map, unsigned int startCell)
{	
	Frontier front;
	
	// Initialize a new queue with the found frontier cell
	Queue frontQueue;
	frontQueue.insert(Entry(0.0, startCell));
	bool isBoundary = false;
	
	while(!frontQueue.empty())
	{
		// Get the nearest cell from the queue
		Queue::iterator next = frontQueue.begin();
		double distance = next->first;
		unsigned int index = next->second;
		unsigned int x, y;
		frontQueue.erase(next);
		
		// Check if it is a frontier cell by inspecting its neighboring 8 cells
		if(!map->isFrontier(index)) 
			continue;
		
		// Add it to current frontier
		front.push_back(index);
		mFrontierCells++;
		
		// Add all adjacent cells to queue
		for(unsigned int it = 0; it < 4; it++)
		{
			int i = index + mOffset[it];
			// Only insert free and initially unknown cell into the queue
			// If the i is not free, then it will not be considered as frontier
			// isFree means in current occupancy map, the cell is free
			if(map->isFree(i) && mPlan[i] == -1)  
			{
				mPlan[i] = distance + map->getResolution();
				frontQueue.insert(Entry(distance + map->getResolution(), i));
			}
		}
	}
	this->mFrontiers.push_back(front);
}



void MyPlanner::drawPriorGraph(){
	this->unique_markers_id = 5;
    // Pub vertex marker
    ROS_INFO("Publishing prior graph......");
    visualization_msgs::MarkerArray marker_array;  // only has markers as its members

    visualization_msgs::Marker points;
    points.header.frame_id = "map";
    // points.header.stamp = ros::Time();
    points.type = visualization_msgs::Marker::POINTS;
    points.id = this->unique_markers_id;
	this->unique_markers_id++;
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.8;
    points.scale.y = 0.8;
    // Points are green
    points.color.g = 1.0f;
    points.color.a = 0.8;

    for(const int& id: mNodeList){
        geometry_msgs::Point contour_point;
        Eigen::Vector2f vPos = this->mPriorGraph.vertex(id)->position();
        contour_point.x = vPos.x();
        contour_point.y = vPos.y();
        points.points.push_back(contour_point);

		// Text shown on each vertex
		visualization_msgs::Marker text;
		text.header.frame_id = "map";
		text.header.stamp = ros::Time();
		text.id = unique_markers_id;
		unique_markers_id++;

		text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		text.action = visualization_msgs::Marker::ADD;

		text.pose.position = contour_point;
		text.pose.position.y -= 1;

		std::stringstream ss;
		ss << id;
		text.text = ss.str();

		text.scale.x = 1;
		text.scale.y = 1;
		text.scale.z = 1;

		text.color.r = 1.0f;
		text.color.g = 0.0f;
		text.color.b = 0.0f;
		text.color.a = 1.0;
		ROS_DEBUG("Publish vertex id. %d", text.id);
		marker_array.markers.push_back(text);
    }

	marker_array.markers.push_back(points);

    this->pubPriorMap.publish(marker_array);

    return;
}


void MyPlanner::publishGoalToRviz(GridMap* map, unsigned int goal){
	unsigned int x, y;
	map->getCoordinates(x, y, goal);
	// Publish goal point
    visualization_msgs::Marker goal_marker;
    goal_marker.header.frame_id = "map";
    goal_marker.header.stamp = ros::Time();
    goal_marker.id = 1;
    goal_marker.type = visualization_msgs::Marker::SPHERE;
    goal_marker.action = visualization_msgs::Marker::ADD;
    goal_marker.pose.position.x = map->getOriginX() + x * map->getResolution();
    goal_marker.pose.position.y = map->getOriginY() + y * map->getResolution();
    goal_marker.pose.position.z = 0.0;
    goal_marker.pose.orientation.x = 0.0;
    goal_marker.pose.orientation.y = 0.0;
    goal_marker.pose.orientation.z = 0.0;
    goal_marker.pose.orientation.w = 1.0;
    goal_marker.scale.x = 10 * map->getResolution();
    goal_marker.scale.y = 10 * map->getResolution();
    goal_marker.scale.z = 10 * map->getResolution();
    goal_marker.color.a = 0.5;
    goal_marker.color.r = 1.0;
    goal_marker.color.g = 0.0;
    goal_marker.color.b = 0.0;

    mGoalPublisher.publish(goal_marker);
}

void MyPlanner::publishFrontier(GridMap* map, unsigned int& mFrontierCells, FrontierList& mFrontiers, std::vector<int>& frontier_to_node){
	if(mVisualizeFrontiers)
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = "map";
		marker.header.stamp = ros::Time();
		marker.id = 0;
		marker.type = visualization_msgs::Marker::CUBE_LIST;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = map->getOriginX();
		marker.pose.position.y = map->getOriginY();
		// marker.pose.position.z = map->getResolution() / 2;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = map->getResolution();
		marker.scale.y = map->getResolution();
		marker.scale.z = map->getResolution();
		marker.color.a = 0.5;
		marker.color.r = 1.0;
		marker.color.g = 1.0;
		marker.color.b = 1.0;
		marker.points.resize(mFrontierCells);  
		marker.colors.resize(mFrontierCells);
		
		unsigned int p = 0;
		srand(1337);
		unsigned int x, y;
		
		for(unsigned int i = 0; i < mFrontiers.size(); i++)
		{	
			char r, g, b;
			if(this->node_to_color.find(frontier_to_node[i]) != this->node_to_color.end()){  // key in node_to_color
				r = std::get<0>(this->node_to_color[frontier_to_node[i]]);
				g = std::get<1>(this->node_to_color[frontier_to_node[i]]);
				b = std::get<2>(this->node_to_color[frontier_to_node[i]]);
			}else{
				r = rand() % 256;
				g = rand() % 256;
				b = rand() % 256;
				this->node_to_color[frontier_to_node[i]] = std::tuple<char, char, char>(r, g, b);
			}
			
			for(unsigned int j = 0; j < mFrontiers[i].size(); j++)
			{
				if(p < mFrontierCells)
				{
					if(!map->getCoordinates(x, y, mFrontiers[i][j]))
					{
						ROS_ERROR("[MyPlanner] getCoordinates failed!");
						break;
					}
					marker.points[p].x = x * map->getResolution();
					marker.points[p].y = y * map->getResolution();
					marker.points[p].z = 0;
					
					marker.colors[p].r = r;
					marker.colors[p].g = g;
					marker.colors[p].b = b;
					marker.colors[p].a = 0.6;
				}else
				{
					ROS_ERROR("[MyPlanner] SecurityCheck failed! (Asked for %d / %d)", p, mFrontierCells);
				}
				p++;
			}
		}
		mFrontierPublisher.publish(marker);
	}
}

/**
 * @brief Build prior map, and modify this->mNodeList, this->mEdgeList.
 * 
 */
void MyPlanner::readPriorMap(std::vector<int32_t>& vertex_list, 
							 std::vector<float>& x_coords,
							 std::vector<float>& y_coords,
							 std::vector<int32_t>& edges_start,
							 std::vector<int32_t>& edges_end){
	mNodeList = vertex_list;
	std::vector<std::pair<double, double>> node_pose_list;
	for(int i = 0; i < x_coords.size(); i++){
		node_pose_list.emplace_back(std::make_pair(x_coords[i], y_coords[i]));
	}
	// Add nodes to prior graph
	for(int j = 0; j < mNodeList.size(); j++){
		this->mPriorGraph.addVertex(mNodeList[j], 
									Eigen::Vector2f(node_pose_list[j].first, node_pose_list[j].second));
	}

	// Add edges to prior graph
	for(int k = 0; k < edges_start.size(); k++){
		std::pair<int, int> edge_pair = std::make_pair(edges_start[k], edges_end[k]);
		mEdgeList.push_back(edge_pair);
		Graph::Vertex* fromV = this->mPriorGraph.vertex(edges_start[k]);
		Graph::Vertex* toV = this->mPriorGraph.vertex(edges_end[k]);
		if (fromV == nullptr || toV == nullptr){
			ROS_ERROR("One of the entered id is not present in the graph.");
			continue;
		}
		bool undirected = true;
		Graph::Edge* e = this->mPriorGraph.addEdge(edges_start[k], edges_end[k], undirected);
	}

	return;
}


/**
 * @brief Publish tsp_path to RVIZ
 * 
 */
void MyPlanner::PublishTspPath(){
	this->unique_markers_id = 500;

	// For TSP path index
	visualization_msgs::MarkerArray marker_array;
	// Note line_list type is Marker rather than MarkerArray
	visualization_msgs::Marker line_list;
    line_list.header.frame_id = "map";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "points_and_lines";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = this->unique_markers_id;
	this->unique_markers_id++;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_list.scale.x = 0.1;
    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 0.1;

	// Get line list according to TSP path
	std::vector<std::pair<double, double>> tsp_pos; 
	for(int i = 0; i + 1 < this->tsp_path.size(); i++){
		int idx1 = this->tsp_path[i];
		int idx2 = this->tsp_path[i+1];
		Eigen::Vector2f vPos1 = this->mPriorGraph.vertex(idx1)->position();
		geometry_msgs::Point p1;
		p1.x = vPos1.x();
		p1.y = vPos1.y();
		p1.z = 0;
		line_list.points.push_back(p1);
		Eigen::Vector2f vPos2 = this->mPriorGraph.vertex(idx2)->position();
		geometry_msgs::Point p2;
		p2.x = vPos2.x();
		p2.y = vPos2.y();
		p2.z = 0;
		line_list.points.push_back(p2);
		
		// Text visulation
		std::vector<geometry_msgs::Point> points;
		points.push_back(p1);
		if(i + 2 == this->tsp_path.size()){
			points.push_back(p2);
		}
		int count = i;
		for(geometry_msgs::Point& p: points){
			visualization_msgs::Marker text;
			text.header.frame_id = "map";
			text.header.stamp = ros::Time();
			text.id = this->unique_markers_id;
			this->unique_markers_id++;
			text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			text.action = visualization_msgs::Marker::ADD;
			text.pose.position = p;
			// Plot visit order above the vertex
			text.pose.position.y += 1;
			std::stringstream ss;
			ss << count;
			count++;
			text.text = ss.str();
			text.scale.x = 1;
			text.scale.y = 1;
			text.scale.z = 1;
			text.color.r = 0.0f;
			text.color.g = 0.0f;
			text.color.b = 1.0f;
			text.color.a = 1.0;
			marker_array.markers.push_back(text);
		}
	}
    this->mPubTspPath.publish(line_list);
	this->mPubTspIndex.publish(marker_array);
	return;
}


int removeSmallUnknownCells(GridMap* map){
    // Each cell of the map may have three values, -1, 0, 100?
	unsigned int mapSize = map->getSize();
    std::vector<unsigned int> removeUnknown;
	for(unsigned int i = 0; i < mapSize; i++){
		if(map->isUnknown(i)){
            unsigned int freeNeighbor = map->getNumFreeNeighbors(i);
            if(freeNeighbor >= 7)
                removeUnknown.push_back(i);
        }
	}
	for(unsigned int& i: removeUnknown){
		map->setData(i, 0);
	}
    // ROS_DEBUG("%d Small Unknown cell is removed.", static_cast<int>(removeUnknown.size()));
    return 0;
}


/**
 * @brief Check if a absolute position is in current map or not. Both map and the position have 
 * absolute positions w.r.t the intial position of the robot.
 * @param map 
 * @param x 
 * @param y 
 * @return true 
 * @return false 
 */
bool isGoalInMap(GridMap* map, double x, double y){
	unsigned int mapSize = map->getSize();
	int height = mapSize / map->getWidth();
	double lb_x = map->getOriginX();
	double lb_y = map->getOriginY();
	double rt_x = map->getOriginX() + map->getWidth() * map->getResolution();
	double rt_y = map->getOriginY() + height * map->getResolution();
	if(x >= lb_x && x <= rt_x && y >= lb_y && y <= rt_y){
		return true;
	}
	return false;
}

std::pair<int, int> getPointCoordinate(GridMap* map, double x, double y){
	// Given two coordinates x and y, find its index in map
	int x_grid = static_cast<int>((x - map->getOriginX()) / map->getResolution());
	int y_grid = static_cast<int>((y - map->getOriginY()) / map->getResolution());
	return std::pair<int, int>(x_grid, y_grid);
}

std::pair<double, double> getMapIndexPosition(GridMap* map, const unsigned int& index_x, const unsigned int& index_y){
	double x_absolute = map->getOriginX() + index_x * map->getResolution();
	double y_absolute = map->getOriginY() + index_y * map->getResolution();
	return std::make_pair(x_absolute, y_absolute);
}

/**
 * @brief Remove small frontiers from mFrontiers and return the number of the remaining.
 * 
 * @param mFrontiers 
 * @return int 
 */
int removeSmallFrontiers(std::vector<std::vector<unsigned int>>& mFrontiers){
	int numF = (int)mFrontiers.size();
	if(numF == 0){
		return numF;
	}

    std::vector<int> frontier_sizes;
    for(int k = 0; k < numF; k++){
        frontier_sizes.push_back((int)mFrontiers[k].size());
    }
    std::sort(frontier_sizes.begin(), frontier_sizes.end());
	ROS_DEBUG("Max-front Size: %d, Min-front Size: %d", frontier_sizes.back(), frontier_sizes.front());
	// mid_size will always be smaller than 10
	if(mid_size < 0){  // Initial value is -1
		mid_size = std::min((int)mFrontiers[numF/2].size(), 10);
	}else{
		mid_size = std::max(mid_size, std::min((int)mFrontiers[numF/2].size(), 10));
	}
    // 基本上mid_size会一直保持在10
    // mid_size = 4;

	std::vector<int> removeIndices;
	for(int k = 0; k < (int)mFrontiers.size(); k++){
		if(mFrontiers[k].size() < mid_size){
			removeIndices.push_back(k);
		}
	}
	// Remove from back to front to avoid index change
	for(auto it = removeIndices.rbegin(); it != removeIndices.rend(); ++it){
		if(*it < numF){
			mFrontiers.erase(mFrontiers.begin() + *it);
		}
	}

	ROS_DEBUG("%d Small Frontiers are removed. %d remain.", (int)removeIndices.size(), numF - (int)removeIndices.size());

	if((int)mFrontiers.size() == 0){	
		ROS_WARN("No Frontiers in the map!");
	}
	return (int)mFrontiers.size();
}

