#ifndef MYPLANNER_H
#define MYPLANNER_H

#include <sstream>
#include <nav2d_navigator/ExplorationPlanner.h>
#include <std_msgs/Bool.h>
#include <utility>
#include <chrono>

#include <nav_msgs/OccupancyGrid.h>

#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Int32MultiArray.h"
#include "graph/graph.h"
#include "cpp_solver/TspPathList.h"
#include "cpp_solver/RequestGraph.h"
#include "cpp_solver/ReliableLoop.h"

#include "AStar/AStar2.h"


class MyPlanner : public ExplorationPlanner
{
public:
        MyPlanner();
        ~MyPlanner();
        
        int findExplorationTarget(GridMap* map, unsigned int start, unsigned int &goal);
        
private:
        typedef std::vector<unsigned int> Frontier;
        typedef std::vector<Frontier> FrontierList;
        
        // Methods
        void findCluster(GridMap* map, unsigned int startCell);

        void findFrontiers(GridMap* map, unsigned int start);

        int findDistanceToFrontiers(GridMap* map, unsigned int start, const std::vector<int>& currFrontiers);

        bool hasReached(GridMap* map, unsigned int start, const int& current_goal);
        bool hasReachVertexRegion(GridMap* map, unsigned int start, const int& current_goal);

        void requestReplan();

        void allocateFrontierToVertex(GridMap* map, std::vector<int>& frontier_to_node, std::map<int, std::vector<int>>& node_map_frontier);

        void setReliableLoopPath(const int& currGoal);

        bool loopVertexReached(GridMap* map, unsigned int start);
        bool performReliableLooping(GridMap* map, unsigned int start, unsigned int &goal);
        // Create prior map
        void readPriorMap(std::vector<int32_t>& vertex_list, 
                        std::vector<float>& x_coords,
                        std::vector<float>& y_coords,
                        std::vector<int32_t>& edges_start,
                        std::vector<int32_t>& edges_end);
        // Draw prior map to rviz
        void drawPriorGraph();

        void publishGoalToRviz(GridMap* map, unsigned int goal);

        void publishFrontier(GridMap* map, unsigned int& mFrontierCells, FrontierList& mFrontiers, std::vector<int>&);

        unsigned int getWaypointToGoal(GridMap* map, const int& current_goal, unsigned int start);

        void handleStopExploration(const std_msgs::Bool::ConstPtr& msg);  // Server handle function for stop exploration

        void debugMarker(const std::vector<std::pair<double, double>>& points); // Use marker of type "points" to debug. marker id is 2.

        // void handleTspPlan(const std_msgs::Int32MultiArray::ConstPtr& msg);

        void PublishTspPath();

        bool getUsefulGoalIndex();

        bool callReplanning();

        bool savePlanTime();

        // std::pair<unsigned int, unsigned int> findGoalInFrontier(GridMap* map, const unsigned int& frontier_index, unsigned int start);

        int unique_markers_id;
        
        // ROS-Stuff
        ros::Publisher mFrontierPublisher;  // id = 0
        ros::Publisher mFrontierToVertexPublisher;  // id: [1000, 1500)

        // Publish navigation goal
        ros::Publisher mGoalPublisher;   // id = 1

        // Marker id 2 - 4 reserved for testing
        ros::Publisher mDebugPathFinder; 

        // Publish replanning request
        ros::Publisher mRequestReplan;

        // Publish TSP path marker
        ros::Publisher mPubTspPath;   // id: [500, 1000)
        ros::Publisher mPubTspIndex;

        // Publish priorMap
        ros::Publisher pubPriorMap;  // id: [5, 500)

        // Publish debug marker, index start from 2

        // Publish distance map
        ros::Publisher pubDistanceMap;

        // Subscribe command to stop exploration
        ros::Subscriber mCommandSub;

        // Subscribe TSP plan for navigation
        // ros::Subscriber mSubTspPlan;

        // Subscribe TSP plan for navigation
        ros::Subscriber SubStopExploration;

        // Client ask for python server for tsp planner and prior graph
        ros::ServiceClient clientTspPlan;
        ros::ServiceClient clientPriorGraph;
        ros::ServiceClient clientReliableLoop;


        ros::Publisher finished_pub;

        // Frontier
        // color map for each vertex when drawing frontiers
        std::map<int, std::tuple<char, char, char>> node_to_color;
        

        // Components
        RobotList mRobotList;
        FrontierList mFrontiers;  // Store frontier
        double* mPlan;  // Record the distance of all grid cells to current position
        unsigned int mFrontierCells;

        // Parameters
        int mRobotID;
        bool mVisualizeFrontiers;
        double mMinTargetAreaSize;
        unsigned int mOffset[8]; // Used to apply offset to current cell to get its neighbors
        
        // Prior map
        Graph mPriorGraph;
        std::vector<int> mNodeList;
        std::vector<std::pair<int, int>> mEdgeList;

        // Record the map index of each frontier center. Use same index as in mFrontiers.
        std::vector<std::pair<unsigned int, unsigned int>> mCenterOfFrontier; 

        // Path planning
        std::vector<int> tsp_path;
        std::vector<bool> mIsLoop;  // Indicator whether each vertex in tsp_path is loop closure point
        int currGoalIdx;
        bool mExplorationButton;
        bool getTspPlan;
        bool mReachGoal;    // Indicator whether a vertex in prior graph is reached
        bool useLocalReplanning; // Flag whether to use local path replanning after edge update

        // Active loop closing
        bool mLoopClosing;  // State indicator for: (1) going to closure point, (2) performing reliable closing
        std::vector<std::pair<double, double>> mClosingPath;  // Follow this path for reliable loop closing
        int mCurrClosingIdx; // Current goal index in mClosingPath for realiable loop closing
        std::set<int> mCoveredVertices;  // Record vertices that is reachable in current map, but have no frontier

        double mPlanTimeRecord;  // Record the time used in planning
        std::string mSavePlanTimePath;



};

#endif // MINPOSPLANNER_H_
