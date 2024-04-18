#include <fstream>
#include <sstream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav2d_msgs/RobotPose.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>


#include <Eigen/Core>
#include <Eigen/Geometry>


ros::Publisher pubPathGT;
nav_msgs::Path pathGT;

ros::Publisher pubPathSLAM;
nav_msgs::Path oldSLAMPath;

ros::Publisher pubOdometry;
nav_msgs::Path pathOdom;



std::string gt_save_path;
std::string slam_save_path;

std::string robot_position;
std::vector<double> robot_position_vector;

// ros::Publisher pubSlamGtPath;
// nav_msgs::Path pathSlamGt;

// geometry_msgs::PoseStamped lastGT;



void PathGTHandler(const nav_msgs::Odometry::ConstPtr &odometryGT) {
    // 存储为TUM格式： timestamp x y z q_x q_y q_z q_w
    // timestamp is a float number, indicating the time passes from Unix epoch
    geometry_msgs::PoseStamped poseGT;
    poseGT.header = odometryGT->header;
    poseGT.pose = odometryGT->pose.pose;
    poseGT.pose.position.x -= robot_position_vector[0];
    poseGT.pose.position.y -= robot_position_vector[1];
    poseGT.pose.position.z -= robot_position_vector[2];


    Eigen::Quaterniond quat(poseGT.pose.orientation.w,
                        poseGT.pose.orientation.x,
                        poseGT.pose.orientation.y,
                        poseGT.pose.orientation.z);
    Eigen::Translation3d translation(0, 0, 0);
    Eigen::Isometry3d pose_transform = translation * quat;

    Eigen::Vector3d point_local(0.2, 0.0, 0.0);   // 这里雷达相对于机身的偏移量
    Eigen::Vector3d point_global = pose_transform * point_local;

    poseGT.pose.position.x += point_global.x();
    poseGT.pose.position.y += point_global.y();
    // poseGT.pose.position.x += point_global.x();

    // Directly change frame_id from odom to map
    // We have already use robot_position_vector to handle the origin of odom and map is not coincide
    poseGT.header.frame_id = "map";
    pathGT.poses.push_back(poseGT);
    pathGT.header.stamp = odometryGT->header.stamp;
    pubPathGT.publish(pathGT);

    // write ground truth trajectory to a txt file
    // Add a new line into the output txt file
    std::ofstream outfile(gt_save_path, std::ios_base::app); // append
    if (outfile.is_open()) {
        outfile << poseGT.header.stamp << " "
                << poseGT.pose.position.x << " "
                << poseGT.pose.position.y << " "
                << poseGT.pose.position.z << " "
                << poseGT.pose.orientation.x << " "
                << poseGT.pose.orientation.y << " "
                << poseGT.pose.orientation.z << " "
                << poseGT.pose.orientation.w << std::endl;
        outfile.close(); // Close file
    }
    else {
        std::cout << "Unable to open file." << std::endl;
    }
    
    return;
}

void PathSlamHandler(const nav_msgs::Path::ConstPtr &slamPath){
    // write slam trajectory to a txt file. Note this requires slam_path should have different timestamp for each pose
    // Get timestamp from pathSLAM, and store the corrected pose
    int num_pose;
    if(oldSLAMPath.poses.size() < slamPath->poses.size())
        num_pose = static_cast<int>(oldSLAMPath.poses.size());
    else
        num_pose = static_cast<int>(slamPath->poses.size());

    std::ofstream outfile(slam_save_path, std::ios::out | std::ios::trunc); // append
    if (outfile.is_open()) {
        for(int i = 0; i < num_pose; i++){
            outfile << oldSLAMPath.poses[i].header.stamp << " "
                    << slamPath->poses[i].pose.position.x << " "
                    << slamPath->poses[i].pose.position.y << " "
                    << slamPath->poses[i].pose.position.z << " "
                    << slamPath->poses[i].pose.orientation.x << " "
                    << slamPath->poses[i].pose.orientation.y << " "
                    << slamPath->poses[i].pose.orientation.z << " "
                    << slamPath->poses[i].pose.orientation.w << std::endl;
        }    
        outfile.close(); // Close file
    }
    else {
        std::cout << "Unable to open file." << std::endl;
    }
    return;

}


void PoseSlamHandler(const nav2d_msgs::RobotPose::ConstPtr &poseSLAM) {
    geometry_msgs::PoseStamped copyPoseSLAM;
    copyPoseSLAM.header = poseSLAM->header;
    copyPoseSLAM.pose.position.x = poseSLAM->pose.x;
    copyPoseSLAM.pose.position.y = poseSLAM->pose.y;
    copyPoseSLAM.pose.position.z = 0;
    copyPoseSLAM.pose.orientation = tf::createQuaternionMsgFromYaw(poseSLAM->pose.theta);

    oldSLAMPath.header.stamp = poseSLAM->header.stamp;
    oldSLAMPath.poses.push_back(copyPoseSLAM);
    pubPathSLAM.publish(oldSLAMPath);

}

void PathOdomHandler(const nav_msgs::Odometry::ConstPtr &stage_odom){
    geometry_msgs::PoseStamped poseOdom;
    poseOdom.header = stage_odom->header;
    poseOdom.pose = stage_odom->pose.pose;
    poseOdom.header.frame_id = "map";

    pathOdom.header.stamp = stage_odom->header.stamp;
    pathOdom.poses.push_back(poseOdom);
    pubOdometry.publish(pathOdom);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "pubPath");
    ros::NodeHandle n;

    pathGT.header.frame_id = "/map";
    oldSLAMPath.header.frame_id = "/map"; // slam节点publish robot_pose的frame_id是/map
    pathOdom.header.frame_id = "/map";

    pubPathGT = n.advertise<nav_msgs::Path>("/path_gt", 5);
    pubPathSLAM = n.advertise<nav_msgs::Path>("/path_slam", 5);  // 发布了旧的slam轨迹
    pubOdometry = n.advertise<nav_msgs::Path>("/path_odom", 5);

    ros::Subscriber subOdometryGT = n.subscribe<nav_msgs::Odometry>("/base_pose_ground_truth", 100, PathGTHandler);
    ros::Subscriber subOdometrySLAM = n.subscribe<nav2d_msgs::RobotPose>("/others", 100, PoseSlamHandler);
    ros::Subscriber subStageOdom = n.subscribe<nav_msgs::Odometry>("/odom", 100, PathOdomHandler);
    // Subscribe SLAM trajectory
    ros::Subscriber subPathSLAM = n.subscribe<nav_msgs::Path>("slam_path", 5, PathSlamHandler);

    ros::param::get("/pubPath/gt_save_path", gt_save_path);
    ros::param::get("/pubPath/slam_save_path", slam_save_path);

    ros::param::get("/pubPath/robot_position", robot_position);

    std::stringstream ss(robot_position);
    double value;
    while (ss >> value) {
        robot_position_vector.push_back(value);
    }

    // Clear contents in gt_traj.txt and slam_traj.txt
    std::ofstream file1(gt_save_path, std::ios::trunc);
    file1.close();
    std::ofstream file2(slam_save_path, std::ios::trunc);
    file2.close();

    ROS_INFO("Publish path GT node is running...");

    ros::spin();
    
}