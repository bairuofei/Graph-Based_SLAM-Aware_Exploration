/*
 * @Author: ruofei_ntu 991609404@qq.com
 * @Date: 2023-05-18 10:35:38
 * @LastEditors: ruofei_ntu 991609404@qq.com
 * @LastEditTime: 2024-01-09 19:32:54
 * @FilePath: /Graph-Based_SLAM-Aware_Exploration/src/exploration_plugins.cpp
 * @Description: 
 * 
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
 */
#include <pluginlib/class_list_macros.h>
#include <nav2d_navigator/ExplorationPlanner.h>

#include "MyPlanner.h"

// Declare class MyPlanner as a clsss ExplorartionPlanner
PLUGINLIB_EXPORT_CLASS(MyPlanner, ExplorationPlanner)
