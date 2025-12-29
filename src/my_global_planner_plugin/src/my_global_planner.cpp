// src/my_global_planner.cpp
#include "my_global_planner_plugin/my_global_planner.h"
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

namespace my_global_planner_plugin {

// 默认构造函数
MyGlobalPlanner::MyGlobalPlanner() : 
    path_received_(false), 
    costmap_ros_(nullptr),
    tf_(nullptr) {
}

// 带参数的构造函数
MyGlobalPlanner::MyGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) : 
    path_received_(false),
    costmap_ros_(nullptr),
    tf_(nullptr) {
    initialize(name, costmap_ros);
}

// 析构函数
MyGlobalPlanner::~MyGlobalPlanner() {
    if (tf_) delete tf_;
}

// 初始化函数
void MyGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    // 保存参数
    name_ = name;
    costmap_ros_ = costmap_ros;
    
    // 初始化TF监听器
    tf_ = new tf::TransformListener(ros::Duration(10));
    
    // 订阅你的RRT*路径
    path_sub_ = nh_.subscribe("/rrt_star_fn/path", 1, &MyGlobalPlanner::pathCallback, this);
    
    // 创建调试发布器
    debug_path_pub_ = nh_.advertise<nav_msgs::Path>("/my_global_planner/debug_path", 1);
    
    ROS_INFO("MyGlobalPlanner plugin initialized with name: %s", name_.c_str());
    
    // 如果代价地图为空，给出警告
    if (!costmap_ros_) {
        ROS_WARN("Costmap not provided to MyGlobalPlanner. Some features may not work.");
    }
}

// 路径回调函数
void MyGlobalPlanner::pathCallback(const nav_msgs::Path::ConstPtr& path) {
    if (!path || path->poses.empty()) {
        ROS_WARN("Received empty or invalid path");
        return;
    }
    
    custom_path_ = *path;
    path_received_ = true;
    
    ROS_DEBUG("Received new path with %zu poses", custom_path_.poses.size());
    
    // 发布调试信息
    debug_path_pub_.publish(custom_path_);
}

// 规划函数
bool MyGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                               const geometry_msgs::PoseStamped& goal,
                               std::vector<geometry_msgs::PoseStamped>& plan) {
    ROS_INFO("MyGlobalPlanner::makePlan called from [%s] to [%s]", 
             start.header.frame_id.c_str(), goal.header.frame_id.c_str());
    
    // 检查是否有自定义路径可用
    if (path_received_) {
        // 清空输出路径
        plan.clear();
        
        // 将自定义路径复制到plan中
        for (const auto& pose : custom_path_.poses) {
            plan.push_back(pose);
        }
        
        ROS_INFO("Returning custom path with %zu points", plan.size());
        return !plan.empty();
    }

    return false;
}

}  // namespace my_global_planner_plugin

// 注册插件
PLUGINLIB_EXPORT_CLASS(my_global_planner_plugin::MyGlobalPlanner, nav_core::BaseGlobalPlanner)