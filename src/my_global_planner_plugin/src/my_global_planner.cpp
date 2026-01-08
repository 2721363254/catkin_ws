// src/my_global_planner.cpp
#include "my_global_planner_plugin/my_global_planner.h"
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

namespace my_global_planner_plugin {

std::vector<geometry_msgs::PoseStamped> MyGlobalPlanner::short_hold_plan(const geometry_msgs::PoseStamped& start,
                const std::string& global_frame)
{
    std::vector<geometry_msgs::PoseStamped> plan;

    geometry_msgs::PoseStamped p1 = start;
    geometry_msgs::PoseStamped p2 = start;

    p1.header.frame_id = global_frame;
    p2.header.frame_id = global_frame;

    p1.header.stamp = ros::Time::now();
    p2.header.stamp = ros::Time::now();

    // 关键：第二个点稍微“离开当前位置”
    // 不能太远，否则 local planner 会 empty
    // 不能太近，否则 isGoalReached = true
    const double hold_dist = 1.0;  // 推荐 0.5 ~ 1.5 m

    p2.pose.position.x += hold_dist;

    // 朝向无所谓，但给一个合法值
    p2.pose.orientation.w = 1.0;

    plan.push_back(p1);
    plan.push_back(p2);

    return plan;
}

void MyGlobalPlanner::append_local_fake_tail(
    std::vector<geometry_msgs::PoseStamped>& plan,
    double tail_dist)
{
    if (plan.empty()) return;

    geometry_msgs::PoseStamped tail = plan.back();

    // 沿路径末端方向延伸，而不是世界坐标 +x
    double yaw = tf2::getYaw(tail.pose.orientation);

    tail.pose.position.x += tail_dist * std::cos(yaw);
    tail.pose.position.y += tail_dist * std::sin(yaw);

    tail.header.stamp = ros::Time::now();
    tail.pose.orientation.w = 1.0;

    plan.push_back(tail);
}

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
bool MyGlobalPlanner::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan)
{
    plan.clear();

    const std::string global_frame = costmap_ros_->getGlobalFrameID();

    // ========= 情况 1：还没收到外部路径 =========
    if (!path_received_) {
        ROS_WARN_THROTTLE(1.0, "Waiting for /rrt_star_fn/path ...");

        // 占位 plan（短、局部）
        plan = short_hold_plan(start, "map");
        return true;
    }

        // ========= 情况 2：已经收到外部路径 =========
    if (custom_path_.poses.size() < 2) {
        ROS_ERROR("Custom path too short");
        return false;
    }

    if (path_received_ && !plan_sent_) {
            // 拷贝真实路径
        for (auto pose : custom_path_.poses) {
            pose.header.frame_id = global_frame;
            pose.header.stamp = ros::Time::now();
            plan.push_back(pose);
        }
        append_local_fake_tail(plan);
        plan_sent_ = true;
        ROS_INFO_THROTTLE(1.0, "Return external path, size=%lu", plan.size());
        return true;
    }
}




}  // namespace my_global_planner_plugin

// 注册插件
PLUGINLIB_EXPORT_CLASS(my_global_planner_plugin::MyGlobalPlanner, nav_core::BaseGlobalPlanner)