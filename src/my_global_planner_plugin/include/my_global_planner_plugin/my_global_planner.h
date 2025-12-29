// include/my_global_planner_plugin/my_global_planner.h
#ifndef MY_GLOBAL_PLANNER_H
#define MY_GLOBAL_PLANNER_H

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>

namespace my_global_planner_plugin {

class MyGlobalPlanner : public nav_core::BaseGlobalPlanner {
public:
    // 构造函数
    MyGlobalPlanner();
    MyGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    
    // 继承的虚函数
    virtual void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    
    virtual bool makePlan(const geometry_msgs::PoseStamped& start,
                          const geometry_msgs::PoseStamped& goal,
                          std::vector<geometry_msgs::PoseStamped>& plan);
    
    // 析构函数
    virtual ~MyGlobalPlanner();

private:
    ros::NodeHandle nh_;
    ros::Subscriber path_sub_;
    nav_msgs::Path custom_path_;
    bool path_received_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    std::string name_;
    tf::TransformListener* tf_;
    
    // 回调函数，订阅你的RRT*路径
    void pathCallback(const nav_msgs::Path::ConstPtr& path);
    
    // 发布路径用于调试
    ros::Publisher debug_path_pub_;
};

}  // namespace my_global_planner_plugin

#endif