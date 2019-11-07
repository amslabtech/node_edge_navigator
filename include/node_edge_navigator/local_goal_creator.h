#ifndef __LOCAL_GOAL_CREATOR_H
#define __LOCAL_GOAL_CREATOR_H

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_srvs/SetBool.h>
#include <tf/tf.h>

class LocalGoalCreator
{
public:
    LocalGoalCreator(void);

    void process(void);
    void MapCallback(const nav_msgs::OccupancyGridConstPtr&);
    void TargetCallback(const geometry_msgs::PoseStampedConstPtr&);
    bool TaskHandler(std_srvs::SetBool::Request&, std_srvs::SetBool::Response&);

private:
    ros::NodeHandle nh;
    ros::NodeHandle local_nh;

    ros::Subscriber map_sub;
    ros::Subscriber target_sub;
    ros::Publisher local_goal_pub;
    ros::Publisher local_goal_array_pub;
    ros::ServiceServer task_server;

    float get_yaw(geometry_msgs::Quaternion);
    void detection_main(const geometry_msgs::PoseStamped&, geometry_msgs::PoseStamped&);

    nav_msgs::OccupancyGrid local_map;
    bool map_received;
    bool task_flag;
    double GOAL_DIS;
    double LOCAL_GOAL_ANGLE;
    double D_LOCAL_GOAL_ANGLE;
};

#endif
