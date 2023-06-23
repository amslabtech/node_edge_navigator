/**
 * @file   local_goal_creator.h
 * @brief  header file of local goal creator
 * @author AMSL ibuki1805
 * @date   2021/11/06
 */

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

    /**
     * @brief Constructor
     */
    LocalGoalCreator(void);

    /**
     * @brief Beginning of the local goal creator node
     */
    void process(void);

    /**
     * @breif Receive a local map (from localmap_creator/src/occupancygrid_integrate.cpp)
     * @param[in] Surrounding obstacle data (2D grid map data [0 or 100 or -1] )
     */
    void MapCallback(const nav_msgs::OccupancyGridConstPtr&);

    /**
     * @brief Receive a target node (from node_edge_navigator/src/node_edge_navigator.cpp) , and publish a local goal
     * @param[in] Target node position and orientation data
     */
    void TargetCallback(const geometry_msgs::PoseStampedConstPtr&);

    /**
     * @brief Control the generation of local goal
     * @param[in] Service server request (bool data) and response addresse
     * @param[out] Service server response (true)
     */
    bool TaskHandler(std_srvs::SetBool::Request&, std_srvs::SetBool::Response&);

private:
    ros::NodeHandle nh;
    ros::NodeHandle local_nh;

    ros::Subscriber map_sub;              // subscriber of local map
    ros::Subscriber target_sub;           // subscriber of target node(position and orientation)
    ros::Publisher local_goal_pub;        // publisher of local goal
    ros::Publisher local_goal_array_pub;  // publisher of local goal array
    ros::ServiceServer task_server;       // To use the service server, Service servers are used to pass information at any given time.

    /**
     * @brief Convert a quaternion to a yaw angle
     * @param[in] Quaternion
     * @param[out] Yaw angle
     */
    float get_yaw(geometry_msgs::Quaternion);

    /**
     * @brief Create and update a local goal
     * @param[in] target node, local goal
     */
    void detection_main(const geometry_msgs::PoseStamped&, geometry_msgs::PoseStamped&);

    nav_msgs::OccupancyGrid local_map;  // Empty box to receive local map
    bool map_received;                  // Indicates whether a local map is received
    bool task_flag;                     // Indicates whether the task is being performed
    double GOAL_DIS;                    // Not used
    double LOCAL_GOAL_ANGLE;            // Maximum absolute value
    double D_LOCAL_GOAL_ANGLE;          // Increment
};

#endif
