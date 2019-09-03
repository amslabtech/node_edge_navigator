#ifndef __NODE_EDGE_NAVIGATOR_H
#define __NODE_EDGE_NAVIGATOR_H

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include "amsl_navigation_msgs/Node.h"
#include "amsl_navigation_msgs/Edge.h"
#include "amsl_navigation_msgs/NodeEdgeMap.h"
#include "amsl_navigation_msgs/UpdateNode.h"
#include "amsl_navigation_msgs/UpdateEdge.h"
#include "amsl_navigation_msgs/Replan.h"

class NodeEdgeNavigator
{
public:
    NodeEdgeNavigator(void);

    void map_callback(const amsl_navigation_msgs::NodeEdgeMapConstPtr&);
    void path_callback(const std_msgs::Int32MultiArrayConstPtr&);
    void pose_callback(const nav_msgs::OdometryConstPtr&);
    void edge_callback(const amsl_navigation_msgs::EdgeConstPtr&);
    void intersection_flag_callback(const std_msgs::BoolConstPtr&);
    void process(void);
    void get_node_from_id(int, amsl_navigation_msgs::Node&);
    double pi_2_pi(double);
    void request_replanning(void);
    void arrived_at_node(void);
    double get_distance_from_points(const geometry_msgs::Point&, const geometry_msgs::Point&);

private:
    double HZ;
    double EXCESS_DETECTION_RATIO;
    double GOAL_RADIUS;
    bool ENABLE_REQUESTING_REPLANNING;
    double INTERSECTION_ACCEPTANCE_PROGRESS_RATIO;

    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    ros::Publisher direction_pub;
    ros::Subscriber map_sub;
    ros::Subscriber path_sub;
    ros::Subscriber pose_sub;
    ros::Subscriber edge_sub;
    ros::Subscriber intersection_flag_sub;
    ros::ServiceClient node_client;
    ros::ServiceClient edge_client;
    ros::ServiceClient replan_client;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    amsl_navigation_msgs::NodeEdgeMap map;
    std::vector<int> global_path_ids;
    nav_msgs::Odometry estimated_pose;
    amsl_navigation_msgs::Edge estimated_edge;

    bool map_subscribed;
    bool global_path_subscribed;
    bool pose_updated;
    bool edge_updated;
    std::vector<int> passed_paths;
    bool intersection_flag;
    int last_target_node_id;
    bool first_edge_sub_flag;
    int global_path_index;
};

#endif// __NODE_EDGE_NAVIGATOR_H
