#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>

#include "amsl_navigation_msgs/Node.h"
#include "amsl_navigation_msgs/Edge.h"
#include "amsl_navigation_msgs/NodeEdgeMap.h"

class NodeEdgeNavigator
{
public:
	NodeEdgeNavigator(void);

	void map_callback(const amsl_navigation_msgs::NodeEdgeMapConstPtr&);
	void path_callback(const std_msgs::Int32MultiArrayConstPtr&);
	void pose_callback(const geometry_msgs::PoseStampedConstPtr&);
	void edge_callback(const amsl_navigation_msgs::EdgeConstPtr&);

private:
	ros::NodeHandle nh;
	ros::NodeHandle private_nh;

	ros::Publisher direction_pub;
	ros::Subscriber map_sub;
	ros::Subscriber path_sub;
	ros::Subscriber pose_sub;
	ros::Subscriber edge_sub;

	tf::TransformListener listener;
	tf::StampedTransform transform;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "node_edge_navigator");
	NodeEdgeNavigator node_edge_navigator;
	ros::spin();
	return 0;
}

NodeEdgeNavigator::NodeEdgeNavigator(void)
	: private_nh("~")
{
	direction_pub = nh.advertise<geometry_msgs::PoseStamped>("/direction/relative", 1);
	map_sub = nh.subscribe("/node_edge_map", 1, &NodeEdgeNavigator::map_callback, this);
	path_sub = nh.subscribe("/global_path", 1, &NodeEdgeNavigator::path_callback, this);
	pose_sub = nh.subscribe("/estimated_pose/pose", 1, &NodeEdgeNavigator::pose_callback, this);
	edge_sub = nh.subscribe("/estimated_pose/edge", 1, &NodeEdgeNavigator::edge_callback, this);

	std::cout << "=== node_edge_navigator ===" << std::endl;
}

void NodeEdgeNavigator::map_callback(const amsl_navigation_msgs::NodeEdgeMapConstPtr& msg)
{

}

void NodeEdgeNavigator::path_callback(const std_msgs::Int32MultiArrayConstPtr& msg)
{

}

void NodeEdgeNavigator::pose_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{

}

void NodeEdgeNavigator::edge_callback(const amsl_navigation_msgs::EdgeConstPtr& msg)
{

}
