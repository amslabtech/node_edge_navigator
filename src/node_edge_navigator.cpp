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

private:
	double HZ;
	double EXCESS_DETECTION_RATIO;

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
	amsl_navigation_msgs::Edge edge;

	bool map_subscribed;
	bool global_path_subscribed;
	bool pose_updated;
	bool edge_updated;
	std::vector<int> passed_paths;
	bool intersection_flag;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "node_edge_navigator");
	NodeEdgeNavigator node_edge_navigator;
	node_edge_navigator.process();
	return 0;
}

NodeEdgeNavigator::NodeEdgeNavigator(void)
	: private_nh("~")
{
	direction_pub = nh.advertise<geometry_msgs::PoseStamped>("/direction/relative", 1);
	map_sub = nh.subscribe("/node_edge_map/map", 1, &NodeEdgeNavigator::map_callback, this);
	path_sub = nh.subscribe("/global_path", 1, &NodeEdgeNavigator::path_callback, this);
	pose_sub = nh.subscribe("/estimated_pose/pose", 1, &NodeEdgeNavigator::pose_callback, this);
	edge_sub = nh.subscribe("/estimated_pose/edge", 1, &NodeEdgeNavigator::edge_callback, this);
	intersection_flag_sub = nh.subscribe("/intersection_flag", 1, &NodeEdgeNavigator::intersection_flag_callback, this);
	node_client = nh.serviceClient<amsl_navigation_msgs::UpdateNode>("/node_edge_map/update_node");
	edge_client = nh.serviceClient<amsl_navigation_msgs::UpdateEdge>("/node_edge_map/update_edge");
	replan_client = nh.serviceClient<amsl_navigation_msgs::Replan>("/global_path/replan");

	private_nh.param("HZ", HZ, {50});
	private_nh.param("EXCESS_DETECTION_RATIO", EXCESS_DETECTION_RATIO, {1.2});

	map_subscribed = false;
	global_path_subscribed = false;
	pose_updated = false;
	edge_updated = false;

	std::cout << "=== node_edge_navigator ===" << std::endl;
	std::cout << "HZ: " << HZ << std::endl;
	std::cout << "EXCESS_DETECTION_RATIO: " << EXCESS_DETECTION_RATIO << std::endl;
	std::cout << std::endl;
}

void NodeEdgeNavigator::map_callback(const amsl_navigation_msgs::NodeEdgeMapConstPtr& msg)
{
	map = *msg;
	map_subscribed = true;
	std::cout << "received map" << std::endl;
}

void NodeEdgeNavigator::path_callback(const std_msgs::Int32MultiArrayConstPtr& msg)
{
	global_path_ids = msg->data;
	global_path_subscribed = true;
}

void NodeEdgeNavigator::pose_callback(const nav_msgs::OdometryConstPtr& msg)
{
	estimated_pose = *msg;
	pose_updated = true;
}

void NodeEdgeNavigator::edge_callback(const amsl_navigation_msgs::EdgeConstPtr& msg)
{
	edge = *msg;
	edge_updated = true;
}

void NodeEdgeNavigator::intersection_flag_callback(const std_msgs::BoolConstPtr& msg)
{
	std::cout << "intersection" << std::endl;
	intersection_flag = msg->data;
}

void NodeEdgeNavigator::process(void)
{
	ros::Rate loop_rate(HZ);
	while(ros::ok()){
		if(map_subscribed && global_path_subscribed){
			// std::cout << pose_updated << ", " << edge_updated << std::endl;
			if(pose_updated && edge_updated){
				std::cout << "=== node_edge_navigator ===" << std::endl;
				double start_time = ros::Time::now().toSec();
				if(global_path_ids.empty()){
					// goal
				}
				geometry_msgs::PoseStamped direction;
				amsl_navigation_msgs::Node target_node;
				get_node_from_id(global_path_ids[0], target_node);
				std::cout << "target node: \n" << target_node << std::endl;
				if((target_node.type == "position") || (target_node.type == "gps")){
					// caluculate target node direction
					double global_node_direction = atan2(target_node.point.y - estimated_pose.pose.pose.position.y, target_node.point.x - estimated_pose.pose.pose.position.x);
					double target_node_direction = global_node_direction - tf::getYaw(estimated_pose.pose.pose.orientation);
					target_node_direction = pi_2_pi(target_node_direction);
					direction.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, target_node_direction);
				}else if(target_node.type == "intersection"){
					if(intersection_flag){
						intersection_flag = false;
						arrived_at_node();
						// update target node
						get_node_from_id(global_path_ids[0], target_node);
					}
					// caluculate target node direction
					amsl_navigation_msgs::Node last_node;
					get_node_from_id(edge.node0_id, last_node);
					double global_node_direction = atan2(target_node.point.y - last_node.point.y, target_node.point.x - last_node.point.x);
					std::cout << "edge direction: " << global_node_direction << "[rad]" << std::endl;
					double target_node_direction = global_node_direction - tf::getYaw(estimated_pose.pose.pose.orientation);
					target_node_direction = pi_2_pi(target_node_direction);
					direction.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, target_node_direction);
					// excess detection
					if(edge.progress >= EXCESS_DETECTION_RATIO){
						std::cout << "intersection excession is detected" << std::endl;
						request_replanning();
					}
				}else{
					// default
					double global_node_direction = atan2(target_node.point.y - estimated_pose.pose.pose.position.y, target_node.point.x - estimated_pose.pose.pose.position.x);
					double target_node_direction = global_node_direction - tf::getYaw(estimated_pose.pose.pose.orientation);
					target_node_direction = pi_2_pi(target_node_direction);
					direction.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, target_node_direction);
				}
				std::cout << "target node:\n" << target_node << std::endl;
				std::cout << "direction: " << tf::getYaw(direction.pose.orientation) << "[rad]" << std::endl;
				// direction.pose.position = estimated_pose.pose.pose.position;
				direction.header.frame_id = "base_link"; 
				direction.header.stamp = ros::Time::now();
				direction_pub.publish(direction);

				pose_updated = false;
				edge_updated = false;
				std::cout << "process time: " << ros::Time::now().toSec() - start_time << "[s]" << std::endl;
			}else{
				std::cout << "\033[1A" << "waiting for pose and edge estimation update" << std::endl;
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
}

void NodeEdgeNavigator::get_node_from_id(int id, amsl_navigation_msgs::Node& node)
{
	for(auto n : map.nodes){
		if(n.id == id){
			node = n;
			return;
		}
	}
}

double NodeEdgeNavigator::pi_2_pi(double angle)
{
	return atan2(sin(angle), cos(angle));
}

void NodeEdgeNavigator::request_replanning(void)
{
	amsl_navigation_msgs::Node node;
	int id = 1000;
	for(auto it=map.nodes.begin();it!=map.nodes.end();++it){
		if(it->id != id){
			break;
		}
		id++;
	}
	node.id = id;
	node.label = "replanning";
	node.type = "position";
	node.point = estimated_pose.pose.pose.position;
	amsl_navigation_msgs::UpdateNode node_service;
	node_service.request.node = node;
	node_service.request.operation = node_service.request.ADD;
	if(node_client.call(node_service)){
		std::cout << "node added to map" << std::endl;
	}else{
		std::cout << "failed to call service" << std::endl;
		return;
	}
	amsl_navigation_msgs::Edge new_edge;
	new_edge.node0_id = node.id;
	new_edge.node1_id = edge.node1_id;
	new_edge.distance = edge.distance * (edge.progress - 1);
	amsl_navigation_msgs::UpdateEdge edge_service;
	edge_service.request.edge = new_edge;
	edge_service.request.operation = edge_service.request.ADD;
	if(edge_client.call(edge_service)){
		std::cout << "edge added to map" << std::endl;
	}else{
		std::cout << "failed to call service" << std::endl;
		return;
	}
	amsl_navigation_msgs::Replan replan_service;
	replan_service.request.edge = new_edge;
	if(replan_client.call(replan_service)){
		std::cout << "replan succeeded" << std::endl;
	}else{
		std::cout << "failed to call service" << std::endl;
		return;
	}
}

void NodeEdgeNavigator::arrived_at_node(void)
{
	amsl_navigation_msgs::Node node;
	get_node_from_id(*(global_path_ids.begin()), node);
	std::cout << "arrived at node: \n" << node << std::endl;
	global_path_ids.erase(global_path_ids.begin());
	std::cout << "the begin element of global path was deleted" << std::endl;
}
