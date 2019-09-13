#ifndef __LOCAL_GOAL_CREATOR_H
#define __LOCAL_GOAL_CREATOR_H

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

class LocalGoalCreator
{
public:
	LocalGoalCreator(void);

	void process(void);
	void MapCallback(const nav_msgs::OccupancyGridConstPtr&);
	void TargetCallback(const geometry_msgs::PoseStampedConstPtr&);

private:
	ros::NodeHandle nh;
	ros::NodeHandle local_nh;

	ros::Subscriber map_sub;
	ros::Subscriber target_sub;
	ros::Publisher local_goal_pub;

	float get_yaw(geometry_msgs::Quaternion);
	void detection_main(const geometry_msgs::PoseStamped&, geometry_msgs::PoseStamped&);

	nav_msgs::OccupancyGrid local_map;
	bool map_received;
	double GOAL_DIS;
};

#endif
