#include "node_edge_navigator/local_goal_creator.h"

LocalGoalCreator::LocalGoalCreator(void)
:local_nh("~")
{
	map_sub = nh.subscribe("/local_map", 1, &LocalGoalCreator::MapCallback, this);
	target_sub = nh.subscribe("/direction/relative", 1, &LocalGoalCreator::TargetCallback, this);

	local_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/local_goal",1);

	map_received = false;
}

float LocalGoalCreator::get_yaw(geometry_msgs::Quaternion q)
{
	double r, p, y;
	tf::Quaternion quat(q.x, q.y, q.z, q.w);
	tf::Matrix3x3(quat).getRPY(r, p,y);
	return y;
}

void LocalGoalCreator::MapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
	local_map = *msg;
	map_received = true;
}

void LocalGoalCreator::TargetCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
	geometry_msgs::PoseStamped target = *msg;
	target_orientation = get_yaw(target.pose.orientation);

	geometry_msgs::PoseStamped local_goal;
	if(map_received){
		detection_main(local_goal);
		std::cout << "local goal:" << std::endl;
		std::cout << local_goal.pose.position << std::endl;
		local_goal_pub.publish(local_goal);
	}
	else{
		std::cout << "wating for map..." << std::endl;
	}
}

void LocalGoalCreator::detection_main(geometry_msgs::PoseStamped& goal)
{
	goal.header = local_map.header;
	goal.pose.position.x = 5.0*cos(target_orientation); 
	goal.pose.position.y = 5.0*sin(target_orientation); 
	goal.pose.position.z = 0.0;
	goal.pose.orientation = tf::createQuaternionMsgFromYaw(target_orientation);
}

void LocalGoalCreator::process(void)
{
	std::cout << "=== local_goal_creator ===" << std::endl;
	ros::spin();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "local_goal_creator");
	LocalGoalCreator local_goal_creator;
	local_goal_creator.process();
	return 0;
}
