#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>

nav_msgs::OccupancyGrid local_map;
bool map_received = false;
bool target_received = false;
float target_orientation = 0.0;
double longest_path_length_angle = 0.0;

void TargetCallback(const std_msgs::Float64ConstPtr& msg)
{
  target_orientation = msg->data;
  target_received = true;
}

void MapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  local_map = *msg;
  map_received = true;
}

void detection_main(geometry_msgs::PoseStamped& goal)
{
	goal.header = local_map.header;
	goal.pose.position.x = 5.0*cos(target_orientation); 
	goal.pose.position.y = 5.0*sin(target_orientation); 
	goal.pose.position.z = 0.0;
	goal.pose.orientation = tf::createQuaternionMsgFromYaw(target_orientation);
}

void LocalGoalCreator()
{
  ros::NodeHandle nh;
  ros::Subscriber sub_map = nh.subscribe("/local_map", 1, MapCallback);
  ros::Subscriber sub_target = nh.subscribe("/direction/relative", 1, TargetCallback);
  
  ros::Publisher pub_goal = nh.advertise<geometry_msgs::PoseStamped>("/local_goal",1);

  geometry_msgs::PoseStamped local_goal;

  ros::Rate loop_rate(10);
  while(ros::ok()){
	if(map_received && target_received){
	  detection_main(local_goal);
	  pub_goal.publish(local_goal);
	}
	else{
	  std::cout << "map:" << map_received << " target:" << target_received << std::endl;
	}
	ros::spinOnce();
	loop_rate.sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "local_goal_creator");
  LocalGoalCreator();
  ROS_INFO("Killing now!!!!!");
  return 0;
}
