#include "node_edge_navigator/node_edge_navigator.h"

NodeEdgeNavigator::NodeEdgeNavigator(void)
    : private_nh("~")
{
    direction_pub = nh.advertise<geometry_msgs::PoseStamped>("/direction/relative", 1);
    map_sub = nh.subscribe("/node_edge_map/map", 1, &NodeEdgeNavigator::map_callback, this);
    path_sub = nh.subscribe("/global_path/path", 1, &NodeEdgeNavigator::path_callback, this);
    pose_sub = nh.subscribe("/estimated_pose/pose", 1, &NodeEdgeNavigator::pose_callback, this);
    edge_sub = nh.subscribe("/estimated_pose/edge", 1, &NodeEdgeNavigator::edge_callback, this);
    intersection_flag_sub = nh.subscribe("/intersection_flag", 1, &NodeEdgeNavigator::intersection_flag_callback, this);
    node_client = nh.serviceClient<amsl_navigation_msgs::UpdateNode>("/node_edge_map/update_node");
    edge_client = nh.serviceClient<amsl_navigation_msgs::UpdateEdge>("/node_edge_map/update_edge");
    replan_client = nh.serviceClient<amsl_navigation_msgs::Replan>("/global_path/replan");

    private_nh.param("HZ", HZ, {20});
    private_nh.param("EXCESS_DETECTION_RATIO", EXCESS_DETECTION_RATIO, {1.2});
    private_nh.param("GOAL_RADIUS", GOAL_RADIUS, {0.5});
    private_nh.param("ENABLE_REQUESTING_REPLANNING", ENABLE_REQUESTING_REPLANNING, {false});
    private_nh.param("INTERSECTION_ACCEPTANCE_PROGRESS_RATIO", INTERSECTION_ACCEPTANCE_PROGRESS_RATIO, {0.5});

    map_subscribed = false;
    global_path_subscribed = false;
    pose_updated = false;
    edge_updated = false;
    first_edge_sub_flag = true;
    global_path_index = 0;

    std::cout << "=== node_edge_navigator ===" << std::endl;
    std::cout << "HZ: " << HZ << std::endl;
    std::cout << "EXCESS_DETECTION_RATIO: " << EXCESS_DETECTION_RATIO << std::endl;
    std::cout << "GOAL_RADIUS: " << GOAL_RADIUS << std::endl;
    std::cout << "ENABLE_REQUESTING_REPLANNING: " << ENABLE_REQUESTING_REPLANNING << std::endl;
    std::cout << "INTERSECTION_ACCEPTANCE_PROGRESS_RATIO: " << INTERSECTION_ACCEPTANCE_PROGRESS_RATIO << std::endl;
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
    // reset
    global_path_index = 0;
    std::cout << "received global path" << std::endl;
}

void NodeEdgeNavigator::pose_callback(const nav_msgs::OdometryConstPtr& msg)
{
    estimated_pose = *msg;
    pose_updated = true;
}

void NodeEdgeNavigator::edge_callback(const amsl_navigation_msgs::EdgeConstPtr& msg)
{
    estimated_edge = *msg;
    edge_updated = true;
    if(first_edge_sub_flag){
        first_edge_sub_flag = false;
        last_target_node_id = estimated_edge.node0_id;
    }
    std::cout << "\nestimated edge:\n" << estimated_edge << std::endl;
}

void NodeEdgeNavigator::intersection_flag_callback(const std_msgs::BoolConstPtr& msg)
{
    intersection_flag = msg->data;
    std::cout << "received intersection_flag: " << intersection_flag << std::endl;
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
                int global_path_ids_num = global_path_ids.size();
                if(global_path_ids_num - global_path_index > 0){
                    // unexpected process
                    if(global_path_ids_num - global_path_index > 1){
                        if(global_path_ids[global_path_index] != estimated_edge.node1_id){
                            if(global_path_ids[global_path_index] == estimated_edge.node0_id && global_path_ids[global_path_index + 1] == estimated_edge.node1_id){
                                std::cout << "\033[32mnode " << global_path_ids[global_path_index] << " has been considered to be passed because the robot is on the target edge\033[0m" << std::endl;
                                last_target_node_id = global_path_ids[global_path_index];
                                global_path_index++;
                            }else if(last_target_node_id == estimated_edge.node0_id && global_path_ids[global_path_index] != estimated_edge.node1_id){
                                std::cout << "\033[31mmaybe navigation error\033[0m" << std::endl;
                                std::cout << "estimated edge: " << estimated_edge.node0_id << " -> " << estimated_edge.node1_id << std::endl;
                                std::cout << "desired edge: " << last_target_node_id << " -> " << global_path_ids[global_path_index] << std::endl;
                                ///////////////////////////////////
                                //  navigation recovery behavior //
                                ///////////////////////////////////
                            }
                        }
                    }
                    // ~unexpected process
                    geometry_msgs::PoseStamped direction;
                    amsl_navigation_msgs::Node target_node;
                    get_node_from_id(global_path_ids[global_path_index], target_node);
                    std::cout << "global path id: " << std::endl;
                    for(int i=0;i<global_path_ids_num;i++){
                        if(i != global_path_index){
                            std::cout << global_path_ids[i] << std::endl;;
                        }else{
                            std::cout << "\033[032m" << global_path_ids[i] << "<- current target\033[0m" << std::endl;;
                        }
                    }
                    std::cout << "target node: \n" << target_node << std::endl;
                    if((target_node.type == "position") || (target_node.type == "gps")){
                        double distance = get_distance_from_points(target_node.point, estimated_pose.pose.pose.position);
                        if(distance <= GOAL_RADIUS){
                            arrived_at_node();
                            // update target node
                            get_node_from_id(global_path_ids[global_path_index], target_node);
                        }
                        // caluculate target node direction
                        double global_node_direction = atan2(target_node.point.y - estimated_pose.pose.pose.position.y, target_node.point.x - estimated_pose.pose.pose.position.x);
                        double target_node_direction = global_node_direction - tf::getYaw(estimated_pose.pose.pose.orientation);
                        target_node_direction = pi_2_pi(target_node_direction);
                        direction.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, target_node_direction);
                    }else if(target_node.type == "intersection"){
                        amsl_navigation_msgs::Node last_target_node;
                        get_node_from_id(last_target_node_id, last_target_node);
                        std::cout << "last target node: \n" << last_target_node << std::endl;
                        if(intersection_flag){
                            intersection_flag = false;
                            if(estimated_edge.progress > INTERSECTION_ACCEPTANCE_PROGRESS_RATIO){
                                if(last_target_node_id != estimated_edge.node1_id){
                                    std::cout << "new intersection" << std::endl;
                                    get_node_from_id(last_target_node_id, last_target_node);
                                    arrived_at_node();
                                    // update target node
                                    get_node_from_id(global_path_ids[0], target_node);
                                }else{
                                    std::cout << "already arrived at the intersection" << std::endl;
                                }
                            }
                        }
                        // caluculate target node direction
                        double global_node_direction = atan2(target_node.point.y - last_target_node.point.y, target_node.point.x - last_target_node.point.x);
                        std::cout << "edge direction: " << global_node_direction << "[rad]" << std::endl;
                        double target_node_direction = global_node_direction - tf::getYaw(estimated_pose.pose.pose.orientation);
                        target_node_direction = pi_2_pi(target_node_direction);
                        direction.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, target_node_direction);
                        // excess detection
                        if((estimated_edge.progress >= EXCESS_DETECTION_RATIO) && ENABLE_REQUESTING_REPLANNING){
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
                    //std::cout << "target node:\n" << target_node << std::endl;
                    std::cout << "direction: " << tf::getYaw(direction.pose.orientation) << "[rad]" << std::endl;
                    direction.header = estimated_pose.header;
                    direction_pub.publish(direction);

                    pose_updated = false;
                    edge_updated = false;
                    std::cout << "process time: " << ros::Time::now().toSec() - start_time << "[s]" << std::endl;
                }else{
                    // goal
                    std::cout << "global path is empty" << std::endl;
                }
            }else{
                std::cout << "\033[1A" << "waiting for pose and edge estimation update" << std::endl;
            }
        }else{
            std::cout << "\033[1A" << "waiting for map and global path" << std::endl;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void NodeEdgeNavigator::get_node_from_id(int id, amsl_navigation_msgs::Node& node)
{
    for(const auto n : map.nodes){
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
    new_edge.node1_id = estimated_edge.node1_id;
    new_edge.distance = estimated_edge.distance * (estimated_edge.progress - 1);
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
    get_node_from_id(global_path_ids[global_path_index], node);
    std::cout << "\033[032m===================\033[0m" << std::endl;
    std::cout << "\033[032marrived at node: \n" << node << "\033[0m" << std::endl;
    std::cout << "\033[032m===================\033[0m" << std::endl;
    last_target_node_id = global_path_ids[global_path_index];
    global_path_index++;
}

double NodeEdgeNavigator::get_distance_from_points(const geometry_msgs::Point& p0, const geometry_msgs::Point& p1)
{
    return sqrt((p0.x - p1.x) * (p0.x - p1.x) + (p0.y - p1.y) * (p0.y - p1.y));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "node_edge_navigator");
    NodeEdgeNavigator node_edge_navigator;
    node_edge_navigator.process();
    return 0;
}

