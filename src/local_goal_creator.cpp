#include "node_edge_navigator/local_goal_creator.h"

LocalGoalCreator::LocalGoalCreator(void)
:local_nh("~")
{
    local_nh.param("GOAL_DIS", GOAL_DIS, {5.0});
    local_nh.param("LOCAL_GOAL_ANGLE", LOCAL_GOAL_ANGLE, {M_PI / 3.0});
    local_nh.param("D_LOCAL_GOAL_ANGLE", D_LOCAL_GOAL_ANGLE, {M_PI / 18.0});

    map_sub = nh.subscribe("/local_map", 1, &LocalGoalCreator::MapCallback, this);
    target_sub = nh.subscribe("/direction/relative", 1, &LocalGoalCreator::TargetCallback, this);
    task_server = nh.advertiseService("/local_goal/interruption", &LocalGoalCreator::TaskHandler, this);

    local_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/local_goal",1);
    local_goal_array_pub = nh.advertise<geometry_msgs::PoseArray>("/local_goal/array", 1);

    map_received = false;
    task_flag = false;
}

float LocalGoalCreator::get_yaw(geometry_msgs::Quaternion q)
{
    double r, p, y;
    tf::Quaternion quat(q.x, q.y, q.z, q.w);
    tf::Matrix3x3(quat).getRPY(r, p,y);
    return y;
}

bool LocalGoalCreator::TaskHandler(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
{
    task_flag = request.data;
    response.success = true;
    return true;
}

void LocalGoalCreator::MapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    local_map = *msg;
    map_received = true;
}

void LocalGoalCreator::TargetCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    geometry_msgs::PoseStamped target = *msg;

    geometry_msgs::PoseStamped local_goal;
    if(map_received && !task_flag){
        detection_main(target, local_goal);
        std::cout << "local goal:" << std::endl;
        std::cout << local_goal.pose.position << std::endl;
        local_goal_pub.publish(local_goal);
    }
    else{
        std::cout << "wating for map..." << std::endl;
    }
}

void LocalGoalCreator::detection_main(const geometry_msgs::PoseStamped& target, geometry_msgs::PoseStamped& goal)
{
    goal.header = local_map.header;
    double target_distance = sqrt(target.pose.position.x * target.pose.position.x + target.pose.position.y * target.pose.position.y);
    std::cout << "target distance: " << target_distance << std::endl;
    double target_orientation = get_yaw(target.pose.orientation);
    std::cout << "target direction\n" << target_orientation << std::endl;
    const int ANGLE_SAMPLE_NUM = 2 * round(LOCAL_GOAL_ANGLE / D_LOCAL_GOAL_ANGLE) + 1;
    double goal_distance = 0;
    double goal_direction = 1e6;
    std::cout << "ANGLE_SAMPLE_NUM: " << ANGLE_SAMPLE_NUM << std::endl;
    if(ANGLE_SAMPLE_NUM > 0){
        geometry_msgs::PoseArray local_goal_array;
        local_goal_array.header = goal.header;
        for(int i=0;i<ANGLE_SAMPLE_NUM;i++){
            double angle = -LOCAL_GOAL_ANGLE + i * D_LOCAL_GOAL_ANGLE + target_orientation;
            std::cout << "angle: " << angle << std::endl;
            double distance = 0;
            for(;distance<=target_distance;distance+=local_map.info.resolution){
                int x_grid = round((distance * cos(angle) - local_map.info.origin.position.x) / local_map.info.resolution);
                int y_grid = round((distance * sin(angle) - local_map.info.origin.position.y) / local_map.info.resolution);
                if(local_map.data[x_grid + local_map.info.width * y_grid] != 0){
                    distance = std::max(0.0, distance - local_map.info.resolution);
                    break;
                }
            }
            geometry_msgs::Pose p;
            p.position.x = distance * cos(angle);
            p.position.y = distance * sin(angle);
            p.orientation = tf::createQuaternionMsgFromYaw(angle);
            local_goal_array.poses.push_back(p);
            std::cout << "distance: " << distance << std::endl;
            if(goal_distance < distance){
                goal_distance = distance;
                goal_direction = angle;
                std::cout << "updated!" << std::endl;
                std::cout << "goal_distance: " << goal_distance << std::endl;
                std::cout << "goal_direction: " << goal_direction << std::endl;
            }else if(goal_distance == distance){
                if(fabs(goal_direction - target_orientation) > fabs(angle - target_orientation)){
                    goal_direction = angle;
                    std::cout << "updated!" << std::endl;
                    std::cout << "goal_distance: " << goal_distance << std::endl;
                    std::cout << "goal_direction: " << goal_direction << std::endl;
                }
            }
        }
        local_goal_array_pub.publish(local_goal_array);
    }else{
        double angle = target_orientation;
        std::cout << "angle: " << angle << std::endl;
        double distance = 0;
        for(;distance<=target_distance;distance+=local_map.info.resolution){
            int x_grid = round((distance * cos(angle) - local_map.info.origin.position.x) / local_map.info.resolution);
            int y_grid = round((distance * sin(angle) - local_map.info.origin.position.y) / local_map.info.resolution);
            if(local_map.data[x_grid + local_map.info.width * y_grid] != 0){
                distance = std::max(0.0, distance - local_map.info.resolution);
                break;
            }
        }
        std::cout << "distance: " << distance << std::endl;
        if(goal_distance <= distance){
            if(fabs(goal_direction - target_orientation) > fabs(angle - target_orientation)){
                goal_distance = distance;
                goal_direction = angle;
                std::cout << "updated!" << std::endl;
                std::cout << "goal_distance: " << goal_distance << std::endl;
                std::cout << "goal_direction: " << goal_direction << std::endl;
            }
        }
    }
    goal.pose.position.x = goal_distance * cos(goal_direction);
    goal.pose.position.y = goal_distance * sin(goal_direction);
    goal.pose.position.z = 0.0;
    goal.pose.orientation = tf::createQuaternionMsgFromYaw(goal_direction);
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
