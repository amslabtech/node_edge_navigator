#include "node_edge_navigator/local_goal_creator.h"

LocalGoalCreator::LocalGoalCreator(void)
  : local_nh_("~")
{
  local_nh_.param("goal_dis", goal_dis_, {5.0});
  local_nh_.param("local_goal_angle", local_goal_angle_, {M_PI / 3.0});
  local_nh_.param("d_local_goal_angle", d_local_goal_angle_, {M_PI / 18.0});

  map_sub_ = nh_.subscribe("/local_map", 1, &LocalGoalCreator::MapCallback, this);
  target_sub_ = nh_.subscribe("/direction/relative", 1, &LocalGoalCreator::TargetCallback, this);
  task_server_ = nh_.advertiseService("/local_goal/interruption", &LocalGoalCreator::TaskHandler, this);

  local_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/local_goal", 1);
  local_goal_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/local_goal/array", 1);

  map_received_ = false;
  task_flag_ = false;
}

float LocalGoalCreator::get_yaw(geometry_msgs::Quaternion q)
{
  double r, p, y;
  tf::Quaternion quat(q.x, q.y, q.z, q.w);
  tf::Matrix3x3(quat).getRPY(r, p, y);
  return y;
}

bool LocalGoalCreator::TaskHandler(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
{
  task_flag_ = request.data;
  response.success = true;
  return true;
}

void LocalGoalCreator::MapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  local_map_ = *msg;
  map_received_ = true;
}

void LocalGoalCreator::TargetCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  geometry_msgs::PoseStamped target = *msg;

  geometry_msgs::PoseStamped local_goal;
  if (map_received_ && !task_flag_)
  {
    detection_main(target, local_goal);
    std::cout << "local goal:" << std::endl;
    std::cout << local_goal.pose.position << std::endl;
    local_goal_pub_.publish(local_goal);
  }
  else
  {
    std::cout << "wating for map..." << std::endl;
  }
}

void LocalGoalCreator::detection_main(const geometry_msgs::PoseStamped& target, geometry_msgs::PoseStamped& goal)
{
  goal.header = local_map_.header;
  double target_distance = sqrt(target.pose.position.x * target.pose.position.x + target.pose.position.y * target.pose.position.y);
  std::cout << "target distance: " << target_distance << std::endl;

  double target_orientation = get_yaw(target.pose.orientation);
  std::cout << "target direction\n"
            << target_orientation << std::endl;

  const int ANGLE_SAMPLE_NUM = 2 * round(local_goal_angle_ / d_local_goal_angle_) + 1;  // Number of samples to see obstacles in the vicinity, 2*(60/10)+1 = 13
  double goal_distance = 0;
  double goal_direction = 1e6;
  std::cout << "ANGLE_SAMPLE_NUM: " << ANGLE_SAMPLE_NUM << std::endl;

  if (ANGLE_SAMPLE_NUM > 0)
  {
    geometry_msgs::PoseArray local_goal_array;  // local_goal_array = Pose[] (Pose is position and Quaternion)
    local_goal_array.header = goal.header;      // goal = local_goal
    for (int i = 0; i < ANGLE_SAMPLE_NUM; i++)
    {                                                                                  // Checking the distance to obstacles around the target node, Repeat until i=12
      double angle = -local_goal_angle_ + i * d_local_goal_angle_ + target_orientation;  // -60 + i*10 + target_node
      std::cout << "angle: " << angle << std::endl;
      double distance = 0;
      for (; distance <= target_distance; distance += local_map_.info.resolution)
      {  // Distance is defined as the distance to 1grid before the obstacle
        int x_grid = round((distance * cos(angle) - local_map_.info.origin.position.x) / local_map_.info.resolution);
        int y_grid = round((distance * sin(angle) - local_map_.info.origin.position.y) / local_map_.info.resolution);
        if (local_map_.data[x_grid + local_map_.info.width * y_grid] != 0)
        {
          distance = std::max(0.0, distance - local_map_.info.resolution);
          break;
        }
      }

      geometry_msgs::Pose p;
      p.position.x = distance * cos(angle);
      p.position.y = distance * sin(angle);
      p.orientation = tf::createQuaternionMsgFromYaw(angle);  // Create quaternion from yaw data
      local_goal_array.poses.push_back(p);                    // Put p from behind the local goal array
      std::cout << "distance: " << distance << std::endl;

      // Updating local goal, Initial goal_distance is 0
      if (goal_distance < distance)
      {  // Make goal_distance longer
        goal_distance = distance;
        goal_direction = angle;
        std::cout << "updated!" << std::endl;
        std::cout << "goal_distance: " << goal_distance << std::endl;
        std::cout << "goal_direction: " << goal_direction << std::endl;
      }
      else if (goal_distance == distance)
      {
        if (fabs(goal_direction - target_orientation) > fabs(angle - target_orientation))
        {  // Make goal_direction smaller
          goal_direction = angle;
          std::cout << "updated!" << std::endl;
          std::cout << "goal_distance: " << goal_distance << std::endl;
          std::cout << "goal_direction: " << goal_direction << std::endl;
        }
      }
    }
    local_goal_array_pub_.publish(local_goal_array);  // local goal array is also published
  }
  else
  {  // ANGLE_SAMPLE_NUM <= 0, This else statement is not much different from the above
    double angle = target_orientation;
    std::cout << "angle: " << angle << std::endl;

    double distance = 0;

    for (; distance <= target_distance; distance += local_map_.info.resolution)
    {
      int x_grid = round((distance * cos(angle) - local_map_.info.origin.position.x) / local_map_.info.resolution);
      int y_grid = round((distance * sin(angle) - local_map_.info.origin.position.y) / local_map_.info.resolution);
      if (local_map_.data[x_grid + local_map_.info.width * y_grid] != 0)
      {
        distance = std::max(0.0, distance - local_map_.info.resolution);
        break;
      }
    }
    std::cout << "distance: " << distance << std::endl;

    if (goal_distance <= distance)
    {
      if (fabs(goal_direction - target_orientation) > fabs(angle - target_orientation))
      {
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

