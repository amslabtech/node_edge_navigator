#include "node_edge_navigator/path_searcher.h"

namespace node_edge_navigator
{
PathSearcher::PathSearcher(void)
: local_nh_("~")
, tfl_(tf_)
{
    map_sub_ = nh_.subscribe("local_map", 1, &PathSearcher::map_callback, this, ros::TransportHints().reliable().tcpNoDelay());
    goal_sub_ = nh_.subscribe("subgoal", 1, &PathSearcher::goal_callback, this, ros::TransportHints().reliable().tcpNoDelay());
    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("local_goal", 1);
    path_pub_ = local_nh_.advertise<nav_msgs::Path>("path", 1);

    local_nh_.param<double>("goal_range", goal_range_, 3.0);
    local_nh_.param<std::string>("fixed_frame", fixed_frame_, "map");
    subgoal_received_ = false;
}

void PathSearcher::process(void)
{
    ros::spin();
}

void PathSearcher::map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    ROS_INFO("=== map received ===");
    grid_cells_.clear();
    const unsigned int size = msg->data.size();
    grid_cells_.resize(size);
    for(unsigned int i=0;i<size;++i){
        grid_cells_[i].sum_ = -1;
        grid_cells_[i].parent_index_ = -1;
        grid_cells_[i].cost_ = msg->data[i];
        grid_cells_[i].is_wall_ = grid_cells_[i].cost_ != 0;
    }
    resolution_ = msg->info.resolution;
    grid_width_ = msg->info.width;
    grid_width_2_ = grid_width_ / 2;
    header_ = msg->header;
    ROS_INFO_STREAM("resolution: " << resolution_);
    ROS_INFO_STREAM("grid_width: " << grid_width_);

    if(!subgoal_received_){
        ROS_WARN_THROTTLE(3.0, "subgoal is not received");
        return;
    }

    const Eigen::Vector2d start = Eigen::Vector2d::Zero();
    geometry_msgs::TransformStamped fixed_to_map_transform;
    try{
        fixed_to_map_transform =
            tf_.lookupTransform(msg->header.frame_id, subgoal_.header.frame_id, ros::Time(0), ros::Duration(1.0));
    }catch(tf2::TransformException& ex){
        ROS_WARN_STREAM(ex.what());
        return;
    }
    geometry_msgs::PoseStamped subgoal;
    tf2::doTransform(subgoal_, subgoal, fixed_to_map_transform);
    const Eigen::Vector2d goal(subgoal.pose.position.x, subgoal.pose.position.y);
    nav_msgs::Path path;
    ROS_INFO("calculate path");
    const double cost = calculate_path(start, goal, path);
    if(cost >= 1e4){
        ROS_WARN("failed to generate a path");
        return;
    }
    path_pub_.publish(path);

    if(!path.poses.empty()){
        // extract local goal from the path
        geometry_msgs::PoseStamped local_goal = path.poses[0];
        for(const auto& p : path.poses){
            const Eigen::Vector2d v(p.pose.position.x, p.pose.position.y);
            if(v.norm() < goal_range_){
                local_goal = p;
            }else{
                break;
            }
        }
        local_goal.header = path.header;
        local_goal.pose.orientation = subgoal.pose.orientation;
        goal_pub_.publish(local_goal);
    }
}

void PathSearcher::goal_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    subgoal_received_ = false;
    subgoal_ = *msg;
    ROS_INFO_STREAM("goal:\n" << subgoal_);
    geometry_msgs::TransformStamped transform;
    try{
        transform = tf_.lookupTransform(fixed_frame_, subgoal_.header.frame_id, ros::Time(0), ros::Duration(1.0));
    }catch(tf2::TransformException& ex){
        ROS_WARN_STREAM(ex.what());
        return;
    }
    tf2::doTransform(subgoal_, subgoal_, transform);
    subgoal_received_ = true;
}

double PathSearcher::calculate_path(const Eigen::Vector2d& start, const Eigen::Vector2d& goal, nav_msgs::Path& output)
{
    open_list_.clear();
    close_list_.clear();
    const unsigned int grid_size = grid_width_ * grid_width_;
    for(unsigned int i=0;i<grid_size;++i){
        grid_cells_[i].sum_ = -1;
        grid_cells_[i].parent_index_ = -1;
        grid_cells_[i].step_ = 0;
    }

    const unsigned int start_index = get_index_from_xy(start(0), start(1));
    const unsigned int start_i = get_x_index_from_x(start(0));
    const unsigned int start_j = get_y_index_from_y(start(1));
    const unsigned int goal_index = get_index_from_xy(goal(0), goal(1));
    const unsigned int goal_i = get_x_index_from_x(goal(0));
    const unsigned int goal_j = get_y_index_from_y(goal(1));
    ROS_INFO("calculating path");
    ROS_INFO_STREAM("from " << start(0) << ", " << start(1) << ", " << start_index);
    ROS_INFO_STREAM(start_i << ", " << start_j);
    ROS_INFO_STREAM("to " << goal(0) << ", " << goal(1) << ", " << goal_index);
    ROS_INFO_STREAM(goal_i << ", " << goal_j);
    if(!is_valid_point(start_i, start_j)){
        ROS_ERROR_STREAM("given start " << start.transpose() << " is out of the map");
        return std::numeric_limits<double>::max();
    }
    if(!is_valid_point(goal_i, goal_j)){
        ROS_ERROR_STREAM("given goal " << goal.transpose() << " is out of the map");
        return std::numeric_limits<double>::max();
    }
    open_list_.push_back(start_index);
    grid_cells_[open_list_[0]].sum_ = grid_cells_[open_list_[0]].step_ + get_heuristic(start_i - goal_i, start_j - goal_j);

    if(grid_cells_[goal_index].is_wall_){
        ROS_WARN("the goal is in an obstacle");
    }

    int count = 0;

    while(ros::ok()){
        if(open_list_.empty()){
            ROS_INFO("open list is empty!");
            break;
        }
        // const double loop_start_time = ros::Time::now().toSec();

        count++;
        if(count > 10000){
            ROS_ERROR("count > 10000");
            return 10000;
        }
        // choose a node with minimum cost from open list
        unsigned int n_index = open_list_[0];
        int n = grid_cells_[n_index].sum_;
        for(unsigned int i=0;i<open_list_.size();i++){
            if(grid_cells_[open_list_[i]].sum_ < n){
                n_index = open_list_[i];
                n = grid_cells_[n_index].sum_;
            }
        }
        // ROS_INFO_STREAM("openlist:" << open_list_.size());
        // ROS_INFO_STREAM("goal:" << goal_i << ", " << goal_j);
        if(n_index != goal_index){
            close_list_.push_back(n_index);
            open_list_.erase(std::remove(open_list_.begin(), open_list_.end(), n_index), open_list_.end());
        }else{
            ROS_INFO("--- goal ---");
            break;
        }

        const int n_i = get_x_index_from_index(n_index);
        const int n_j = get_y_index_from_index(n_index);;
        // ROS_INFO_STREAM("current:" << n_i << ", " << n_j);
        // ROS_INFO_STREAM("sum_:" << grid_cells_[n_index].sum_);
        for(int _i=n_i-1;_i<=n_i+1;_i++){
            for(int _j=n_j-1;_j<=n_j+1;_j++){
                if(!((_i == n_i) && (_j == n_j))){
                    if(_i>=0 && _i<grid_width_ && _j>=0 && _j<grid_width_){
                        const int _index = _j * grid_width_ + _i;
                        // ROS_INFO_STREAM("_i, _j: " << _i << ", " << _j);
                        const int g_score = grid_cells_[n_index].step_ + 1;
                        const int f_score = grid_cells_[_index].cost_ + grid_cells_[_index].step_ + get_heuristic(goal_i-_i, goal_j-_j);
                        if(!is_contained(open_list_, _index) && !is_contained(close_list_, _index)){
                            // ROS_INFO("=== open ===");
                            grid_cells_[_index].step_ = g_score;
                            grid_cells_[_index].sum_ = f_score;
                            grid_cells_[_index].parent_index_ = n_index;
                            open_list_.push_back(_index);
                        }else if(is_contained(open_list_, _index)){
                            if(grid_cells_[_index].sum_ > f_score){
                                grid_cells_[_index].sum_ = f_score;
                                grid_cells_[_index].step_ = g_score;
                                grid_cells_[_index].parent_index_ = n_index;
                            }
                        }else if(is_contained(close_list_, _index)){
                            continue;
                        }
                    }
                }
            }
        }
    }
    ROS_INFO_STREAM("count = " << count);
    nav_msgs::Path temp_path;
    temp_path.header = header_;
    int path_index = goal_index;
    ROS_INFO_STREAM("goal_index: " << goal_index);
    geometry_msgs::PoseStamped path_pose;
    path_pose.pose.orientation.w = 1;
    path_pose.header = header_;
    double total_cost = 0;
    while(1){
        total_cost += grid_cells_[path_index].sum_;
        path_pose.pose.position.x = get_x_from_index(path_index);
        path_pose.pose.position.y = get_y_from_index(path_index);
        // path_pose.pose.orientation = _goal.pose.orientation;
        path_pose.pose.orientation.w = 1;
        // ROS_INFO_STREAM(path_pose.pose.position.x << ", " << path_pose.pose.position.y << ", " << path_index << ", " << grid_cells_[path_index].cost_);
        // ROS_INFO_STREAM(grid_cells_[path_index].cost_);
        temp_path.poses.push_back(path_pose);
        path_index = grid_cells_[path_index].parent_index_;
        // ROS_INFO_STREAM("next:" << path_index);
        if(path_index < 0){
            std::reverse(temp_path.poses.begin(), temp_path.poses.end());
            output = temp_path;
            ROS_INFO("=== path length ===");
            ROS_INFO_STREAM(output.poses.size());
            ROS_INFO_STREAM("=== path cost ===");
            ROS_INFO_STREAM(total_cost);
            ROS_INFO_STREAM("path generated!");
            return total_cost;
        }
    }
}

int PathSearcher::get_index_from_xy(const double x, const double y)
{
    const int xi = get_x_index_from_x(x);
    const int yi = get_y_index_from_y(y);
    return yi * grid_width_ + xi;
}

int PathSearcher::get_x_index_from_index(const int index)
{
    return index % grid_width_;
}

int PathSearcher::get_y_index_from_index(const int index)
{
    return index / grid_width_;
}

double PathSearcher::get_x_from_index(const int index)
{
    return (get_x_index_from_index(index) - grid_width_2_) * resolution_;
}

double PathSearcher::get_y_from_index(const int index)
{
    return (get_y_index_from_index(index) - grid_width_2_) * resolution_;
}

double PathSearcher::get_x_index_from_x(const double x)
{
    return std::floor(x / resolution_ + 0.5) + grid_width_2_;
}

double PathSearcher::get_y_index_from_y(const double y)
{
    return std::floor(y / resolution_ + 0.5) + grid_width_2_;
}

bool PathSearcher::is_valid_point(int ix, int iy)
{
    if(ix < 0 || iy < 0){
        return false;
    }else if(ix >= grid_width_ || iy >= grid_width_){
        return false;
    }
    return true;
}

bool PathSearcher::is_contained(const std::vector<int>& vec, int value)
{
    return std::find(vec.begin(), vec.end(), value) != vec.end();
}

int PathSearcher::get_heuristic(int diff_x, int diff_y)
{
    return sqrt(diff_x * diff_x + diff_y * diff_y);
}

}// node_edge_navigator

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_searcher");
    node_edge_navigator::PathSearcher ps;
    ps.process();
    return 0;
}
