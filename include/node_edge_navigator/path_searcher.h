#ifndef __PATH_SEARCHER
#define __PATH_SEARCHER

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>

namespace node_edge_navigator
{

class PathSearcher
{
public:
    PathSearcher(void);
    void process(void);
    void map_callback(const nav_msgs::OccupancyGridConstPtr& msg);
    void goal_callback(const geometry_msgs::PoseStampedConstPtr& msg);
    double calculate_path(const Eigen::Vector2d& start, const Eigen::Vector2d& goal, nav_msgs::Path& output);
    int get_index_from_xy(const double x, const double y);
    int get_x_index_from_index(const int index);
    int get_y_index_from_index(const int index);
    double get_x_from_index(const int index);
    double get_y_from_index(const int index);
    bool is_valid_point(int ix, int iy);
    bool is_contained(const std::vector<int>& vec, int value);
    int get_heuristic(int diff_x, int diff_y);

    class GridCell
    {
    public:
        GridCell(void)
        {
            cost_ = 0;
            step_ = 0;
            sum_ = -1;
            parent_index_ = -1;
            is_wall_ = false;
        }
        int cost_;
        int step_;
        int sum_;
        int parent_index_;
        bool is_wall_;
    };

protected:
    ros::NodeHandle nh_;
    ros::NodeHandle local_nh_;
    ros::Subscriber map_sub_;
    ros::Subscriber goal_sub_;
    ros::Publisher path_pub_;
    ros::Publisher goal_pub_;
    tf2_ros::Buffer tf_;
    tf2_ros::TransformListener tfl_;

    double goal_range_;
    std::string fixed_frame_;

    double resolution_;
    int grid_width_;
    int grid_width_2_;
    std_msgs::Header header_;
    std::vector<GridCell> grid_cells_;
    std::vector<int> open_list_;
    std::vector<int> close_list_;
    geometry_msgs::PoseStamped subgoal_;
    bool subgoal_received_;
};

}// namespace node_edge_navigator

#endif// __PATH_SEARCHER
