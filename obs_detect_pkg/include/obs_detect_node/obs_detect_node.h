// RRT assignment
// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf

#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <sstream>
#include <cmath>
#include <vector>
#include <random>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
/// CHECK: include needed ROS msg type headers and libraries

using namespace std;

class OBS_DETECT : public rclcpp::Node{
public:
    OBS_DETECT();
    virtual ~OBS_DETECT();

    ///////////USER INPUT////////////
    //Flags
    bool sim = false;  // Set flag true for simulation, false for real
    bool publish_rviz = true;
    bool publish_thetas = false;

    //Settings
    int collision_loop_threshold = 15; //How many clear path loops need to run before pure pursuit turns off
    //Turning this value up makes follow the gap stay on longer


    //Grid Settings
    bool scan_padding = false; //Makes the track too cluttered
    bool path_line_padding=true; //Makes the path we check wider than one pixel
    const float  resolution = 0.1; //Meters per pixel of the occugrid
    int occu_grid_x_size= 4.0 / resolution; //Meters 
    int occu_grid_y_size= 2.0 / resolution; //Meters
    float collision_time_buffer = 0.5; //s
    int center_y = occu_grid_y_size/2;
    int center_x = 4;
    
    //Pose information
    Eigen::Quaterniond q;
    Eigen::Matrix3d rotation_mat;
    nav_msgs::msg::Odometry current_car_pose;
    std::vector<std::vector<float>> spline_points;   

    //Drive command information 
    float current_car_speed; 

    //OBS_DETECT Stuff
    rclcpp::Time previous_time = rclcpp::Clock().now();
    bool use_coll_avoid=false;
    float collision_l;
    std::vector<float> global_obs_detect_goal{0.0,0.0,0.0};
    int car_spline_idx = 0;
    int goal_spline_idx = 100;
    bool got_pose_flag = false;
    int collision_detect_counter = 0;

    //Gap stuff
    float max_range_threshold = 10.0;
    float bubble_dist_threshold = 6; //meteres
    float disp_threshold = .4;//meter
    float car_width = .60; //Meter
    float angle_cutoff = 1.5; //radians



private:
    //Spline points location
    std::string spline_file_name = "src/f1tenth_icra2022/pure_pursuit_pkg/pure_pursuit_pkg/racelines/temp/spline.csv";
    //std::string spline_file_name = "src/pure_pursuit_pkg/pure_pursuit_pkg/racelines/temp/spline.csv";


    //Publishers 
    std::string coll_grid_topic = "/coll_grid_pub_rviz";
    std::string coll_path_topic = "/coll_path_pub_rviz";
    std::string use_avoid_topic = "/use_obs_avoid";
    std::string gap_theta_topic = "/gap_thetas";
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr path_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr use_avoid_pub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr gap_theta_pub;

    //Subscribers
    std::string scan_topic = "/scan";
    std::string pose_topic_sim = "ego_racecar/odom";
    std::string pose_topic_real = "pf/pose/odom";
    std::string drive_topic = "/drive";
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub_;

    
    // callbacks
    void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg);
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
    void drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr drive_msg);

    //functions
    void check_to_activate_obs_avoid(std::vector<signed char> &obstacle_data);
    std::vector<std::vector<int>> bresenhams_line_algorithm(int goal_point[2], int origin_point[2]);
    int find_spline_index(float x, float y);
    int find_obs_detect_goal_idx(float l_dist, std::vector<std::vector<float>> spline_points, int car_idx);
    std::vector<std::vector<int>> draw_connecting_line(int origin_point[2], int goal_point[2]);
    int* spline_point2occu_coordinate(int spline_idx, int* occu_point);

    //Publisher functions
    void publish_grid(std::vector<signed char> &occugrid_flat);
    void publish_path(std::vector<signed char> &occugrid_flat); 

    //Gap identifier functions
    void preprocess_lidar(std::vector<float>& ranges, int num_readings);
    int* find_max_gap(std::vector<float>& ranges, int num_readings);
    int find_disparities(std::vector<int>& disp_idx, std::vector<float>& ranges, int num_readings);
    void set_disparity(std::vector<float>& ranges, int num_points, std::vector<int>& disp_idx, int num_disp, float angle_increment, std::vector<float>& ranges_clean);
    void set_close_bubble(std::vector<float>& ranges, std::vector<float>& angles, int num_points, float angle_increment);
    void find_and_publish_gap(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);


};


