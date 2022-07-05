// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// Make sure you have read through the header file as well
#include "rrt/rrt.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <Eigen/Geometry>
#include "std_msgs/msg/bool.hpp"
#include <math.h>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
//#include "std_msgs/msg/float32multiarray.hpp"
//#include "std_msgs/msg/MultiArrayDimension.hpp"
#include "geometry_msgs/msg/pose_array.hpp"


// Destructor of the RRT classFalse
RRT::~RRT() {
    // Do something in here, free up used memory, print message, etc.
    RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "RRT shutting down");
}
// Constructor of the RRT class
RRT::RRT(): rclcpp::Node("rrt_node"), gen((std::random_device())()) {
    //User Input
    bool sim = false; //Changes the subscription topics

    // ROS publishers for rviz
    grid_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/rrt_occugrid_rviz",1);
    rrt_rviz = this->create_publisher<visualization_msgs::msg::Marker>("/rrt_node_connections_rviz",1);
    rrt_path_rviz = this->create_publisher<visualization_msgs::msg::Marker>("/rrt_path_connections_rviz",1);
    goal_pub = this->create_publisher<visualization_msgs::msg::Marker>("/rrt_goal_point_rviz",1);
    node_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/rrt_all_nodes_rviz",1);

    //ROS Publishers to talk to pure pursuit
    spline_points_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("/rrt_spline_points",1); //Returned RRT spline points

    // ROS subscribers
    if(sim == true){
        string pose_topic = "ego_racecar/odom";
        pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(pose_topic, 1, std::bind(&RRT::pose_callback, this, std::placeholders::_1));
    }
    else{
        string pose_topic = "pf/pose/odom";
        pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(pose_topic, 1, std::bind(&RRT::pose_callback, this, std::placeholders::_1));
    }
    
    string scan_topic = "/scan";
    string global_goal_topic = "/global_goal_pure_pursuit";

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic, 1, std::bind(&RRT::scan_callback, this, std::placeholders::_1));
    global_goal_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    global_goal_topic, 1, std::bind(&RRT::global_goal_callback, this, std::placeholders::_1));

    previous_time = rclcpp::Clock().now();
    RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "Created new RRT Object.");

    //Read in spline points
    std::string file_name = "src/f1tenth_icra2022/pure_pursuit_pkg/pure_pursuit_pkg/racelines/temp/spline.csv";
    std::vector<float> row;
    std::string line, number;
    std::fstream file (file_name, ios::in);
	if(file.is_open())
	{
		while(getline(file, line))
		{
			row.clear();
			stringstream str(line);
			while(getline(str, number, ','))
				row.push_back(std::stof(number));
			spline_points.push_back(row);
		}
	}
    else{ 
        std::cout<<"ERROR_ERROR_ERROR"<<std::endl;
        std::cout<<"RRT.CPP Failed to open spline csv"<<std::endl;
    }
    q.x()= 0;
    q.y()= 0;
    q.z()= 0;
    q.w()= 1;
    rotation_mat = q.normalized().toRotationMatrix();
}

/// MAIN CALLBACK FUNCTIONS///
void RRT::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
    // Receive a scan message and update the occupancy grid
    // Args:
    //    scan_msg (*LaserScan): pointer to the incoming scan message
    // Returns:
    //    occu_grid_flat: A new occupancy grid

    //std::cout<<"Scan callback"<<std::endl;
    int x_scan;
    int y_scan;
    std::memset(occu_grid, 0, sizeof occu_grid);
    std::vector<signed char> occu_grid_flat(occu_grid_y_size * occu_grid_x_size);

    //Build the occupancy grid
    for(int i=0; i<scan_msg->ranges.size(); i++){
        if (std::isnan(scan_msg->ranges[i])==false && std::isinf(scan_msg->ranges[i])==false && scan_msg->ranges[i]!=0){

            x_scan = scan_msg->ranges[i] * cos(scan_msg->angle_increment * i + scan_msg->angle_min) / resolution;
            y_scan = scan_msg->ranges[i] * sin(scan_msg->angle_increment * i + scan_msg->angle_min) / resolution;

            //Make the scans show up larger on the occupancy grid
            for(int j=-1 + x_scan;j<1+ x_scan;j++){//8
                for(int k=-1 + y_scan;k<1 + y_scan;k++){
                    if(j+center_x >0 && j+center_x <occu_grid_x_size){
                        if(k+center_y >0 && k+center_y <occu_grid_y_size){
                            occu_grid[(j+center_x)][occu_grid_y_size-(k+center_y)]=100;
                            occu_grid_flat[((k  + center_y)* occu_grid_x_size) + (j + center_x)]=100;
                        }
                    }
                }
            }
        }
    }
    /* Set scans around the car to open - prevents issues with RRT path search
    for(int i=-1;i<=1;i++){
        for(int j=-1;j<=1;j++){
            occu_grid[(j+center_x)][occu_grid_y_size-(i+center_y)]=0;
            occu_grid_flat[((j  + center_y)* occu_grid_x_size) + (i + center_x)]=0;
        }
    }
    */


    auto new_grid= nav_msgs::msg::OccupancyGrid();
    new_grid.info.resolution=resolution;
    new_grid.info.width=occu_grid_x_size;
    new_grid.info.height=occu_grid_y_size;
    std::string frame_id="map";
    new_grid.header.frame_id=frame_id;
    new_grid.header.stamp=rclcpp::Clock().now();
    new_grid.info.origin = current_car_pose.pose.pose;

    Eigen::Vector3d shift_coords(center_x * resolution, center_y* resolution, 0);
    Eigen::Vector3d shift_in_global_coords = rotation_mat * shift_coords;

    new_grid.info.origin.position.x-= shift_in_global_coords[0];
    new_grid.info.origin.position.y-= shift_in_global_coords[1];

    rclcpp::Time current_time = rclcpp::Clock().now();

    new_grid.data= occu_grid_flat;
    grid_pub->publish(new_grid);
}

void RRT::pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg) {
    // The pose callback when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    // Args:
    //    pose_msg (*PoseStamped): pointer to the incoming pose message
    // Returns:
    //    An updated position and rotation of the car, q and rotation_mat
    //    Performs rrt and returns the path

    current_car_pose = *pose_msg;
    q.x()= current_car_pose.pose.pose.orientation.x;
    q.y()= current_car_pose.pose.pose.orientation.y;
    q.z()= current_car_pose.pose.pose.orientation.z;
    q.w()= current_car_pose.pose.pose.orientation.w;
    rotation_mat = q.normalized().toRotationMatrix();

    //Update rrt if using RRT is required
    try{
        rclcpp::Time current_time = rclcpp::Clock().now();
        if((current_time - previous_time).seconds() > update_rate && rrt_use_it == true){
            previous_time = current_time;
            found_path = false;
            final_path_output = perform_rrt();
        }
        else{
            found_path = true;
        }
        //Convert Local coordinates to global coordinates for pure pursuit
        if(found_path == true && message_sent == false){
            message_sent = true;
            geometry_msgs::msg::PoseArray goal_path_points;
            goal_path_points.header.frame_id = "map";
            geometry_msgs::msg::Pose point_value;
            for (int i=0; i<final_path_output.size(); i++){
                Eigen::Vector3d shift_coords(final_path_output[i].x, final_path_output[i].y, 0);
                Eigen::Vector3d global_coords = rotation_mat * shift_coords;
                point_value.position.x = current_car_pose.pose.pose.position.x + float(global_coords[0]);
                point_value.position.y = current_car_pose.pose.pose.position.y + float(global_coords[1]);
                point_value.position.z= final_path_output.size();
                goal_path_points.poses.push_back(point_value);
            }
            spline_points_pub->publish(goal_path_points);
        }
    }
    catch(...){
        std::cout<<"IN Catch"<<std::endl;
    }
}

/// SECONDARY CALLBACK FUNCTION///
void RRT::global_goal_callback(const nav_msgs::msg::Odometry::ConstSharedPtr goal_msg){
    global_goal = *goal_msg; 
}

///////////////// RRT FUNCTIONS /////////////////
std::vector<std::vector<int>> RRT::bresenhams_line_algorithm(int goal_point[2], int origin_point[2]){
    try{
        int x1 = origin_point[0];
        int y1 = origin_point[1];
        int x2 = goal_point[0];
        int y2 = goal_point[1];

        int y_diff = y2 - y1;
        int x_diff = x2 - x1;

        if (abs(y_diff) >= abs(x_diff)){
            x1 = origin_point[1];
            y1 = origin_point[0];
            x2 = goal_point[1];
            y2 = goal_point[0];
        }

        int intermediate;
        if(x1 > x2){
            intermediate = x1;
            x1 = x2;
            x2 = intermediate;

            intermediate = y1;
            y1 = y2;
            y2 = intermediate;
        }

        y_diff = y2 - y1;
        x_diff = x2 - x1;

        int error = int(x_diff / 2);
        float ystep=-1;
        if(y1 < y2){
            ystep=1;
        }

        int y = y1;
        std::vector<std::vector<int>> output;
        for(int x=x1; x < x2+1 ;x++){
            std::vector<int> coords{x,y};
            if (abs(y_diff) > abs(x_diff)){
                coords[0] = y;
                coords[1] = x;
            }
            output.push_back(coords);
            error -= abs(y_diff);
            if(error < 0){
                y+=ystep;
                error+=x_diff;
            }
        }
        if(abs(goal_point[0] - output.back()[0]) > 1 && abs(origin_point[0] - output.back()[0]) > 1){
            std::vector<std::vector<int>> newoutput;
            for(int i=0;i<output.size();i++){
                std::vector<int> newcoords{output[i][1],output[i][0]};
                newoutput.push_back(newcoords);
            }
            return newoutput;
        }
        else{
            return output;
        }    
    }
    catch(...){
        std::cout<<"bresenhams failed"<<std::endl;
    }
    
}

std::vector<RRT_Node> RRT::perform_rrt(){
    try{
        //std::cout<<"-----Starting RRT-----"<<std::endl;
        bool continue_searching = true;
        int loop_count=0;
        int increment = 1;
        int increment_val = 1;
        int max_loops=4;

        std::vector<RRT_Node> output_path;
        while(continue_searching == true){
            float x_goal;
            float y_goal;

            std::vector<RRT_Node> tree;
            std::vector<double> sampled_point;
            std::vector<double> parent_vec;
            std::vector<RRT_Node> path;

            // Add starting pose to the tree
            struct RRT_Node x_0 = {0.0, 0.0};
            x_0.parent = -1; //set the parent of the base node to -1 for detecting path completion in find_path()
            tree.push_back(x_0); //add to tree

            //Declare variables
            std::vector<double> node_rand;
            int node_nearest;
            bool collision;
            bool goal_flag;

            //Take the pure_pursuit goal in the initial loop
            if(loop_count==0){
                x_goal = global_goal.pose.pose.position.x - current_car_pose.pose.pose.position.x;
                y_goal = global_goal.pose.pose.position.y - current_car_pose.pose.pose.position.y;
            }
            else{//If pure_pursuit goal not drivable, start bringing the goal closer to the car along the spline
                float spline_index=-10000;
                for(int i=0;i<spline_points.size();i++){
                    if(abs(global_goal.pose.pose.position.x - spline_points[i][0]) < 0.1 && abs(global_goal.pose.pose.position.y - spline_points[i][1]) < 0.1){
                        spline_index=i;
                        break;
                    }
                }

                if (spline_index ==-10000){
                    //std::cout<<"No path was found, resorting back to original goal"<<std::endl;
                    x_goal = global_goal.pose.pose.position.x - current_car_pose.pose.pose.position.x;
                    y_goal = global_goal.pose.pose.position.y - current_car_pose.pose.pose.position.y;
                }
                else{
                    int index_val = spline_index - increment;
                    //Make sure the chosen spline index is a realistic value
                    if(index_val < 0){
                    index_val = index_val + spline_points.size();
                    }
                    x_goal = spline_points[index_val][0] - current_car_pose.pose.pose.position.x;
                    y_goal = spline_points[index_val][1] - current_car_pose.pose.pose.position.y;
                }
                float distance_to_goal = sqrt(abs(pow(global_goal.pose.pose.position.x - current_car_pose.pose.pose.position.x,2))+abs(pow(global_goal.pose.pose.position.y - current_car_pose.pose.pose.position.y,2)));
                //std::cout<<"distance="<<distance_to_goal<<std::endl;
                //Don't let the closer goal end up behind the car
                if(abs(distance_to_goal) > 0.25){
                    increment+=increment_val;
                }
            }

            Eigen::Vector3d shift_coords(x_goal, y_goal, 0);
            Eigen::Vector3d local_goal_ = rotation_mat.inverse() * shift_coords;
            x_goal = local_goal_[0];
            y_goal = local_goal_[1];
            //std::cout<<"Local goal x: "<<x_goal <<", Local goal y: "<< y_goal <<std::endl;
            std::vector<RRT_Node> final_path;

            for (int i = 0; i < number_of_nodes; i++){
            struct RRT_Node node_new; //New node
            node_rand = sample(); //Create sampled node "node_rand"
            node_nearest = nearest(tree, node_rand); //closest neigbor in tree
            node_new = steer(tree[node_nearest], node_rand); //get new point
            node_new.parent = node_nearest; //Record the parent and add to the tree
            const auto current_node_index = tree.size();

            collision = check_collision(tree[node_nearest], node_new); //Check if the random node collides with the wall
            if(collision == false){
                // RRT* implementation starts
                node_new.cost = cost(tree, node_new);
                const auto near_neighbour_indices = near(tree, node_new);
                std::vector<bool> is_near_neighbor_collided;
                int best_neighbor = node_new.parent;
                for (const int near_node_index: near_neighbour_indices) {
                    if (check_collision(tree[near_node_index], node_new)) {
                        is_near_neighbor_collided.push_back(true);
                        continue;
                    }
                    is_near_neighbor_collided.push_back(false);

                    double cost = tree[near_node_index].cost + line_cost(tree[near_node_index], node_new);
                    // Rewire tree upstream
                    if (cost < node_new.cost) {
                        node_new.cost = cost;
                        node_new.parent = near_node_index;
                        best_neighbor = near_node_index;
                    }
                }
                for (int i = 0; i < near_neighbour_indices.size(); i++) {
                    if (is_near_neighbor_collided[i] || i == best_neighbor) {
                        continue;
                    }
                    if (tree[near_neighbour_indices[i]].cost > node_new.cost + line_cost(node_new, tree[near_neighbour_indices[i]])) {
                        tree[near_neighbour_indices[i]].parent = current_node_index;
                    }
                }
            tree.emplace_back(node_new);

            goal_flag = is_goal(node_new, x_goal, y_goal);
            if (goal_flag == true){
                message_sent = false;
                //std::cout<<"New Node: (X, Y)= "<<node_new.x<<", "<<node_new.y<<std::endl;
                found_path=true;
                continue_searching = false;
                path = find_path(tree, node_new);  // Find the path
                //Publish visualization stuff for RVIZ
                update_rrt_path_lines(tree, path);
                update_rrt_rviz(tree);

                update_goal_point(x_goal, y_goal);
                std::cout<<"Path found"<<std::endl;
                output_path = path;
                return output_path;
                //break;
            }
            }
        }

        loop_count++;
        if(loop_count > max_loops){
            found_path = false;
            std::cout<<"No Path Found"<<std::endl;
            message_sent = false;
            continue_searching= false;

        }
    }
    return output_path;
    }
    catch(...){
        std::cout<<"IN THE CATCH, RRT FUNCTION FAILED"<<std::endl;
    }
}

std::vector<double> RRT::sample() {
    // This method returns a sampled point from the free space
    // You should restrict so that it only samples a small region
    // of interest around the car's current position
    // Args:
    // Returns:
    //     sampled_point (std::vector<double>): the sampled point in free space

    std::vector<double> sampled_point;

    // XY Method
    // double x_rand = (x_dist(gen) - .5) * occu_grid_x_size  * resolution + (occu_grid_x_size * 0.2 * resolution);
    // double y_rand = (y_dist(gen) - .5) * occu_grid_y_size  * resolution;

    // sampled_point.push_back(x_rand);
    // sampled_point.push_back(y_rand);

    // R-Theta Method
    double theta = (rand1(gen) - .5) * 1.57; //scale from -1.57 to 1.57
    double r = rand2(gen)*6; //
    double x_rand = r*cos(theta);
    double y_rand = r*sin(theta);

    sampled_point.push_back(x_rand);
    sampled_point.push_back(y_rand);        
    // std::cout<<"r: "<<r<<std::endl;        
    // std::cout<<"theta: "<<theta<<std::endl;

    return sampled_point;

}

int RRT::nearest(std::vector<RRT_Node> &tree, std::vector<double> &sampled_point) {
    // This method returns the nearest node on the tree to the sampled point
    // Args:
    //     tree (std::vector<RRT_Node>): the current RRT tree
    //     sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //     nearest_node (int): index of nearest node on the tree
    // Written by: Lenny

    int nearest_node = 0;
    float min_dist = 1000.0;

    double x = sampled_point[0];
    double y = sampled_point[1];
    double cur_x;
    double cur_y;
    float cur_dist;

    for(int i = 0; i < tree.size(); i++){
      //Calculate the distance between sampled point and tree node
      cur_x = tree[i].x;
      cur_y = tree[i].y;
      cur_dist = std::sqrt(abs(std::pow(cur_x - x, 2)) + abs(std::pow(cur_y - y, 2)) * 1.0);

      //Check if less than the best distance
      if (cur_dist < min_dist){
        min_dist = cur_dist;
        nearest_node = i;
      }
    }
    /*
    std::cout<<"******************"<<std::endl;
    std::cout<<"Tree size: "<<tree.size()<<std::endl;
    std::cout<<"Nearest node: "<<nearest_node<<std::endl;
    */

    return nearest_node; //return parent point
}

RRT_Node RRT::steer(RRT_Node &nearest_node, std::vector<double> &sampled_point) {
    // The function steer:(x,y)->z returns a point such that z is “closer”
    // to y than x is. The point z returned by the function steer will be
    // such that z minimizes ||z−y|| while at the same time maintaining
    //||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0

    // basically, expand the tree towards the sample point (within a max dist)

    // Args:
    //    nearest_node (RRT_Node): nearest node on the tree to the sampled point
    //    sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //    new_node (RRT_Node): new node created from steering

    RRT_Node new_node;

    double dx = sampled_point[0] - nearest_node.x;
    double dy = sampled_point[1] - nearest_node.y;
    double angle = atan2(dy,dx);

    double dx_new = max_expansion_dist * cos(angle);
    double dy_new = max_expansion_dist * sin(angle);

    new_node.x = dx_new + nearest_node.x;
    new_node.y = dy_new + nearest_node.y;

    return new_node;
}

bool RRT::check_collision(RRT_Node &nearest_node, RRT_Node &new_node) {
    // This method returns a boolean indicating if the path between the
    // nearest node and the new node created from steering is collision free
    // Args:
    //    nearest_node (RRT_Node): nearest node on the tree to the sampled point
    //    new_node (RRT_Node): new node created from steering
    // Returns:
    //    collision (bool): true if in collision, false otherwise
    // TODO: fill in this method

    float x0_f = center_x + (nearest_node.x / resolution) + 0.5; //added 0.5 to round to the proper number
    float y0_f = center_y - (nearest_node.y / resolution) + 0.5;

    float x1_f = center_x + (new_node.x / resolution) + 0.5;
    float y1_f = center_y - (new_node.y / resolution) + 0.5;

    int dx, dy, sx, sy, error, e2;
    int x0 = (int)x0_f;
    int y0 = (int)y0_f;
    int x1 = (int)x1_f;
    int y1 = (int)y1_f;


    dx = abs(x1 - x0);
    if (x0 < x1){
      sx = 1;
    } else{
      sx = -1;
    }
    dy = -abs(y1 - y0);
    if (y0 < y1){
      sy = 1;
    } else{
      sy = -1;
    }
    error = dx + dy;

    bool collision = false;
    while(1){
        if(occu_grid[x0][y0] > 70){
          collision = true;
          break;
        }
        if(x0 == x1 && y0 == y1){
          break;
        }
        e2 = 2 * error;
        if(e2 >= dy){
            if(x0 == x1){
              break;
            }
            error = error + dy;
            x0 = x0 + sx;
        }
        if(e2 <= dx){
            if(y0 == y1){
              break;
            }
            error = error + dx;
            y0 = y0 + sy;
        }
    }
    return collision;
}

bool RRT::is_goal(RRT_Node &node, double goal_x, double goal_y) {
    // This method checks if the latest node added to the tree is close
    // enough (defined by goal_threshold) to the goal so we can terminate
    // the search and find a path
    // Args:
    //   latest_added_node (RRT_Node): latest addition to the tree
    //   goal_x (double): x coordinate of the current goal
    //   goal_y (double): y coordinate of the current goal
    // Returns:
    //   close_enough (bool): true if node close enough to the goal

    bool close_enough = false;
    // TODO: fill in this method
    double node_x = node.x;
    double node_y = node.y;

    if(sqrt(abs(pow(goal_x-node_x,2)) + abs(pow(goal_y-node_y,2))) < goal_threshold){
        close_enough = true;
    }
    return close_enough;
}

 std::vector<RRT_Node> RRT::find_path(std::vector<RRT_Node> &tree, RRT_Node &latest_added_node) {
    // This method traverses the tree from the node that has been determined
    // as goal
    // Args:
    //   latest_added_node (RRT_Node): latest addition to the tree that has been
    //      determined to be close enough to the goal
    // Returns:
    //   path (std::vector<RRT_Node>): the vector that represents the order of
    //      of the nodes traversed as the found path

    std::vector<RRT_Node> found_path;
    auto current_node = latest_added_node;  //initialize with the last point in parent array which corresponds to nearest node in rrt tree
    found_path.push_back(current_node); //store the nearest node

    while(1){ //terminate loop when stop loop is true
        int parent = current_node.parent;
        if(parent == -1){
          break;
        }
        current_node = tree[parent];
        //std::cout<<"X, Y: "<<current_node.x<<", "<<current_node.y<<std::endl;
        found_path.push_back(current_node); //store the nearest node
    }
    return found_path;
}

double RRT::cost(std::vector<RRT_Node> &tree, RRT_Node &newnode) {
    // This method returns the cost associated with a node
    // Args:
    //    tree (std::vector<RRT_Node>): the current tree
    //    node (RRT_Node): the node the cost is calculated for
    // Returns:
    //    cost (double): the cost value associated with the node

    double cost = 0;

    /////////////Debugging stuff////////////////////
    //std::cout<<"test"<<std::endl;
    //std::cout<<"newnode.parent"<<newnode.parent<<std::endl;
    //std::cout<<"tree[newnode.parent].x "<<tree[newnode.parent].x<<std::endl;
    //std::cout<<"tree[newnode.parent].y "<<tree[newnode.parent].y<<std::endl;
    //std::cout<<"tree[newnode.parent].cost "<<tree[newnode.parent].cost<<std::endl;
    ////////////////////////////////////////////////

    // Cost = parent cost + cost of line joining parent
    cost = tree[newnode.parent].cost + line_cost(tree[newnode.parent], newnode);
    //std::cout<<"cost "<<cost<<std::endl;
    return cost;
}

double RRT::line_cost(RRT_Node &n1, RRT_Node &n2) {
    // This method returns the cost of the straight line path between two nodes
    // Args:
    //    n1 (RRT_Node): the RRT_Node at one end of the path
    //    n2 (RRT_Node): the RRT_Node at the other end of the path
    // Returns:
    //    cost (double): the cost value associated with the path

    double line_cost = 0;
    double n1x = n1.x;
    double n1y = n1.y;
    double n2x = n2.x;
    double n2y = n2.y;      
    // TODO: fill in this method
    line_cost = std::sqrt(abs(std::pow(n1x - n2x, 2)) + abs(std::pow(n1y - n2y, 2))); //L2 norm as cost

    return line_cost;
}

std::vector<int> RRT::near(std::vector<RRT_Node> &tree, RRT_Node &node) {
    // This method returns the set of Nodes in the neighborhood of a
    // node.
    // Args:
    //   tree (std::vector<RRT_Node>): the current tree
    //   node (RRT_Node): the node to find the neighborhood for
    // Returns:
    //   neighborhood (std::vector<int>): the index of the nodes in the neighborhood

    std::vector<int> neighborhood;
    // TODO:: fill in this method
    for(int i=0; i<tree.size(); i++){
        const double distance = sqrt(abs(pow(node.x - tree[i].x, 2)) + abs(pow(node.y - tree[i].y, 2))); //L2 norm as cost
        if(distance < search_radius){
            neighborhood.push_back(i);
        }
    }
    //std::cout<<"neighborohood size="<<neighborhood.size()<<std::endl;
    return neighborhood;
}

/// VIZUALIZATION FUNCTIONS ///////////////////////////////////////////////
void RRT::update_rrt_rviz(std::vector<RRT_Node> &tree){
        auto message = visualization_msgs::msg::Marker();

        Eigen::Vector3d shift_coords(tree[0].x, tree[0].y, 0);
        Eigen::Vector3d shift_in_global_coords = rotation_mat.inverse() * shift_coords;
        message.header.frame_id="map";
        message.type= visualization_msgs::msg::Marker::LINE_LIST;
        message.action = visualization_msgs::msg::Marker::ADD;
        message.scale.x= 0.050;
        message.pose.position.x= current_car_pose.pose.pose.position.x;
        message.pose.position.y= current_car_pose.pose.pose.position.y;
        message.pose.position.z=0.0;
        message.color.a=1.0;
        message.color.r=0.0;
        message.color.b=1.0;
        message.color.g=0.0;
        message.pose.orientation.x=0.0;
        message.pose.orientation.y=0.0;
        message.pose.orientation.z=0.0;
        message.pose.orientation.w=1.0;
        message.lifetime.nanosec=int(1e8);

        for(int i=1;i<tree.size();i++){
            message.header.stamp = rclcpp::Clock().now();
            message.id=i;

            Eigen::Vector3d shift_coords_pt1(float(tree[tree[i].parent].x), float(tree[tree[i].parent].y), 0);
            Eigen::Vector3d shift_in_global_coords_pt1 = rotation_mat * shift_coords_pt1;

            auto point1 = geometry_msgs::msg::Point();
            point1.x=shift_in_global_coords_pt1[0];
            point1.y=shift_in_global_coords_pt1[1];
            point1.z=0.0;

            Eigen::Vector3d shift_coords_pt2(float(tree[i].x), float(tree[i].y), 0);
            Eigen::Vector3d shift_in_global_coords_pt2 = rotation_mat * shift_coords_pt2;

            message.points.push_back(point1);

            auto point2=geometry_msgs::msg::Point();
            point2.x=shift_in_global_coords_pt2[0];
            point2.y=shift_in_global_coords_pt2[1];
            point2.z=0.0;

            message.points.push_back(point2);
            rrt_rviz->publish(message);
        }
}

void RRT::update_rrt_path_lines(std::vector<RRT_Node> &tree, std::vector<RRT_Node> &path){
        auto message = visualization_msgs::msg::Marker();

        Eigen::Vector3d shift_coords(path[0].x, path[0].y, 0);
        Eigen::Vector3d shift_in_global_coords = rotation_mat * shift_coords;
        message.header.frame_id="map";
        message.type= visualization_msgs::msg::Marker::LINE_LIST;
        message.action = visualization_msgs::msg::Marker::ADD;
        message.scale.x= 0.050;
        message.pose.position.x= current_car_pose.pose.pose.position.x;
        message.pose.position.y= current_car_pose.pose.pose.position.y;
        message.pose.position.z=0.0;
        message.color.a=1.0;
        message.color.r=0.0;
        message.color.b=0.0;
        message.color.g=1.0;
        message.pose.orientation.x=0.0;
        message.pose.orientation.y=0.0;
        message.pose.orientation.z=0.0;
        message.pose.orientation.w=1.0;
        message.lifetime.nanosec=int(1e8);

        for(int i=1;i<path.size();i++){
            message.header.stamp = rclcpp::Clock().now();
            message.id=i;

            Eigen::Vector3d shift_coords_pt1(float(tree[path[i].parent].x), float(tree[path[i].parent].y), 0);
            Eigen::Vector3d shift_in_global_coords_pt1 = rotation_mat * shift_coords_pt1;

            auto point1 = geometry_msgs::msg::Point();
            point1.x=shift_in_global_coords_pt1[0];
            point1.y=shift_in_global_coords_pt1[1];
            point1.z=0.0;

            Eigen::Vector3d shift_coords_pt2(float(path[i].x), float(path[i].y), 0);
            Eigen::Vector3d shift_in_global_coords_pt2 = rotation_mat * shift_coords_pt2;

            message.points.push_back(point1);

            auto point2=geometry_msgs::msg::Point();
            point2.x=shift_in_global_coords_pt2[0];
            point2.y=shift_in_global_coords_pt2[1];
            point2.z=0.0;

            message.points.push_back(point2);
            rrt_path_rviz->publish(message);
        }
}

void RRT::update_nodes(std::vector<RRT_Node> &tree){
        auto message = visualization_msgs::msg::MarkerArray();
        //std::cout<<"Size of tree: "<<tree.size()<<std::endl;

        for(int i=0; i<tree.size();i++){
            //Find position of marker
            Eigen::Vector3d shift_coords(tree[i].x, tree[i].y, 0);
            Eigen::Vector3d shift_in_global_coords = rotation_mat * shift_coords;

            auto marker = visualization_msgs::msg::Marker();
            marker.header.frame_id="map";
            marker.type= visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.scale.x= 0.1;
            marker.scale.y= 0.1;
            marker.scale.z= 0.1;
            marker.pose.position.x= shift_in_global_coords[0] + current_car_pose.pose.pose.position.x;
            marker.pose.position.y= shift_in_global_coords[1] + current_car_pose.pose.pose.position.y;
            marker.pose.position.z=0.0;
            marker.color.a=1.0;
            marker.color.r=0.0;
            marker.color.b=0.0;
            marker.color.g=1.0;
            marker.pose.orientation.x=0.0;
            marker.pose.orientation.y=0.0;
            marker.pose.orientation.z=0.0;
            marker.pose.orientation.w=1.0;
            marker.lifetime.nanosec=int(1e8);
            marker.header.stamp = rclcpp::Clock().now();
            marker.id=i;

            message.markers.push_back(marker);
        }
        node_pub->publish(message);
}

// call this function directly in pose_callback
void RRT::update_goal_point(float goal_point_x, float goal_point_y){
        auto message = visualization_msgs::msg::Marker();

        Eigen::Vector3d shift_coords(goal_point_x, goal_point_y, 0);
        Eigen::Vector3d shift_in_global_coords = rotation_mat * shift_coords;

        message.header.frame_id="map";
        message.type= visualization_msgs::msg::Marker::SPHERE;
        message.action = visualization_msgs::msg::Marker::ADD;
        message.scale.x= 0.25;
        message.scale.y= 0.25;
        message.scale.z= 0.25;
        message.pose.position.x=  current_car_pose.pose.pose.position.x + shift_in_global_coords[0] ;
        message.pose.position.y=  current_car_pose.pose.pose.position.y + shift_in_global_coords[1] ;
        message.pose.position.z=0.0;
        message.color.a=1.0;
        message.color.r=0.0;
        message.color.b=0.0;
        message.color.g=1.0;
        message.pose.orientation.x=0.0;
        message.pose.orientation.y=0.0;
        message.pose.orientation.z=0.0;
        message.pose.orientation.w=1.0;
        message.lifetime.nanosec=int(1e8);

        message.header.stamp = rclcpp::Clock().now();
        message.id=0;

        goal_pub->publish(message);
}
int RRT::find_spline_index(float x, float y){
/*
Returns the index of the closest point on the spline to (x,y)
*/
    float spline_index=-10000;
    float min_val = 1000;

    for(int i=0;i<spline_points.size();i++){
    float dist = sqrt(pow(abs(x - spline_points[i][0]), 2)  + pow(abs(y - spline_points[i][1]), 2));
        if(dist < min_val){
            spline_index=i;
            min_val = dist;
        }
    }
    return spline_index;
}