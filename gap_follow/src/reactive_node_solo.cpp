#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int16.hpp"
#include <fstream>
#include <iostream>


/// CHECK: include needed ROS msg type headers and libraries


using std::placeholders::_1;


class ReactiveFollowGap : public rclcpp::Node {
// Implement Reactive Follow Gap on the car
// This is just a template, you are free to implement your own node!

    public:
        ReactiveFollowGap() : Node("reactive_node_solo")
        {
            /// Create ROS subscribers and publishers
            drive_publisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 1);
            scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(lidarscan_topic, 1, std::bind(&ReactiveFollowGap::lidar_callback, this, _1));
        }

    private:
        // variables
        int window_size = 3; //This is the size of the "window" for the window mean
        float max_range_threshold = 10.0; //Anything beyond this value is set to this value
        float max_drive_range_threshold = 9.0;

        float car_width_bubble = 1.2; //Changes the size of the bubble set close
        float car_width_disp = 0.5; //Changes the size of disparity extension
        float angle_cutoff = 2.1;//1.5; //radians
        float disp_threshold = .4;//meter
        float bubble_dist_threshold = 6; //meteres

        float vel_max = 3.0; // m/s
        float vel_min = 2.0; // m/s
        float max_drive_angle = .52; //Rads
        

        std::string lidarscan_topic = "/scan";
        std::string drive_topic = "/drive";


        /// Create ROS subscribers and publishers
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher;

        void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
        {
            
            std::cout<<"USING GAP FOLLOW"<<std::endl;
            //Load in range information from message
            int num_readings = scan_msg->ranges.size();
            float ranges_raw[1081];
            copy(std::begin(scan_msg->ranges), std::end(scan_msg->ranges), std::begin(ranges_raw));

            //Create array of angles for the raw ranges
            std::vector<float> angles_raw;
            float cur_angle;
            float angle_increment = scan_msg->angle_increment;

            for(int i = 0; i < num_readings; i++){
                cur_angle = (scan_msg->angle_increment * i) + (scan_msg->angle_min);
                angles_raw.push_back(cur_angle);
            }

            //Find the start and end of the angle cutoff
            //Reduces the number of ranges looked at
            int cutoff_start_idx;
            int cutoff_end_idx;
            int num_readings_p;
            for(int i = 0; i < num_readings; i++){
                if (angles_raw[i] > (angle_cutoff * -1.0)){
                    cutoff_start_idx = i;
                    break;
                }
            }
            for(int i = 0; i < num_readings; i++){
                if (angles_raw[i] > (angle_cutoff)){
                    cutoff_end_idx = i;
                    break;
                }
            }
            num_readings_p = cutoff_end_idx - cutoff_start_idx;

            //Create new "processed" angle and ranges vectors as a subset of the raw ranges and angles
            std::vector<float> ranges_p(&ranges_raw[cutoff_start_idx], &ranges_raw[cutoff_end_idx]);
            std::vector<float> angles_p(&angles_raw[cutoff_start_idx], &angles_raw[cutoff_end_idx]);

            preprocess_lidar(ranges_p, num_readings_p); //updates ranges_p

            int num_disp;
            std::vector<int> disp_idx;
            num_disp = find_disparities(disp_idx, ranges_p, num_readings_p);
            std::vector<float> ranges_p_clean = ranges_p;
            set_disparity(ranges_p, num_readings_p, disp_idx, num_disp, angle_increment, ranges_p_clean);  // Set all values at the disparity to the value of the closest point
            set_close_bubble(ranges_p, angles_p, num_readings_p, angle_increment);
            int *gap_idxes = find_max_gap(ranges_p, num_readings_p); //find the drive idx from the max gap

            float drive_angle;
            int drive_idx {0};
            drive_angle = find_drive_angle(ranges_p, angles_p, gap_idxes, drive_idx);

            bool going_to_hit=false;
            going_to_hit = corner_safety_check(ranges_raw, angles_raw, drive_angle, num_readings, angle_increment);

            // Publish Drive message
            if (going_to_hit==true){
                drive_angle = 0.0;
            }
            if (drive_angle > max_drive_angle){
                drive_angle = max_drive_angle;
            }
            if (drive_angle < -1 * max_drive_angle){
                drive_angle = -1 * max_drive_angle;
            }

            auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
            drive_msg.drive.steering_angle = drive_angle;
            std::cout<<"Drive angle :"<<drive_angle<<std::endl;
            std::cout<<"Drive idx : "<<drive_idx<<std::endl;
            drive_msg.drive.speed = 0.0; drive_speed_calc(drive_angle, vel_max, vel_min); //Scales the velocity from the pure pursuit velocity to some lower bound, depending on the distance of range readings... maybe steer angle would be better? 
            drive_publisher->publish(drive_msg);
            
        }

        void preprocess_lidar(std::vector<float>& ranges, int num_readings)
        {
            std::vector<float> old_ranges;
            old_ranges = ranges;

            // 1.Setting each value to the mean over some window
            float running_mean = 0.0;
            ranges[0] = old_ranges[0];
            for(int i = 1; i < (num_readings - 1); i++){
                running_mean = 0.0;
                for(int j = -1; j < 2; j++){
                    running_mean += old_ranges[i + j];
                }
                ranges[i] = running_mean/3.0;
                if (ranges[i] < 0){
                    ranges[i] = 0;
                }
            }
            ranges[num_readings -1] = old_ranges[num_readings - 1];

            //2.Rejecting high values
            for(int i = 0; i < num_readings; i++) {
                if (ranges[i] > max_range_threshold) {
                    ranges[i] = max_range_threshold;
                }
            }

            return;
        }

        int find_disparities(std::vector<int>& disp_idx, std::vector<float>& ranges, int num_readings){
            int disp_count = 0;
            for(int i = 1; i < num_readings; i++){
                if (abs(ranges[i] - ranges[i-1]) > disp_threshold){
                    //RCLCPP_INFO(this->get_logger(), "Disparity IDX [%d], Range 1: %f  Range 2:%f", i, ranges[i], ranges[i-1]);
                    //there is a disparity
                    if (ranges[i] < ranges[i -1]){
                      disp_idx.push_back(i);
                    } else{
                      disp_idx.push_back(i-1);
                    }
                    disp_count += 1;
                }
            }
            return disp_count;
        }

        int* find_max_gap(std::vector<float>& ranges, int num_readings)
        {
            // Return the start index & end index of the max gap
            int current_gap_width = 0; // width of the current gap being tested
            int max_gap_width = 0; // largest gap found so far
            int gap_flag = 0; // 0 for no gap, 1 for gap
            int current_start_idx = 0; // start index for current gap being tested

            int start_idx = 0; // overall largest gap start
            int end_idx = 0; // overall largest gap end

            static int gap_idxes[2];

            for(int i = 0; i < num_readings; i++){
                if (ranges[i] > 0.0){
                    if(gap_flag == 0){
                        //New gap
                        current_start_idx = i;
                    }
                    gap_flag = 1;
                    current_gap_width +=1;
                } else {
                    if(gap_flag == 1){
                        //Gap ended
                        if(current_gap_width > max_gap_width){
                            //New largest gap
                            start_idx = current_start_idx;
                            end_idx = i -1;
                            max_gap_width = current_gap_width;
                        }
                    }
                    gap_flag = 0;
                    current_gap_width = 0;
                }
            }
            if(current_gap_width > max_gap_width){
                start_idx = current_start_idx;
                end_idx = num_readings - 1;
            }

            gap_idxes[0] = start_idx;
            gap_idxes[1] = end_idx;
            return gap_idxes;
        }

        float find_drive_angle(std::vector<float>& ranges, std::vector<float>& angles, int*gap_idx, int &drive_idx){
            //Find the largest group of max range readings and drive to the center
            //Attempts to fix the side to side motion of the car
            //Find the deepest range reading
            float deepest_val = 0.0;
            int deepest_idx;
            float drive_angle;

            for(int i = gap_idx[0]; i<gap_idx[1]; i++){
                if(ranges[i] > deepest_val){
                    deepest_val = ranges[i];
                    deepest_idx = i;
                }
            }

            if (deepest_val > (max_drive_range_threshold - 0.1)){
                //Find the max gap of the max readings
                int current_gap_width = 0; // width of the current gap being tested
                int max_gap_width = 0; // largest gap found so far
                int gap_flag = 0; // 0 for no gap, 1 for gap
                int current_start_idx = 0; // start index for current gap being tested

                int start_idx = 0; // overall largest gap start
                int end_idx = 0; // overall largest gap end

                for(int i = gap_idx[0]; i < gap_idx[1]; i++){
                    if (ranges[i] > (max_drive_range_threshold - 0.1)){
                        if(gap_flag == 0){
                            //New gap
                            current_start_idx = i;
                        }
                        gap_flag = 1;
                        current_gap_width +=1;
                    } else {
                        if(gap_flag == 1){
                            //Gap ended
                            if(current_gap_width > max_gap_width){
                                //New largest gap
                                start_idx = current_start_idx;
                                end_idx = i - 1;
                                max_gap_width = current_gap_width;
                            }
                        }
                        gap_flag = 0;
                        current_gap_width = 0;
                    }
                }
                if(current_gap_width > max_gap_width){
                    start_idx = current_start_idx;
                    end_idx = gap_idx[1] - 1;
                }
                drive_idx = ((end_idx - start_idx)/2) + start_idx;
                drive_angle = angles[drive_idx];
            } else {
                //Take the deepest point in the vector
                drive_angle = angles[deepest_idx];
                drive_idx = deepest_idx;
            }
            return drive_angle;
        }

        void set_disparity(std::vector<float>& ranges, int num_points, std::vector<int>& disp_idx, int num_disp, float angle_increment, std::vector<float>& ranges_clean){
            float theta;
            int n;
            float n_float;
            int bubble_idx;
            float bubble_dist;

            for (int i=0; i<num_disp; i++){
                bubble_idx = disp_idx[i];

                bubble_dist = ranges_clean[bubble_idx];

                if(bubble_dist > bubble_dist_threshold){
                  continue;
                }

                theta = atan2((car_width_disp /2.0), bubble_dist);
                n_float = theta/angle_increment; //Is 270 radians!!!!
                n = static_cast<int>(n_float);
                //RCLCPP_INFO(this->get_logger(), "Bubble idx [%d], N value [%d], Theta [%f],  Bubble range [%f]", bubble_idx, n, theta, ranges_clean[bubble_idx]);


                //Cases to fix out of bounds errors
                if (bubble_idx + n > num_points){
                    n = num_points - bubble_idx;
                }
                if (bubble_idx - n < 0.0 ){
                    n = bubble_idx;
                }

                //Set points within bubble to zero
                for (int j = 0; j < n + 1; j++){
                    if (bubble_dist < ranges[bubble_idx + j]){
                        ranges[bubble_idx + j] = bubble_dist;
                    }

                }
                for (int j = 0; j < n + 1; j++){
                    if (bubble_dist < ranges[bubble_idx - j]){
                        ranges[bubble_idx - j] = bubble_dist;
                    }
                }

                /*Debugging
                for (int i = 0; i < 5; i++){
                    RCLCPP_INFO(this->get_logger(), "IDX[%d], Range [%f]", bubble_idx + i, ranges[bubble_idx + i]);
                }
                */
            }
        }

        void set_close_bubble(std::vector<float>& ranges, std::vector<float>& angles, int num_points, float angle_increment){
            int close_idx;
            float low_val = 100.0;

            //Find the closest point
            for (int i = 0; i < num_points; i++){
                if (ranges[i] < low_val){
                    low_val = ranges[i];
                    close_idx = i;
                }
            }
            float theta;
            int n;
            float n_float;
            int bubble_idx;

            bubble_idx = close_idx;

            //Use trig to find number of points to eliminate
            theta = atan2((car_width_bubble /2.0), ranges[bubble_idx]);
            n_float = theta/angle_increment; //Is 270 radians!!!!
            n = static_cast<int>(n_float);
            RCLCPP_INFO(this->get_logger(), "Bubble CLOSE- idx [%d], angle[%f], N value [%f], range [%f]", bubble_idx, angles[bubble_idx], n_float, ranges[bubble_idx]);


            //Cases to fix out of bounds errors
            //Cases to fix out of bounds errors
            int n_up = n;
            int n_down = n;
            if (bubble_idx + n > num_points){
                n_up = num_points - bubble_idx;
            }
            if (bubble_idx - n < 0.0 ){
                n_down = bubble_idx;
            }

            //Set points within bubble to zero
            for (int j = 0; j < n_up + 1; j++){
                ranges[bubble_idx + j] = 0.0;
            }
            for (int j = 0; j < n_down + 1; j++){
                ranges[bubble_idx - j] = 0.0;
            }
        }

        float drive_speed_calc(float steer_angle, float max_drive_speed, float min_drive_speed){
            float drive_speed = max_drive_speed - ((abs(steer_angle)/max_drive_angle) * (max_drive_speed - min_drive_speed));          

            return drive_speed;
        }

        bool corner_safety_check(float p_ranges[1080], std::vector<float> angles,  float drive_angle, int num_readings, float angle_increment){
            bool crashing = false;

            float safety_range_threshold = .5;
            float drive_activation_angle = .20632; //Set to 25 deg right now
            float window_size = 0.174533; //10 degrees
            float window_start = 1.5708;

            float num_window_readings = window_size / angle_increment;

            float window_mean = 0.0;


            if (abs(drive_angle) > drive_activation_angle && drive_angle < 0){
                //right turn
                //get average of right side
                for(int i = 0; i < num_readings; i++){
                    if (angles[i] < (window_start * -1) && angles[i] > (window_start * -1 - window_size)){
                        window_mean += p_ranges[i];
                    }
                }
                window_mean = window_mean / num_window_readings;

                if (window_mean < safety_range_threshold){
                    crashing = true;
                }
            }

            if (abs(drive_angle) > drive_activation_angle && drive_angle > 0){
                //left turn
                //get average of left side
                for(int i = 0; i < num_readings; i++){
                    if (angles[i] > window_start  && angles[i] < window_start + window_size){
                        window_mean += p_ranges[i];
                    }
                }
                window_mean = window_mean / num_window_readings;
                if (window_mean < safety_range_threshold){
                    crashing = true;
                }
            }
            return crashing;
        }

};

int main(int argc, char ** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}
