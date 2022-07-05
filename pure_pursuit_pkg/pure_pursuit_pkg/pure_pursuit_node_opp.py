#!/usr/bin/env python3
import os

import rclpy
from rclpy.node import Node
from scipy import interpolate
import scipy.ndimage
from scipy.spatial.transform import Rotation as R
import numpy as np
import csv
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class PurePursuitOpp(Node):
    """
    Implement Pure Pursuit on simulation
    """
    def __init__(self):
        super().__init__('pure_pursuit_node_opp')

        # User inputs
        traj_csv = "good_real_points.csv" #Name of csv in racelines directory
        self.speed_override = 2.0 #Set to None for there to be no speed override

        # Define paths
        pkg_dir = os.path.join(os.getcwd(), 'src', 'pure_pursuit_pkg', 'pure_pursuit_pkg')
        traj_csv = os.path.join(pkg_dir, 'racelines', traj_csv)

        #### PURE PURSUIT ###
        # Pure pursuit parameters
        self.pp_steer_L = 1.5  # steering look ahead for pure pursuit
        self.kp = 0.25##0.6
        self.kp_rrt = 0.5#
        self.v_max = 10.0#3#5 #8 #6
        self.v_min = 3.5#0.4# #2.5 #1.5  # This value is NOT used for calculations... only keeps the car above a certain value
        self.ay_max = 5.0#0.1#3
        self.ax_max = 6.0#0.1#3
        self.floor_friction_coeff = 1.0#0.2#1.0 #0.8 #0.4
        self.k = 3.0 #3 #5  # Curvature Scaling Factor
        self.offset = 0.0  # Offset to make car brake before turn, and accelerate out of turn units in spline index steps

        self.spline_index_car = 0  # Index of the car on the spline

        # Convert waypoints to spline
        self.pp_waypoints = load_from_csv(traj_csv, clicked=True, scaler=1)
        spline_data, m = interpolate.splprep([self.pp_waypoints[:, 0], self.pp_waypoints[:, 1]], s=0, per=True)
        self.pp_x_spline, self.pp_y_spline = interpolate.splev(np.linspace(0, 1, 1000), spline_data)
        self.pp_spline_points = np.vstack((self.pp_x_spline, self.pp_y_spline, np.zeros((len(self.pp_y_spline)))))
        self.drive_velocity = np.roll(self.calculate_velocity_profile(self.pp_x_spline, self.pp_y_spline), - self.offset)

        ### ROS PUB/SUB ###
        self.pose_subscriber = self.create_subscription(Odometry, 'opp_racecar/odom', self.pose_callback, 1)
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, 'opp_drive', 1)


    def pose_callback(self, pose_msg):
        """
        This is the main pure pursuit callback loop
        All parameters are set in the init function
        """
        # parse information from pose_msg
        current_position = pose_msg.pose.pose.position
        current_quat = pose_msg.pose.pose.orientation

        # get the current spline index of the car and goal point
        self.spline_index_car = self.get_closest_point_to_car(current_position, self.pp_spline_points)

        # Calculate goal point
        global_goal_point = self.find_goal_point(self.pp_steer_L)
        local_goal_point = self.global_2_local(current_quat, current_position, global_goal_point)

        steering_angle = self.calc_steer(local_goal_point, self.kp)
        drive_speed = self.drive_velocity[self.spline_index_car]
        if drive_speed < self.v_min: # Dont drive too slow
            drive_speed = self.v_min
        msg = AckermannDriveStamped()
        if self.speed_override is None:
            msg.drive.speed = float(drive_speed)###CHANGE THIS BACK, IN SIM THE CHANGING VELOCITY WAS CAUSING PROBLEMS
        else:
            msg.drive.speed = self.speed_override
        msg.drive.steering_angle = float(steering_angle)
        self.drive_publisher.publish(msg)


    def calc_steer(self, goal_point_car, kp):
        """
        Returns the steering angle from the local goal point
        """
        y = goal_point_car[1]
        steer_dir = np.sign(y)
        r = self.pp_steer_L ** 2 / (2 * np.abs(y))
        gamma = 1 / r
        steering_angle = (gamma * kp * steer_dir)
        return steering_angle
        
    def calculate_velocity_profile(self, x_spline_data, y_spline_data):
        #CALCULATE DERIVATIVES FOR CURVATURE
        xder1 = np.gradient(x_spline_data,edge_order=2)
        yder1 = np.gradient(y_spline_data,edge_order=2)

        xder2 = np.gradient(xder1)
        yder2 = np.gradient(yder1)

        curvature = np.abs((xder1 * yder2) - (yder1 * xder2)) / np.power(np.power(xder1,2) + np.power(yder1,2),3/2)

        xy_points = np.array((x_spline_data, y_spline_data))
        xy_offset = np.roll(xy_points,1)
        distances = np.linalg.norm(xy_points -xy_offset, axis=0)
        distances[0]=0

        #pass 1
        ux =np.sqrt((self.floor_friction_coeff * 9.81) / np.abs(curvature * self.k))#Curvature should really be curve radius
        ux = np.minimum(ux, self.v_max)

        ayi = ux**2 *(curvature * self.k)
        ayi = np.minimum(ayi,self.ay_max)

        #Forward pass
        v_forward=[]
        for i in range(len(ux)):
            li = distances[i-1]
            axi = self.ax_max * np.sqrt(1 - (ayi[i-1] / self.ay_max))
            v_forward.append(np.sqrt(ux[i-1]**2+(2*axi*li)))
        v_forward = np.array(v_forward)

        #Backward Pass
        v_backward=[]
        ux = np.flip(v_forward)
        ayi = ux**2 *(curvature * self.k)
        ayi = np.minimum(ayi,self.ay_max)
        for i in range(len(distances)):
            li = distances[i-1]
            axi = self.ax_max * np.sqrt(1 - (ayi[i-1] / self.ay_max))
            v_backward.append(np.sqrt(ux[i-1]**2+(2*axi*li)))
            
        v_backward = np.array(v_backward)
        v_backward = np.flip(np.minimum(v_backward, self.v_max))

        final_velocity_profile = v_backward
        final_velocity_profile = self.smooth_and_sharpen_velocity(final_velocity_profile)

        return final_velocity_profile

    def global_2_local(self, current_quat, current_position, goal_point_global):
        # Construct transformation matrix from rotation matrix and position
        H_global2car = np.zeros([4, 4]) #rigid body transformation from  the global frame of referce to the car
        H_global2car[3, 3] = 1
        current_rotation_matrix = R.from_quat(np.array([current_quat.x,current_quat.y,current_quat.z,current_quat.w])).as_matrix()
        H_global2car[0:3, 0:3] = np.array(current_rotation_matrix)
        H_global2car[0:3, 3] = np.array([current_position.x, current_position.y, current_position.z])

        # Calculate point
        goal_point_global = np.append(goal_point_global, 1).reshape(4, 1)
        goal_point_car = np.linalg.inv(H_global2car) @ goal_point_global

        return goal_point_car

    def get_point_distances(self, points1, points2):
        return np.linalg.norm(points1-points2)

    def get_closest_point_to_car(self, current_position, all_points):
        try:
            current_position=np.array([current_position.x, current_position.y, current_position.z])
        except:
            current_position=np.array([current_position[0], current_position[1], current_position[2]])
        current_position=np.transpose(np.multiply(current_position,np.transpose(np.ones((all_points.shape)))))

        dist = np.linalg.norm(current_position - all_points, axis=0)

        point_index = np.argmin(dist)
        return point_index

    def get_path_distances(self,current_pos, points, current_point):

        pose=Point()
        pose.x=current_pos[0]
        pose.y=current_pos[1]
        pose.z=current_pos[2]
        closest_index = self.get_closest_point_to_car(pose, points)

        if current_point - closest_index > 4:
            a_vect=np.zeros((3,current_point - closest_index+1))
            b_vect=np.zeros((3,current_point - closest_index+1))

            a_vect[:,0]= current_pos
            a_vect[:,1:] = points[:,closest_index:current_point]

            b_vect = points[:,closest_index:current_point+1]

            distance=np.sum(np.linalg.norm(a_vect-b_vect, axis=0))
        else:
            distance = self.get_point_distances(current_pos, points[:,current_point])
        return distance

    def find_goal_point(self, L):
        # Returns the global x,y,z position of the goal point
        points_in_front = np.roll(self.pp_spline_points, - self.spline_index_car, axis=1)
        points_dist = np.linalg.norm(np.roll(points_in_front, 1, axis=1) - points_in_front, axis=0)
        points_dist = np.cumsum(points_dist)
        idx = np.argmin(np.abs(points_dist - L))
        goal_point_car = points_in_front[:, idx]
        return goal_point_car

    def smooth_velocity_profile(self, velocities, kernel_size=30):
        #Smooths the velocity profile
        k = np.ones(int(kernel_size))
        k = k / np.sum(k)
        return scipy.ndimage.convolve(velocities, k, mode='wrap')

    def smooth_forward(self, velocities, kernel_size=30):
        #Smooths the velocity profile
        k = np.hstack((np.zeros(int(kernel_size / 2)), np.ones(int(kernel_size / 2))))
        k = k / np.sum(k)
        return scipy.ndimage.convolve(velocities, k, mode='wrap')

    def smooth_backward(self, velocities, kernel_size=30):
        # Smooths the velocity profile
        k = np.hstack((np.ones(int(kernel_size / 2)), np.zeros(int(kernel_size / 2))))
        k = k / np.sum(k)
        return scipy.ndimage.convolve(velocities, k, mode='wrap')

    def derivative(self, velocities, kernel_size=30):
        k = np.hstack((np.ones(int(kernel_size / 2)), -1 * np.ones(int(kernel_size / 2))))
        k = k / np.sum(abs(k))
        return scipy.ndimage.convolve(velocities, k, mode='wrap')

    def smooth_and_sharpen_velocity(self, velocities, kernel_size=30):

        derivative_vel = self.derivative(velocities)
        speed_up = derivative_vel > 0

        vel_smooth = self.smooth_velocity_profile(velocities)
        vel_smooth_forward = self.smooth_forward(vel_smooth)
        vel_smooth_backward = self.smooth_backward(vel_smooth)

        vel_new = velocities.copy()
        vel_new[speed_up] = vel_smooth_backward[speed_up]
        vel_new[~speed_up] = vel_smooth_forward[~speed_up]
        vel_new = self.smooth_velocity_profile(vel_new)

        return vel_new

def load_from_csv(traj_csv, clicked=False, scaler=10):
    # Open csv and read the waypoint data
    if clicked:
        with open(traj_csv, 'r') as f:
            lines = (line for line in f if not line.startswith('#'))
            data = np.loadtxt(lines, delimiter=',')
        points = data[:, 0:2] / scaler

    else:
        with open(traj_csv, 'r') as f:
            lines = (line for line in f if not line.startswith('#'))
            data = np.loadtxt(lines, delimiter=';')
        points = data[:, 1:3] / scaler

    return points

def main(args=None):
    rclpy.init(args=args)
    print("PurePursuitOpp Initialized")
    pure_pursuit_node_opp = PurePursuitOpp()
    rclpy.spin(pure_pursuit_node_opp)

    pure_pursuit_node_opp.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
