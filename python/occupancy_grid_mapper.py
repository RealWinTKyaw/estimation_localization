#!/usr/bin/env python3

import rclpy
import tf2_ros
from tf2_ros import TransformBroadcaster as tr
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from std_msgs.msg import Header, ColorRGBA
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from threading import Lock
from math import sqrt, cos, sin, pi, atan2, log
from rclpy.qos import QoSProfile
import random
import numpy as np

class OccupancyGridMap:
    def __init__(self, num_rows, num_cols, meters_per_cell, grid_origin_in_map_frame, init_log_odds):
        self.num_rows = num_rows
        self.num_cols = num_cols
        self.meters_per_cell = meters_per_cell
        self.log_odds_ratio_occupancy_grid_map = init_log_odds * np.ones((num_rows, num_cols), dtype='float64')
        self.seq = 0

        self.map_info = MapMetaData()
        self.map_info.resolution = meters_per_cell
        self.map_info.width = num_rows
        self.map_info.height = num_cols
    
        self.map_info.origin.position.x = float(grid_origin_in_map_frame[0])
        self.map_info.origin.position.y = float(grid_origin_in_map_frame[1])
        self.map_info.origin.position.z = float(grid_origin_in_map_frame[2])

        self.map_info.origin.orientation.x = 0.0
        self.map_info.origin.orientation.y = 0.0
        self.map_info.origin.orientation.z = 0.0
        self.map_info.origin.orientation.w = 1.0
                
    def update_log_odds_ratio_in_grid_coords(self, row, col, delta_log_odds):
        assert (row >=0 and row < self.num_rows)
        assert (col >=0 and col < self.num_cols)
        self.log_odds_ratio_occupancy_grid_map[row][col] += delta_log_odds 
        
    def cartesian_to_grid_coords(self, x, y):
        gx = (x - self.map_info.origin.position.x) / self.map_info.resolution
        gy = (y - self.map_info.origin.position.y) / self.map_info.resolution
        row = min(max(int(gy), 0), self.num_rows)
        col = min(max(int(gx), 0), self.num_cols)
        return (row, col)

    def log_odds_ratio_to_belief(self, lor):
        return 1.0 - 1.0/(1 + np.exp(lor))

    def get_map_as_ros_msg(self, timestamp, frame_id):
        self.seq += 1
        msg = OccupancyGrid()
        msg.header.seq = self.seq
        msg.header.stamp = timestamp
        msg.header.frame_id = frame_id
        msg.info = self.map_info
        
        occupancy_belief = 100 * self.log_odds_ratio_to_belief(self.log_odds_ratio_occupancy_grid_map)

        assert (occupancy_belief >= 0).all()
        assert (occupancy_belief <= 100).all()
        
        msg.data = occupancy_belief.astype(dtype='int8', copy=True).reshape((self.num_rows*self.num_cols, ))
        return msg
        

class HuskyMapper:
    def __init__(self, num_rows, num_cols, meters_per_cell):
        rclpy.init(args=None)
        self.node = rclpy.create_node('occupancy_grid_mapper')
        self.tf_listener = tf2_ros.BufferClient(self.node,'/your_namespace')

        self.odometry_position_noise_std_dev = self.node.declare_parameter("odometry_position_noise_std_dev").value
        self.odometry_orientation_noise_std_dev = self.node.declare_parameter("odometry_orientation_noise_std_dev").value
        og_origin_in_map_frame = np.array([-20, -10, 0])
        self.init_log_odds_ratio = 0  # log(0.5/0.5)
        self.ogm = OccupancyGridMap(num_rows, num_cols, meters_per_cell, og_origin_in_map_frame, self.init_log_odds_ratio)

        self.max_laser_range = None
        self.min_laser_range = None
        self.max_laser_angle = None
        self.min_laser_angle = None

        self.odometry = None

        self.q_map_baselink = None
        self.R_map_baselink = None
        self.p_map_baselink = None

        self.q_map_baselaser = None
        self.R_map_baselaser = None
        self.p_map_baselaser = None

        self.q_baselink_baselaser = np.array([1.0, 0, 0, 0])
        self.p_baselink_baselaser = np.array([0.337, 0.0, 0.308])
    
        self.mutex = Lock()

        self.occupancy_grid_pub = self.node.create_publisher(OccupancyGrid, '/husky/occupancy_grid', 10)
        self.laser_points_marker_pub = self.node.create_publisher(Marker, '/husky/debug/laser_points', 10)
        self.robot_pose_pub = self.node.create_publisher(PoseStamped, '/husky/debug/robot_pose', 10)

        self.laser_sub = self.node.create_subscription(LaserScan, '/husky/scan', self.laser_scan_callback, 10)
        self.odometry_sub = self.node.create_subscription(Odometry, '/husky/odometry/ground_truth', self.odometry_callback, 10)
        
    def odometry_callback(self, msg):
        self.mutex.acquire()
        self.odometry = msg

        # Adds noise to the odometry position measurement according to the standard deviations specified as parameters in the launch file
        # We should have used noisy measurements from the Gazebo simulator, but it is more complicated to configure
        # so, we add random noise here ourselves, assuming perfect odometry from the simulator. 
        self.odometry.pose.pose.position.x += random.gauss(0, self.odometry_position_noise_std_dev)
        self.odometry.pose.pose.position.y += random.gauss(0, self.odometry_position_noise_std_dev)
        
        #
        # TODO: populate the quaternion from the husky_1/base_link frame to the map frame 
        #       based on the current odometry message. In order to know more about where
        #       these frames are located on the robot, run: rosrun rviz rviz and look at the TF
        #       widget. Pay attention to the following frames: husky_1/base_link which is at the center 
        #       of the robot's body, husky_1/base_laser, which is the frame of the laser sensor,
        #       and map, which is where odometry messages are expressed in. In fact, odometry 
        #       messages from the Husky are transformations from husky_1/base_link to map 
        # 
        #self.q_map_baselink = np.array([x, y, z, w])                            msg.pose.pose.orientation.w])
        
        # Corrupting the quaternion with noise in yaw, because we have configured the simulator
        # to return noiseless orientation measurements.
        yaw_noise = random.gauss(0, self.odometry_orientation_noise_std_dev) * pi/180.0
        q_truebaselink_noisybaselink = np.array([0, 0, np.sin(yaw_noise), np.cos(yaw_noise)])
        self.q_map_baselink = tr.quaternion_multiply(self.q_map_baselink, q_truebaselink_noisybaselink)
        
        
        # Computes the rotation matrix from husky_1/base_link to map
        self.R_map_baselink = tr.quaternion_matrix(self.q_map_baselink)[0:3, 0:3]

        #
        # TODO: populate the position of the husky_1/base_link frame in map frame 
        #       coordinates based on the current odometry message
        #
        self.p_map_baselink = np.array([msg.pose.pose.position.x,
                                       msg.pose.pose.position.y,
                                       msg.pose.pose.position.z])

        #
        # TODO: populate the quaternion from the frame husky_1/base_laser to the map frame 
        #       note: you have access to the static quaternion from husky_1/base_laser to 
        #       husky_1/base_link
        self.q_map_baselaser = tr.quaternion_multiply(self.q_baselink_baselaser, self.q_map_baselink)

        #
        # TODO: populate the rotation matrix from the frame husky_1/base_laser to the map frame 
        #       note: you have access to the static rotation matrix from husky_1/base_laser to 
        #       husky_1/base_link
        #       also note: np.dot(A,B) multiplies numpy matrices A and B, whereas A*B is element-wise 
        #       multiplication, which is not usually what you want
        #
        self.R_map_baselaser = np.dot(tr.quaternion_matrix(self.q_map_baselaser)[0:3, 0:3],
                                     tr.quaternion_matrix(self.q_baselink_baselaser)[0:3, 0:3])
        #
        # TODO: populate the origin of the frame husky_1/base_laser in coordinates of the map frame 
        #       note: you have access to the static rotation matrix from husky_1/base_laser to 
        #       husky_1/base_link and also to the origin of the husky_1/base_laser frame in coordinates of
        #       frame husky_1/base_link
        self.p_map_baselaser = np.dot(tr.translation_matrix(self.p_baselink_baselaser),
                                      tr.translation_matrix(self.p_map_baselink))[0:3, 3]
        self.mutex.release()

        
    def from_laser_to_map_coordinates(self, points_in_baselaser_frame):
        #
        # The robot's odometry is with respect to the map frame, but the points measured from
        # the laser are given with respec to the frame husky_1/base_laser. This function convert
        # the measured points in the laser scan from husky_1/base_laser to the map frame. 
        #
        points_in_map_frame = [np.dot(self.R_map_baselaser, xyz_baselaser) + self.p_map_baselaser for xyz_baselaser in points_in_baselaser_frame]
        return points_in_map_frame
        

    def is_in_field_of_view(self, robot_row, robot_col, robot_theta, row, col):
        # Returns true iff the cell (row, col) in the grid is in the field of view of the 2D laser of the
        # robot located at cell (robot_row, robot_col) and having yaw robot_theta in the map frame.  
        # Useful things to know:
        # 1) self.ogm.meters_per_cell converts cell distances to metric distances 
        # 2) atan2(y, x) gives the angle of the vector (x, y)
        # 3) atan2(sin(theta_1 - theta_2), cos(theta_1 - theta_2)) gives the angle difference between theta_1 and theta_2 in [-pi, pi]
        # 4) self.max_laser_range and self.max_laser_angle specify some of the limits of the laser sensor
        #
        # TODO: fill this
        #
        return False


    def inverse_measurement_model(self, row, col, robot_row, robot_col, robot_theta_in_map, beam_ranges, beam_angles):
        alpha = 0.1
        beta = 10 * pi/180.0
        p_occupied = 0.999
        
        #
        # TODO: Find the range r and angle diff_angle of the beam (robot_row, robot_col) ------> (row, col)  
        # r should be in meters and diff_angle should be in [-pi, pi]. Useful things to know are same as above.
        #
        diff_row = row - robot_row
        diff_col = col - robot_col
        r = sqrt(diff_row**2 + diff_col**2) * self.ogm.meters_per_cell
        diff_angle = atan2(sin(robot_theta_in_map - atan2(diff_row, diff_col)), cos(robot_theta_in_map - atan2(diff_row, diff_col)))

        closest_beam_angle, closest_beam_idx = min((val, idx) for (idx, val) in enumerate([abs(diff_angle - ba) for ba in beam_angles]))
        r_cb = beam_ranges[closest_beam_idx]
        theta_cb = beam_angles[closest_beam_idx]

        if r > min(self.max_laser_range, r_cb + alpha/2.0) or abs(diff_angle - theta_cb) > beta/2.0:
            return self.init_log_odds_ratio
        
        if r_cb < self.max_laser_range and abs(r - r_cb) < alpha/2.0:
            return log(p_occupied/(1-p_occupied))
        
        if r <= r_cb:
            return log((1-p_occupied)/p_occupied)

        return 0.0

        
    def laser_scan_callback(self, msg):
        self.mutex.acquire()
        
        self.min_laser_angle = msg.angle_min
        self.max_laser_angle = msg.angle_max
        self.min_laser_range = msg.range_min
        self.max_laser_range = msg.range_max
        
        if self.odometry is None:
            # ignore the laser message if no odometry has been received
            self.mutex.release()
            return
                
        N = len(msg.ranges)
        
        ranges_in_baselaser_frame = msg.ranges
        angles_in_baselaser_frame = [(msg.angle_max - msg.angle_min)*float(i)/N + msg.angle_min for i in range(len(msg.ranges))]
        angles_in_baselink_frame = angles_in_baselaser_frame[::-1]
        # This is because the z-axis of husky_1/base_laser is pointing downwards, while for husky_1/base_link and the map frame
        # the z-axis points upwards

        
        points_xyz_in_baselaser_frame = [np.array([r*cos(theta), r*sin(theta), 0])   for (r, theta) in zip(ranges_in_baselaser_frame, angles_in_baselaser_frame) if r < self.max_laser_range and r > self.min_laser_range]
        
        points_xyz_in_map_frame = self.from_laser_to_map_coordinates(points_xyz_in_baselaser_frame)
        
        baselaser_x_in_map = self.p_map_baselaser[0] 
        baselaser_y_in_map = self.p_map_baselaser[1] 
        baselaser_row, baselaser_col = self.ogm.cartesian_to_grid_coords(baselaser_x_in_map, baselaser_y_in_map)
        _, _, yaw_map_baselaser = tr.euler_from_quaternion(self.q_map_baselaser)
        _, _, yaw_map_baselink = tr.euler_from_quaternion(self.q_map_baselink)

        
        # Publishing the pose of the robot as a red arrow in rviz to help you debug
        ps = self._get_pose_marker(msg.header.stamp, 'map', self.p_map_baselaser, self.q_map_baselaser)

        # Publishing the points of the laser transformed into the map frame, as green points in rviz, to help you debug
        pts_marker = self._get_2d_laser_points_marker(msg.header.stamp, 'map', points_xyz_in_map_frame)
        self.mutex.release()

        self.robot_pose_pub.publish(ps)
        self.laser_points_marker_pub.publish(pts_marker)
        
        
        #
        # This is the main loop in occupancy grid mapping
        #
        max_laser_range_in_cells = int(self.max_laser_range / self.ogm.meters_per_cell) + 1 
        for delta_row in range(-max_laser_range_in_cells, max_laser_range_in_cells):
            for delta_col in range(-max_laser_range_in_cells, max_laser_range_in_cells):
                row = baselaser_row + delta_row
                col = baselaser_col + delta_col

                if row < 0 or row >= self.ogm.num_rows or col < 0 or col >= self.ogm.num_cols:
                    continue
                
                if self.is_in_field_of_view(baselaser_row, baselaser_col, yaw_map_baselink, row, col):
                    delta_log_odds = self.inverse_measurement_model(row,
                                                                    col,
                                                                    baselaser_row,
                                                                    baselaser_col,
                                                                    yaw_map_baselaser,
                                                                    ranges_in_baselaser_frame,
                                                                    angles_in_baselink_frame) - self.init_log_odds_ratio
                    
                    self.ogm.update_log_odds_ratio_in_grid_coords(row, col, delta_log_odds)
        
        self.occupancy_grid_pub.publish(self.ogm.get_map_as_ros_msg(msg.header.stamp, 'map'))
   

    def _get_2d_laser_points_marker(self, timestamp, frame_id, pts_in_map):
        msg = Marker()
        msg.header.stamp = timestamp
        msg.header.frame_id = frame_id
        msg.ns = 'laser_points'
        msg.id = 0
        msg.type = 6
        msg.action = 0
        msg.points = [Point(pt[0], pt[1], pt[2]) for pt in pts_in_map]
        msg.colors = [ColorRGBA(0, 1.0, 0, 1.0) for pt in pts_in_map]
        
        for pt in pts_in_map:
            assert((not np.isnan(pt).any()) and np.isfinite(pt).all())
        
        msg.scale.x = 0.1 
        msg.scale.y = 0.1
        msg.scale.z = 0.1
        return msg

    def _get_pose_marker(self, timestamp, frame_id, p, q):
        ps = PoseStamped()
        ps.pose.position.x = p[0]
        ps.pose.position.y = p[1]
        ps.pose.position.z = p[2]
        ps.pose.orientation.x = q[0]
        ps.pose.orientation.y = q[1]
        ps.pose.orientation.z = q[2]
        ps.pose.orientation.w = q[3]
        ps.header.stamp = timestamp
        ps.header.frame_id = frame_id
        return ps
        
    def run(self):
        self.rate = self.node.create_rate(200)
        
        while rclpy.ok():
            self.rate.sleep()

    
if __name__ == '__main__':
    num_rows = 250
    num_cols = 250
    meters_per_cell = 0.2
    hm = HuskyMapper(num_rows, num_cols, meters_per_cell)
    hm.run()
