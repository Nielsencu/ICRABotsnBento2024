import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
import threading
import time
import numpy as np
import cv2
from sensor_msgs.msg import JointState, CompressedImage, Imu
from rclpy.qos import qos_profile_sensor_data
from enum import Enum
from global_planner import get_global_plan, get_global_plan_to_unexplored
from tf_transformations import euler_from_quaternion
from apriltag_msgs.msg import AprilTagDetectionArray

MAP_WIDTH_M = 5
MAP_HEIGHT_M = 6

EXPLORATION_MAP_RES = 1 # in meters

EXPLORATION_MAP_WIDTH_IN_PIXELS = int(MAP_WIDTH_M / EXPLORATION_MAP_RES)
EXPLORATION_MAP_HEIGHT_IN_PIXELS = int(MAP_HEIGHT_M / EXPLORATION_MAP_RES)

ODOM_MAP_RES = 0.1

ODOM_MAP_WIDTH_IN_PIXELS = int(MAP_WIDTH_M / EXPLORATION_MAP_RES)
ODOM_MAP_HEIGHT_IN_PIXELS = int(MAP_HEIGHT_M / EXPLORATION_MAP_RES)

MAX_LIN_VEL = 0.15 # m/s
MAX_ANG_VEL = 0.1 # rad/s 

WHEEL_RADIUS = 10

class FSMState(Enum):
    EXPLORATION = 0
    KLT_DETECTED = 1
    PARKING = 2

def get_position_from_artags_id(id):
    id -= 101
    x = (id // 5) + 0.5
    y = (id % 5) + 0.5
    return (x, y)

def is_klt_artag(id):
    return 1 <= id <= 5

def is_ground_artag(id):
    return 101 <= id <= 130

def get_dt_from_consecutive_msg_in_secs(msg, prev_msg):
    return get_dt_from_timestamps(msg.header.stamp, prev_msg.header.stamp)

def get_dt_from_timestamps(cur_stamp, prev_stamp):
    cur_time_ns = (cur_stamp.sec * 10e-9) + cur_stamp.nanosec
    prev_time_ns = (prev_stamp.sec * 10e-9) + prev_stamp.nanosec
    return (cur_time_ns - prev_time_ns) * 10e-9

def get_l2_distance(pt1, pt2):
    dx = pt1[0] - pt2[0]
    dy = pt1[1] - pt2[1]
    return 0.5**(dx*dx + dy*dy) 

def print_valid_artags(detections):
    for detection in detections:    
        if is_klt_artag(detection.id):
            print("Valid KLT artag ID detected in front!", detection.id)
        elif is_ground_artag(detection.id):
            print("Valid Ground artag ID detected in front!", detection.id)
            artag_position = get_position_from_artags_id(detection.id)
            print("Updated x, y", artag_position)

class RobotState:
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw
        
    def pack_as_tuple_without_yaw(self):
        return (self.x, self.y)
    
    def pack_as_tuple(self):
        return (self.x, self.y, self.yaw)

class MainNode(Node):
    def __init__(self, config_path, node_name):
        super().__init__(node_name)
        
        self.odom_map = np.zeros((ODOM_MAP_HEIGHT_IN_PIXELS, ODOM_MAP_WIDTH_IN_PIXELS))
        self.exploration_map = np.zeros((EXPLORATION_MAP_HEIGHT_IN_PIXELS, EXPLORATION_MAP_WIDTH_IN_PIXELS))
        
        # Subscription to wheel encoder
        self.joint_fl_sub = self.create_subscription(JointState, '/olive/servo/motor03/joint/state', self.joint_fl_state_cb, QoSProfile(depth=10))
        self.joint_fr_sub = self.create_subscription(JointState, '/olive/servo/motor04/joint/state', self.joint_fr_state_cb, QoSProfile(depth=10))
        self.joint_br_sub = self.create_subscription(JointState, '/olive/servo/motor01/joint/state', self.joint_br_state_cb, QoSProfile(depth=10))
        self.joint_bl_sub = self.create_subscription(JointState, '/olive/servo/motor02/joint/state', self.joint_bl_state_cb, QoSProfile(depth=10))
        
        self.imu_sub = self.create_subscription(Imu, '/olive/imu/imu06/filtered_imu', self.imu_cb, QoSProfile(depth=10))
        
        self.cmdvel_pub = self.create_publisher(Twist, 'cmd_vel', QoSProfile(depth=10))
        
        self.front_artag_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/olive/camera/cam08/tags',
            self.front_artag_cb,
            qos_profile=qos_profile_sensor_data
        )
        
        self.back_artag_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/olive/camera/cam07/tags',
            self.back_artag_cb,
            qos_profile=qos_profile_sensor_data
        )
        
        self.state = FSMState.EXPLORATION
        
        self.thread_main = threading.Thread(target=self.thread_main)
        self.thread_exited = False
        self.rate_control_hz = 25
        
        self.thread_main.start()
        
        self.robot_state = RobotState(0.0, 0.0, 0.0)
        
        self.initial_yaw = 0.0

        self.dFR, self.dBR, self.dBL, self.dFL = 0.0, 0.0, 0.0, 0.0
        
        self.prev_imu_msg = None
        self.prev_fl_joint_msg = None
        self.prev_fr_joint_msg = None
        self.prev_bl_joint_msg = None
        self.prev_br_joint_msg = None
        
        self.cur_fl_joint_msg = None
        self.cur_fr_joint_msg = None
        self.cur_bl_joint_msg = None
        self.cur_br_joint_msg = None
        
        self.plan = []
        self.curWaypointID = 0
        
        self.has_completed_global_plan = False
        
        self.has_rotated_360 = False
        self.start_yaw_rotation = 0.0
        
        self.front_artags = []
        self.back_artags = []
        
        self.klt_artags = []
        
        self.parking_artag_id = -1
        
        self.last_wheel_odom_estimate = None
        
        # Set up the window for fullscreen or maximizable
        cv2.namedWindow("robotmap", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("robotmap", ODOM_MAP_WIDTH_IN_PIXELS, ODOM_MAP_HEIGHT_IN_PIXELS)  # Optional: Set an initial window size
        
    def front_artag_cb(self, msg):
        if len(msg.detections) > 0:
            print("Front camera detections!", len(msg.detections))
            self.front_artags = msg.detections
            print_valid_artags(self.front_artags)
            
            for artag in self.front_artags:
                if is_ground_artag(artag.id):
                    artag_position = get_position_from_artags_id(artag.id)
                    self.robot_state.x, self.robot_state.y = artag_position
                    self.exploration_map[artag_position[0]][artag_position[1]] = 1
                    break
            
    def back_artag_cb(self, msg):
        if len(msg.detections) > 0:
            print("Back camera detections!", len(msg.detections))
            self.back_artags = msg.detections
            print_valid_artags(self.back_artags)
            
            for artag in self.back_artags:
                if is_ground_artag(artag.id):
                    artag_position = get_position_from_artags_id(artag.id)
                    self.robot_state.x, self.robot_state.y = artag_position
                    self.exploration_map[artag_position[0]][artag_position[1]] = 1
                    break
    
    def set_state(self, state):
        self.state = state
        
    def get_klt_artags(self):
        klt_artags = []
        
        for artag in self.front_artags:
            if is_klt_artag(artag.id):
                klt_artags.append(artag)
                
        for artag in self.back_artags:
            if is_klt_artag(artag.id):
                klt_artags.append(artag)
                
        return klt_artags
        
    def get_global_plan_target_point(self):
        # TODO: Improve target point for global plan
        target_point = self.plan[self.curWaypointID]
        if get_l2_distance(self.robot_state.pack_as_tuple(), target_point) <= 0.1:
            self.curWaypointID += 1
        return target_point
    
    def get_diff_drive_control(self, target_point):
        
        robot_x, robot_y, robot_yaw = self.robot_state.pack_as_tuple()
        
        dy = target_point[1] - robot_y
        dx = target_point[0] - robot_x
        
        kp = 5
        
        dist_to_target = (dy*dy + dx*dx) ** 0.5
        
        forward_err = kp * dist_to_target
        
        lin_x = min(forward_err, MAX_LIN_VEL)
        
        # Assuming a x facing forward coordinate system
        goal_yaw = np.arctan2(dx, dy)
        
        ang_kp = 0.3
        
        angular_err = ang_kp * (goal_yaw - robot_yaw) 
        
        ang_z = max(min(angular_err, MAX_ANG_VEL), -MAX_ANG_VEL)
        
        if ang_z > 0.1:
            lin_x = 0.0
        
        return (lin_x, ang_z)
    
    def get_lateral_controller(self, target_point):
        ...
        
    def get_cmd_vel_to_target_point(self, target_point):
        # TODO: Improve controller, use pure pursuit?
        lin_x, lin_y, ang_z = 0.0, 0.0, 0.0
        
        if True:
            lin_x, ang_z = self.get_diff_drive_control(target_point)   
        
        return (lin_x, lin_y, ang_z)
    
    def follow_global_plan(self):
        target_point = self.get_global_plan_target_point()
        cmd_vel = self.get_cmd_vel_to_target_point(target_point)
        
        cmd = Twist()
        cmd.linear.x = cmd_vel[0]
        cmd.linear.y = cmd_vel[1]
        cmd.angular.z = cmd_vel[2]
        
        self.cmdvel_pub.publish(cmd)
        
    def reset_global_plan(self):
        self.plan = []
        self.curWaypointID = 0
        self.has_completed_global_plan = False
        
    def is_ready_to_grip(self):
        return True
    
    def grip(self):
        ...
        
    def release_grip(self):
        ...
        
    def get_closer_to_artags(self):
        ...
        
    def has_rotated_360(self):
        # Perform a 360 degree rotation to find KLT
        if abs(self.robot_state.yaw - self.start_yaw_rotation) > 1.9* np.pi:
            self.reset_global_plan()
            
        cmd = Twist()
        cmd.angular.z = 0.3

        self.cmdvel_pub.publish(cmd)

    def thread_main(self):
        time.sleep(1)
        
        while not self.thread_exited:            
            time.sleep(1 / self.rate_control_hz)
            
            self.update_localization()
            
            curPosInPixels = (self.robot_state.x / ODOM_MAP_RES, self.robot_state.y / ODOM_MAP_RES)
            
            print("Robot state : ", self.state, " pos ", self.robot_state.pack_as_tuple())
            
            # For each of the robot states, publish cmd_vel
            if self.state == FSMState.EXPLORATION:
                self.klt_artags = self.get_klt_artags()
                if len(self.klt_artags) > 0:
                    self.set_state(FSMState.KLT_DETECTED)
                    self.reset_global_plan()
                    continue
                
                if len(self.plan) == 0:
                    self.plan = [(0, 0), (1, 0), (2, 0)]
                    # self.plan = get_global_plan_to_unexplored(self.robot_state.pack_as_tuple_without_yaw(), self.exploration_map)
                    print("Plan is ", self.plan)
                elif not self.has_completed_global_plan:
                    self.follow_global_plan()
                elif self.has_completed_global_plan:
                    if self.has_rotated_360():
                        self.set_state(FSMState.EXPLORATION)
                    
            elif self.state == FSMState.KLT_DETECTED:
                
                # Set the parking artag id to the first one detected
                if self.parking_artag_id == -1:
                    self.parking_artag_id = self.klt_artags[0]
                    for artag in self.klt_artags:
                        print("KLT ARTag ID: ", artag.id)

                # TODO: Try to orientate and find the best gripping position
                if self.is_ready_to_grip():
                    self.grip()
                    self.set_state(FSMState.PARKING)
                    continue
                self.get_closer_to_artags()
                    
            elif self.state == FSMState.PARKING:
                if len(self.plan) == 0:
                    parking_spot_id = self.parking_artag_id + 100
                    print("Going to parking spot id of ", parking_spot_id)
                    parkingGoalPos = get_position_from_artags_id(parking_spot_id)
                    print("Parking spot posiiton ", parkingGoalPos)
                    parkingGoalInPixels = parkingGoalPos * EXPLORATION_MAP_RES
                    
                    self.plan = get_global_plan(curPosInPixels, parkingGoalInPixels, self.exploration_map)
                elif not self.has_completed_global_plan:
                    self.follow_global_plan()
                elif self.has_completed_global_plan:
                    #TODO: Release the gripper to park, probably want to go forward first b4 release?
                    self.release_grip()
                    # Set robot state back to exploration for finding gripper
                    self.set_state(FSMState.EXPLORATION)    
                    self.parking_artag_id = -1
                    self.reset_global_plan()                    
    
    def imu_cb(self, msg):
        # TODO: Consider madgwick filter?
        dt = 0.0
        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        (_, _, yaw) = euler_from_quaternion(quat)
        if self.prev_imu_msg is None:
            self.initial_yaw = yaw
        if self.prev_imu_msg is not None:
            dt = get_dt_from_consecutive_msg_in_secs(msg, self.prev_imu_msg)
            
        # print("ang vel z is ", msg.angular_velocity.z)
        # print("dt is ", dt )
        # self.yaw = (self.yaw + (msg.angular_velocity.z * dt)) % np.pi
        # print("Yaw is ", self.yaw)
        
        self.robot_state.yaw = yaw - self.initial_yaw
        
        print("Yaw: ", self.robot_state.yaw)
        
        self.prev_imu_msg = msg
        
    def joint_fr_state_cb(self, msg):
        if self.prev_fr_joint_msg is not None:
            self.dFR += msg.position[0] - self.prev_fr_joint_msg.position[0]
        self.prev_fr_joint_msg = None
        # self.cur_fr_joint_msg = msg
        
    def joint_br_state_cb(self, msg):
        if self.prev_br_joint_msg is not None:
            self.dBR += msg.position[0] - self.prev_br_joint_msg.position[0]
        self.prev_br_joint_msg = None
        # self.cur_br_joint_msg = msg
        
    def joint_bl_state_cb(self, msg):
        if self.prev_bl_joint_msg is not None:
            self.dBL += msg.position[0] - self.prev_bl_joint_msg.position[0]
        self.prev_bl_joint_msg = None
        # self.cur_bl_joint_msg = msg
        
    def joint_fl_state_cb(self, msg):
        if self.prev_fl_joint_msg is not None:
            self.dFL += msg.position[0] - self.prev_fl_joint_msg.position[0]
        self.prev_fl_joint_msg = None
        # self.cur_fl_joint_msg = msg
        
    def update_localization(self):
        # dFL, dFR, dBL, dBR = 0.0, 0.0, 0.0, 0.0
        
        # if self.prev_fl_joint_msg is not None:
        #     dFL = self.cur_fl_joint_msg.position[0] - self.prev_fl_joint_msg.position[0]
        #     dBL = self.cur_bl_joint_msg.position[0] - self.prev_bl_joint_msg.position[0]
        #     dBR = self.cur_br_joint_msg.position[0] - self.prev_br_joint_msg.position[0]
        #     dFR = self.cur_fr_joint_msg.position[0] - self.prev_fr_joint_msg.position[0]
            
        # # print(dFL, dFR, dBL, dBR)
        
        # self.prev_fr_joint_msg = self.cur_fr_joint_msg
        # self.prev_fl_joint_msg = self.cur_fl_joint_msg
        # self.prev_br_joint_msg = self.cur_br_joint_msg
        # self.prev_bl_joint_msg = self.cur_bl_joint_msg
                
        dFL, dFR, dBL, dBR = self.dFL, self.dFR, self.dBL, self.dBR
        
        print("dthetas ", dFL, dFR, dBL, dBR)
        
        dForwardTheta = (dFL + dFR + dBL + dBR) / 4.0
        dRightTheta = (-dBL + dBR + dFL -dFR) / 4.0
        
        print("dForwardTheta", dForwardTheta)
        
        dt = 0.001
        cur_time = self.get_clock().now().to_msg()
        if self.last_wheel_odom_estimate is not None:
            dt = get_dt_from_timestamps(cur_time, self.last_wheel_odom_estimate)
            
        lin_vel_x = (WHEEL_RADIUS) * (dForwardTheta / dt)
        lin_vel_y = (WHEEL_RADIUS) * (dRightTheta / dt)
        
        print("lin_vel_x", lin_vel_x)
        
        dForward = lin_vel_x * dt
        dRight = lin_vel_y * dt
        
        print("dforward ", dForward)
        
        # Clockwise rotation matrix
        yaw = self.robot_state.yaw
        
        base2world = np.array([
            [np.cos(yaw), np.sin(yaw)],
            [-np.sin(yaw), np.cos(yaw)]
        ])
        
        dx, dy = np.dot(base2world, np.array([dForward, dRight]))
        
        self.robot_state.x += dx
        self.robot_state.y += dy
        
        xInPixels = int(self.robot_state.x / ODOM_MAP_RES)
        yInPixels = int(self.robot_state.y / ODOM_MAP_RES)
        
        self.odom_map[xInPixels][yInPixels] = 255
        
        self.dFL, self.dFR, self.dBL, self.dBR = 0.0, 0.0, 0.0, 0.0
        
        self.last_wheel_odom_estimate = cur_time
        
        # cv2.imshow("odom map", self.odom_map)
        # Close the window when 'q' is pressed
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     cv2.destroyAllWindows()
        #     rclpy.shutdown()
        #     exit()
            
        # cv2.imshow("explorations map", self.exploration_map)
        # # Close the window when 'q' is pressed
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     cv2.destroyAllWindows()
        #     rclpy.shutdown()
        #     exit()

    def __del__(self):
        self.thread_exited = True
        if self.thread_main.is_alive():
            self.thread_main.join()

if __name__ == '__main__':
    rclpy.init()
    main_node = MainNode("config/path", "main_node")
    print("Localization node running")
    rclpy.spin(main_node)
    main_node.destroy_node()
    rclpy.shutdown()
