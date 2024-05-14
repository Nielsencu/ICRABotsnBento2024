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
MAP_HEIGHT_M = 5

MAP_RES = 0.1 # in meters

MAP_WIDTH_IN_PIXELS = int(MAP_WIDTH_M / MAP_RES)
MAP_HEIGHT_IN_PIXELS = int(MAP_HEIGHT_M / MAP_RES)

MAX_LIN_VEL = 0.15 # m/s
MAX_ANG_VEL = 0.1 # rad/s 

class RobotState(Enum):
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
    cur_time_ns = (msg.header.stamp.sec * 10e-9) + msg.header.stamp.nanosec
    prev_time_ns = (prev_msg.header.stamp.sec * 10e-9) + prev_msg.header.stamp.nanosec
    return (cur_time_ns - prev_time_ns) * 10e-9

def get_l2_distance(pt1, pt2):
    dx = pt1[0] - pt2[0]
    dy = pt1[1] - pt2[1]
    return 0.5**(dx*dx + dy*dy) 

class MainNode(Node):
    def __init__(self, config_path, node_name):
        super().__init__(node_name)
        
        self.robot_map = np.zeros((MAP_WIDTH_IN_PIXELS, MAP_HEIGHT_IN_PIXELS))
        
        # Subscription to wheel encoder
        self.joint_fl_sub = self.create_subscription(JointState, '/olive/servo/motor01/joint/state', self.joint_fl_state_cb, QoSProfile(depth=10))
        self.joint_fr_sub = self.create_subscription(JointState, '/olive/servo/motor02/joint/state', self.joint_fr_state_cb, QoSProfile(depth=10))
        self.joint_br_sub = self.create_subscription(JointState, '/olive/servo/motor03/joint/state', self.joint_br_state_cb, QoSProfile(depth=10))
        self.joint_bl_sub = self.create_subscription(JointState, '/olive/servo/motor04/joint/state', self.joint_bl_state_cb, QoSProfile(depth=10))
        
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
        
        self.state = RobotState.EXPLORATION
        
        self.thread_main = threading.Thread(target=self.thread_main)
        self.thread_exited = False
        self.rate_control_hz = 25
        
        self.thread_main.start()
        
        self.x = 0.0
        self.y = 0.0
        
        self.initial_yaw = 0.0
        self.yaw = 0.0
        
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
        self.curPlanID = 0
        
        self.has_completed_global_plan = False
        
        self.has_rotated_360 = False
        self.start_yaw_rotation = 0.0
        
        self.front_artag_id = -1
        self.back_artag_id = -1
        
        # Set up the window for fullscreen or maximizable
        cv2.namedWindow("robotmap", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("robotmap", MAP_WIDTH_IN_PIXELS, MAP_HEIGHT_IN_PIXELS)  # Optional: Set an initial window size
        
    def front_artag_cb(self, msg):
        if len(msg.detections) > 0:
            print("Front camera detections!", len(msg.detections))
            self.front_artag_id = msg.detections[0].id
            
        if is_klt_artag(self.front_artag_id):
            print("Valid KLT artag ID detected in front!", self.front_artag_id)
        elif is_ground_artag(self.front_artag_id):
            print("Valid Ground artag ID detected in front!", self.front_artag_id)
            artag_position = get_position_from_artags_id(self.front_artag_id)
            self.x, self.y = artag_position
            print("Updated x, y", self.x, self.y)
            
    def back_artag_cb(self, msg):
        if len(msg.detections) > 0:
            print("Back camera detections!", len(msg.detections))
            self.back_artag_id = msg.detections[0].id
            
        if is_klt_artag(self.back_artag_id):
            print("Valid KLT artag ID detected at back!", self.back_artag_id)
        elif is_ground_artag(self.back_artag_id):
            print("Valid Ground artag ID detected at back!", self.back_artag_id)
            artag_position = get_position_from_artags_id(self.back_artag_id)
            self.x, self.y = artag_position
            print("Updated x, y", self.x, self.y)
    
    def set_state(self, state):
        self.state = state
        
    def get_global_plan_target_point(self):
        # TODO: Improve target point for global plan
        
        target_point = self.plan[self.curPlanID]
        self.curPlanID += 1
        return target_point
        
    def get_cmd_vel_to_target_point(self, target_point):
        # TODO: Improve controller, use pure pursuit?
        lin_x, ang_z = 0.0, 0.0    
        
        dy = target_point[1] - self.y
        dx = target_point[0] - self.x
        
        kp = 0.5
        
        dist_to_target = (dy*dy + dx*dx) ** 0.5
        
        forward_err = kp * dist_to_target
        
        lin_x = min(forward_err, MAX_LIN_VEL)
        
        goal_yaw = np.arctan2(dy, dx)
        
        ang_kp = 0.3
        
        angular_err = ang_kp * (goal_yaw - self.yaw) 
        
        ang_z = max(min(angular_err, MAX_ANG_VEL), -MAX_ANG_VEL)
        
        return (lin_x, ang_z)
    
    def follow_global_plan(self):
        target_point = self.get_global_plan_target_point()
        cmd_vel = self.get_cmd_vel_to_target_point(target_point)
        
        cmd = Twist()
        cmd.linear.x = cmd_vel[0]
        cmd.angular.z = cmd_vel[1]
        
        self.cmdvel_pub.publish(cmd)
        
    def reset_global_plan(self):
        self.plan = []
        self.curPlanID = 0
        self.has_completed_global_plan = False

    def thread_main(self):
        time.sleep(1)
        
        while not self.thread_exited:            
            time.sleep(1 / self.rate_control_hz)
            
            self.update_localization()
            
            curPosInPixels = (self.x / MAP_RES, self.y / MAP_RES)
            
            print("Robot state : ", self.state)
            
            # For each of the robot states, publish cmd_vel
            if self.state == RobotState.EXPLORATION:
                if is_klt_artag(self.front_artag_id) or is_klt_artag(self.back_artag_id):
                    self.set_state(RobotState.KLT_DETECTED)
                    self.reset_global_plan()
                    continue
                
                if len(self.plan) == 0:
                    self.plan = get_global_plan_to_unexplored(curPosInPixels, self.robot_map)
                    self.curPlanID = 1
                elif not self.has_completed_global_plan:
                    self.follow_global_plan()
                elif self.has_completed_global_plan:
                    if not self.started_rotating_360:
                        self.start_yaw_rotation = self.yaw
                        self.started_rotating_360 = True
                        
                    # Perform a 360 degree rotation to find KLT
                    if abs(self.yaw - self.start_yaw_rotation) > 1.9* np.pi:
                        self.reset_global_plan()
                        self.started_rotating_360 = False
                        continue
                    
                    cmd = Twist()
                    cmd.angular.z = 0.3
        
                    self.cmdvel_pub.publish(cmd)
                    
            elif self.state == RobotState.KLT_DETECTED:
                # TODO: Try to orientate and find the best gripping position
                ...
                
            elif self.state == RobotState.PARKING:
                if len(self.plan) == 0:
                    print("Front ARTag ID: ", self.front_artag_id)
                    artag_id = -1
                    if is_klt_artag(self.front_artag_id):
                        artag_id = self.front_artag_id
                    elif is_klt_artag(self.back_artag_id):
                        artag_id = self.back_artag_id
                    if artag_id == -1:
                        print("ARTag in the KLT not detected or not valid!!!, setting robot to exploration mode again")
                        self.set_state(RobotState.EXPLORATION)
                        self.reset_global_plan()
                        continue
                    
                    parking_spot_id = artag_id + 100
                    print("Going to parking spot id of ", parking_spot_id)
                    parkingGoalPos = get_position_from_artags_id(parking_spot_id)
                    print("Parking spot posiiton ", parkingGoalPos)
                    parkingGoalInPixels = parkingGoalPos * MAP_RES
                    
                    self.plan = get_global_plan(curPosInPixels, parkingGoalInPixels, self.robot_map)
                    self.curPlanID = 1
                elif not self.has_completed_global_plan:
                    self.follow_global_plan()
                elif self.has_completed_global_plan:
                    #TODO: Release the gripper to park, probably want to go forward first b4 release?
                    ...
                    
                    # Set robot state back to exploration for finding gripper
                    self.set_state(RobotState.EXPLORATION)    
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
        
        self.yaw = yaw - self.initial_yaw
        
        print(self.get_cmd_vel_to_target_point((1,1)))
        print("Yaw: ", self.yaw)
        
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
            
        dForward = (dFL + dFR + dBL + dBR) / 4.0
        dRight = (-dBL + dBR + dFL -dFR) / 4.0
        
        # Clockwise rotation matrix
        base2world = np.array([
            [np.cos(self.yaw), np.sin(self.yaw)],
            [-np.sin(self.yaw), np.cos(self.yaw)]
        ])
        
        dx, dy = np.dot(base2world, np.array([dForward, dRight]))
        
        self.x += dx
        self.y += dy
        
        xInPixels = int(self.x / MAP_RES)
        yInPixels = int(self.y / MAP_RES)
        
        self.robot_map[yInPixels][xInPixels] = 255
        
        self.dFL, self.dFR, self.dBL, self.dBR = 0.0, 0.0, 0.0, 0.0
        
        # cv2.imshow("robotmap", self.robot_map)
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
