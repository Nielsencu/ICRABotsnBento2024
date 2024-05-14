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
from collections import deque

MAP_WIDTH_M = 5
MAP_HEIGHT_M = 5

MAP_RES = 0.01 # in meters

MAP_WIDTH_IN_PIXELS = int(MAP_WIDTH_M / MAP_RES)
MAP_HEIGHT_IN_PIXELS = int(MAP_HEIGHT_M / MAP_RES)

class RobotState(Enum):
    EXPLORATION = 0
    KLT_DETECTED = 1
    GRIPPING = 2
    PARKING = 3
    FOLLOWING_GLOBAL_PATH = 4

def get_position_from_artags_id(id) -> tuple[int, int]:
    id -= 101
    x = int((id // 5) + 0.5)
    y = int((id % 5) + 0.5)
    return (x, y)

def is_valid_front_artag(id):
    return 1 <= id <= 5

def is_valid_ground_artag(id):
    return 101 <= id <= 130

def get_dt_from_consecutive_msg(msg, prev_msg):
    return 

class MainNode(Node):
    def __init__(self, config_path, node_name):
        super().__init__(node_name)
        
        self.robot_map = np.zeros((MAP_WIDTH_IN_PIXELS, MAP_HEIGHT_IN_PIXELS))
        
        # Subscription to wheel encoder
        self.joint_sub = self.create_subscription(JointState, 'joint_states', self.joint_state_cb, QoSProfile(depth=10))
        
        self.imu_sub = self.create_subscription(Imu, 'imu', self.imu_cb, QoSProfile(depth=10))
        
        self.cmdvel_pub = self.create_publisher(Twist, 'cmd_vel', QoSProfile(depth=10))
        
        # Subscribe to ground camera 
        self.ground_camera_sub = self.create_subscription(
            CompressedImage,
            '/olive/camera/cam07/tags/image/compressed',
            self.ground_camera_cb,
            qos_profile=qos_profile_sensor_data
        )
        
        self.front_camera_sub = self.create_subscription(
            CompressedImage,
            '/olive/camera/cam08/tags/image/compressed',
            self.front_camera_cb,
            qos_profile=qos_profile_sensor_data
        )
        
        # TODO: Create Finite state machines for exploration, detection of KLT and grabbing 
        self.state = RobotState.EXPLORATION
        
        self.thread_main = threading.Thread(target=self.thread_main)
        self.thread_exited = False
        self.rate_control_hz = 25
        
        self.thread_main.start()
        
        self.x = 0.0
        self.y = 0.0
        
        self.yaw = 0.0
        
        self.prev_imu_msg = None
        self.prev_joint_msg = None
        
        self.plan = []
        self.curPlanID = 0
        
        self.has_global_plan = False
        self.has_completed_global_plan = False
        
        self.has_rotated_360 = False
        
        self.front_artag_id = -1
        
        # Set up the window for fullscreen or maximizable
        cv2.namedWindow("robotmap", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("robotmap", MAP_WIDTH_IN_PIXELS, MAP_HEIGHT_IN_PIXELS)  # Optional: Set an initial window size
        
    def ground_camera_cb(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Convert to OpenCV image
        
        # TODO: Retrieve ground artag id
        
        ground_artag_id = 0
        
        if is_valid_ground_artag(ground_artag_id):
            # Set x and y to ground truth by looking at AR Tags
            self.x, self.y = get_position_from_artags_id(ground_artag_id)
            
    def front_camera_cb(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Convert to OpenCV image
        
        # TODO: Retrieve front artag id
        front_artag_id = -1
        
        self.front_artag_id = front_artag_id
        
    def set_state(self, state):
        self.state = state
        
    def get_global_plan_target_point(self):
        # TODO: Improve target point for global plan
        
        target_point = self.plan[self.curPlanID]
        self.curPlanID += 1
        return target_point
        
    def get_cmd_vel_to_target_point(self, target_point):
        # TODO: Implement controtller to track target point (PID?)
        lin_x, ang_z = 0.0, 0.0    
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
            curPosInPixels = (self.x / MAP_RES, self.y / MAP_RES)
            
            print("Robot state : ", self.state)
            
            # For each of the robot states, publish cmd_vel
            if self.state == RobotState.EXPLORATION:
                if not self.has_global_plan:
                    self.plan = get_global_plan_to_unexplored(curPosInPixels, self.robot_map)
                    self.curPlanID = 1
                elif not self.has_completed_global_plan:
                    self.follow_global_plan()
                elif self.has_completed_global_plan:
                    # TODO: Perform a 360 degree rotation until a KLT is found
                    if self.has_rotated_360:
                        self.reset_global_plan()
                        continue
                    elif is_valid_front_artag(self.front_artag_id):
                        self.set_state(RobotState.KLT_DETECTED)
                        self.reset_global_plan()
            elif self.state == RobotState.KLT_DETECTED:
                # TODO: Try to orientate and find the best gripping position
                ...
                
            elif self.state == RobotState.PARKING:
                if not self.has_global_plan:
                    print("Front ARTag ID: ", self.front_artag_id)
                    if is_valid_front_artag(self.front_artag_id):
                        print("ARTag in the KLT not detected or not valid!!!, setting robot to exploration mode again")
                        self.state = RobotState.EXPLORATION
                        continue
                    
                    parking_spot_id = self.front_artag_id + 100
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
            elif self.state == RobotState.GRIPPING:
                # TODO: Command the servo motor to perform the gripping until successful
                ...
    
    def imu_cb(self, msg):
        # TODO: Consider madgwick filter?
        dt = get_dt_from_consecutive_msg(msg, self.prev_imu_msg)
        self.yaw += msg.angular_velocity.z * dt
        self.prev_imu_msg = msg

    def joint_state_cb(self, msg):
        dFL, dFR, dBR, dBL = 0.0, 0.0, 0.0, 0.0
        if self.prev_joint_msg is not None:
            dFR = msg.position[0] - self.prev_joint_msg.position[0]
            dFL = msg.position[1] - self.prev_joint_msg.position[1]
            dBR = msg.position[2] - self.prev_joint_msg.position[2]
            dBL = msg.position[3] - self.prev_joint_msg.position[3]
        
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
        
        cv2.imshow("robotmap", self.robot_map)
        # Close the window when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()
            exit()
        
        self.prev_joint_msg = msg
        

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
