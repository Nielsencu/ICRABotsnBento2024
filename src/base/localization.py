import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
import threading
import time
import numpy as np
import cv2
from sensor_msgs.msg import JointState, CompressedImage
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

class MainNode(Node):
    def __init__(self, config_path, node_name):
        super().__init__(node_name)
        
        self.robot_map = np.zeros((MAP_WIDTH_IN_PIXELS, MAP_HEIGHT_IN_PIXELS))
        
        # Subscription to wheel encoder
        self.joint_sub = self.create_subscription(JointState, 'joint_states', self.joint_state_cb, QoSProfile(depth=10))
        
        # Subscribe to ground camera 
        self.ground_camera_sub = self.create_subscription(
            CompressedImage,
            '/olive/camera/cam07/tags/image/compressed',
            self.ground_camera_cb,
            qos_profile=qos_profile_sensor_data)
        
        # TODO: Implement IMU subscription to get the header
        self.imu_sub = None
        
        # TODO: Create Finite state machines tfor exploration, detection of KLT and grabbing 
        self.state = RobotState.EXPLORATION
        
        self.thread_main = threading.Thread(target=self.thread_main)
        self.thread_exited = False
        self.rate_control_hz = 25
        
        self.thread_main.start()
        
        self.x = 0.0
        self.y = 0.0
        
        self.prev_joint_msg = None
        
        self.plan = None
        self.curID = 0
        
        # Set up the window for fullscreen or maximizable
        cv2.namedWindow("robotmap", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("robotmap", MAP_WIDTH_IN_PIXELS, MAP_HEIGHT_IN_PIXELS)  # Optional: Set an initial window size
        
    def ground_camera_cb(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Convert to OpenCV image
        
        # TODO: If ar tags detected, update x and y positions
        

    def thread_main(self):
        time.sleep(1)
        
        while not self.thread_exited:            
            time.sleep(1 / self.rate_control_hz)
            curPosInPixels = (self.x / MAP_RES, self.y / MAP_RES)
            
            # For each of the robot states, publish cmd_vel
            if self.state == RobotState.EXPLORATION:
                self.plan = get_global_plan_to_unexplored(curPosInPixels, self.robot_map)
                self.curID = 1
                
                # TODO: Compute linear and angular velocity from the target point
                
            elif self.state == RobotState.KLT_DETECTED:
                # TODO: Try to orientate and find the best gripping position
                ...
                
            elif self.state == RobotState.GRIPPING:
                # TODO: Command the servo motor to perform the gripping until successful
                ...
                
            elif self.state == RobotState.PARKING:
                # TODO: Get the correct AR Tag ID from the detection, and use 
                # a dictionary to get the corresponding parking location in the map
                
                parkingGoalInPixels = 0.0, 0.0
                self.plan = get_global_plan(curPosInPixels, parkingGoalInPixels, self.robot_map)
                self.curID = 1
                
                # TODO: Compute linear and angular velocity from the target point

    def joint_state_cb(self, msg):
        dFL, dFR, dBR, dBL = 0.0, 0.0, 0.0, 0.0
        if self.prev_joint_msg is not None:
            dFR = msg.position[0] - self.prev_joint_msg.position[0]
            dFL = msg.position[1] - self.prev_joint_msg.position[1]
            dBR = msg.position[2] - self.prev_joint_msg.position[2]
            dBL = msg.position[3] - self.prev_joint_msg.position[3]
        
        dForward = (dFL + dFR + dBL + dBR) / 4.0
        dRight = (-dBL + dBR + dFL -dFR) / 4.0
        
        # TODO: Use current header to transform dForward and dRight
        dx = dForward
        dy = dRight
        
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
