import rclpy
from rclpy.node import Node
import time
from humanoid_hand_controller.utils import BaseHand
from humanoid_hand_controller.gestures import HandGestures
from service_interfaces.msg import GetForceAct1, GetAngleAct1, GetTouchAct1, SetAngle1, SetForce1, SetSpeed1, GetTemp1 
import numpy as np
from std_msgs.msg import Float32MultiArray
from service_interfaces.msg import SetAngle1
from ccma import CCMA
from collections import deque
import yaml
from pathlib import Path
import os

class HandNode(HandGestures):
    def __init__(self, config_filepath: str):
        super().__init__('hand_node', config_filepath)
        self.get_logger().info(f"Hand node started")

        # Read configuration from config.yaml
        with open(config_filepath, 'r') as f:
            config = yaml.safe_load(f)

        
        self.buffer_size = int(config.get("buffer_size"))
        self.threshold = int(config.get("threshold"))
        self.ccma = CCMA()
        self.joint_history = {
            "pinky_proximal_joint": deque([1000] * self.buffer_size, maxlen=self.buffer_size),
            "ring_proximal_joint": deque([1000] * self.buffer_size, maxlen=self.buffer_size),
            "middle_proximal_joint": deque([1000] * self.buffer_size, maxlen=self.buffer_size),
            "index_proximal_joint": deque([1000] * self.buffer_size, maxlen=self.buffer_size),
            "thumb_proximal_pitch_joint": deque([1000] * self.buffer_size, maxlen=self.buffer_size),
            "thumb_proximal_yaw_joint": deque([1000] * self.buffer_size, maxlen=self.buffer_size),
        }
        self.current_angle = [1000, 1000, 1000, 1000, 1000, 1000]
        self.latest_joint_values = None
        self.joint_subscription = self.create_subscription(Float32MultiArray, "finger_joints", self.joint_callback, 10)
        self.set_angle_publisher = self.create_publisher(SetAngle1, 'set_angle_data', 10)
        self.get_angle_subscriber = self.create_subscription(GetAngleAct1, 'angle_data', self.get_angle_callback, 10)

        self.angle_publisher = self.create_publisher(Float32MultiArray, "retarget_joints", 10)
        # self.create_timer(0.01, self.publish_current_angles)
        self.initialize()

    def initialize(self):
        self.get_logger().info("Going to home position")
        self.set_angle_and_wait(1000, 1000, 1000, 1000, 1000, 1000)
        time.sleep(2)
        self.calibrate_force()
        time.sleep(2)
        self.get_logger().info("Calibrating force sensor done")
       
        # Create overall path
        self.overall_path = []

        for key, path in self.gestures_paths.items():
            self.overall_path.extend(path)
        self.overall_path = self.remove_redundant_nodes(self.overall_path)
        self.get_logger().info(f"Overall path: {self.overall_path}")

        for node in self.overall_path:
            angles = [int(angle) for angle in self.gestures[node]]

            self.set_angle_and_wait(*angles)
            self.get_logger().info(f"Angle reached: {self.angle_reached}")
            
            while not self.angle_reached:
                rclpy.spin_once(self, timeout_sec=0.1)  # Process callbacks while waiting
    
    # def publish_current_angles(self):
    #     msg = Float32MultiArray()
    #     print(self.current_angle.angles)
    #     angles = [float(angle) for angle in self.current_angle.angles]
    #     msg.data = angles
    #     self.angle_publisher.publish(msg)

    def get_angle_callback(self, msg):
        if self.current_angle is None:
            self.current_angle = msg.angles
        else:
            angles = np.array(msg.angles)
            current_angle = np.array(self.current_angle)
            for i in range(len(angles)):
                if abs(angles[i] - current_angle[i]) > self.threshold:
                    self.current_angle[i] = angles[i]
    
    def joint_callback(self, msg):
        """Callback for receiving joint values from the retargeting node"""
        self.latest_joint_values = msg.data
        set_angle_msg = SetAngle1()
        set_angle_msg.finger_ids = [1, 2, 3, 4, 5, 6]

        if self.latest_joint_values is not None:
            # only for index finger
            pinky_proximal_joint = self.latest_joint_values[0]
            ring_proximal_joint = self.latest_joint_values[1]
            middle_proximal_joint = self.latest_joint_values[2]
            index_proximal_joint = self.latest_joint_values[3]
            thumb_proximal_pitch_joint = self.latest_joint_values[4]
            thumb_proximal_yaw_joint = self.latest_joint_values[5]

            angle0 = abs((1000 / 0.9) * (1.47 - pinky_proximal_joint))
            angle1 = abs((1000 / 1.2) * (1.47 - ring_proximal_joint))
            angle2 = abs((1000 / 1.2) * (1.47 - middle_proximal_joint))
            angle3 = abs((1000 / 1.2) * (1.47 - index_proximal_joint))
            angle4 = abs((1000 / 0.3) * (0.3 - thumb_proximal_pitch_joint))
            angle5 = abs((1000 / 1.308) * (1.308 - thumb_proximal_yaw_joint))

            self.joint_history["pinky_proximal_joint"].append(angle0)
            self.joint_history["ring_proximal_joint"].append(angle1)
            self.joint_history["middle_proximal_joint"].append(angle2)
            self.joint_history["index_proximal_joint"].append(angle3)
            self.joint_history["thumb_proximal_pitch_joint"].append(angle4)
            self.joint_history["thumb_proximal_yaw_joint"].append(angle5)

            # smooth the angles
            if len(self.joint_history["pinky_proximal_joint"]) == self.buffer_size:
                time = np.arange(self.buffer_size)
                angle0 = np.hstack([time[:, None], np.array(self.joint_history["pinky_proximal_joint"])[:, None]])
                angle0 = self.ccma.filter(angle0)[-1][1]

                angle1 = np.hstack([time[:, None], np.array(self.joint_history["ring_proximal_joint"])[:, None]])
                angle1 = self.ccma.filter(angle1)[-1][1]

                angle2 = np.hstack([time[:, None], np.array(self.joint_history["middle_proximal_joint"])[:, None]])
                angle2 = self.ccma.filter(angle2)[-1][1]

                angle3 = np.hstack([time[:, None], np.array(self.joint_history["index_proximal_joint"])[:, None]])
                angle3 = self.ccma.filter(angle3)[-1][1]

                angle4 = np.hstack([time[:, None], np.array(self.joint_history["thumb_proximal_pitch_joint"])[:, None]])
                angle4 = self.ccma.filter(angle4)[-1][1]

                angle5 = np.hstack([time[:, None], np.array(self.joint_history["thumb_proximal_yaw_joint"])[:, None]])
                angle5 = self.ccma.filter(angle5)[-1][1]
                
            angle0 = angle0 if abs(self.current_angle[0] - angle0) > self.threshold else self.current_angle[0]
            angle1 = angle1 if abs(self.current_angle[1] - angle1) > self.threshold else self.current_angle[1]
            angle2 = angle2 if abs(self.current_angle[2] - angle2) > self.threshold else self.current_angle[2]
            angle3 = angle3 if abs(self.current_angle[3] - angle3) > self.threshold else self.current_angle[3]
            angle4 = angle4 if abs(self.current_angle[4] - angle4) > self.threshold else self.current_angle[4]
            angle5 = angle5 if abs(self.current_angle[5] - angle5) > self.threshold else self.current_angle[5]
            
            set_angle_msg.angles = [int(angle0), int(angle1), int(angle2), int(angle3), int(angle4), int(angle5)]
            self.set_angle_publisher.publish(set_angle_msg)

def main(args=None):
    print("Starting hand test node")
    rclpy.init(args=args)
    current_filepath = os.path.abspath(__file__)
    root = Path(current_filepath).parent.parent.parent.parent
    node = HandNode(str(root / "src" / "humanoid_hand" / "config.yaml"))
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    