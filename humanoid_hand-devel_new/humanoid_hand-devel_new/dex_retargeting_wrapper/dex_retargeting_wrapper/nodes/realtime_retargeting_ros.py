import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image

import multiprocessing
import time
from pathlib import Path
from queue import Empty
from typing import Optional

import cv2
import numpy as np
import sapien
import tyro
from loguru import logger
from sapien.asset import create_dome_envmap
from sapien.utils import Viewer
import json

from dex_retargeting.constants import (
    RobotName,
    RetargetingType,
    HandType,
    get_default_config_path,
)
from dex_retargeting.retargeting_config import RetargetingConfig
from dex_retargeting_wrapper.dex_retargeting.example.vector_retargeting.single_hand_detector import SingleHandDetector

import yaml
import os
from pathlib import Path

class RealtimeRetargetingNode(Node):
    def __init__(self, config_filepath: str):
        super().__init__('realtime_retargeting_node')

        # Read configuration from config.yaml
        with open(config_filepath, 'r') as f:
            config = yaml.safe_load(f)

        robot_name_str = config.get("robot_name")
        optimizer_str = config.get("optimizer")
        hand_type_str = config.get("hand_type")

        self.robot_dir = config.get("robot_dir")
        
        match robot_name_str:
            case "allegro":
                self.robot_name = RobotName.allegro
            case "shadow":
                self.robot_name = RobotName.shadow
            case "svh":
                self.robot_name = RobotName.svh
            case "leap":
                self.robot_name = RobotName.leap
            case "ability":
                self.robot_name = RobotName.ability
            case "inspire":
                self.robot_name = RobotName.inspire
            case _:
                raise ValueError(f"Invalid choice of robot hand")

        match optimizer_str:
            case "vector":
                self.retargeting_type = RetargetingType.vector
            case "position":
                self.retargeting_type = RetargetingType.position
            case "dexpilot":
                self.retargeting_type = RetargetingType.dexpilot
            case _:
                raise ValueError(f"Invalid choice of optimizer.")

        match hand_type_str:
            case "right":
                self.hand_type = HandType.right
            case "left":
                self.hand_type = HandType.left
            case _:
                raise ValueError(f"Invalid hand type: {self.hand_type}")

        self.config_path = str(get_default_config_path(self.robot_name, self.retargeting_type, self.hand_type))

        self.results_queue = multiprocessing.Queue(maxsize=5)  # joint angles of target joints
        self.frame_queue = multiprocessing.Queue(maxsize=5)  
        self.results_all = multiprocessing.Queue(maxsize=5) # joint angles of all joints 
        self.consumer_process = multiprocessing.Process(target=self.start_retargeting,args=(self.frame_queue, self.results_queue, self.results_all, self.robot_dir, self.config_path))
        
        self.camera_topic = config.get("camera_topic")
        self.get_logger().info(f"Camera topic used: {self.camera_topic}")
        self.all_finger_publisher = self.create_publisher(JointState, "all_finger_joints",10)
        self.finger_publisher = self.create_publisher(JointState, "finger_joints",10)
        self.realsense_camera_subscriber = self.create_subscription(Image, self.camera_topic, self.realsense_camera_callback, 10)
        
        self.create_timer(0.01, self.publish_joint_angles)
        self.create_timer(0.01, self.publish_all_joint_angles)
        
        self.get_logger().info("Node initialized")        
        self.start_processes()
        
    def start_processes(self):
        self.get_logger().info("Starting processes")
        self.consumer_process.start()
        self.get_logger().info("Processes started")

    def publish_all_joint_angles(self):
        msg = JointState()
        msg.name = ['index_proximal_joint', 'index_intermediate_joint', 
            'middle_proximal_joint', 'middle_intermediate_joint',
            'pinky_proximal_joint', 'pinky_intermediate_joint',
            'ring_proximal_joint', 'ring_intermediate_joint', 
            'thumb_proximal_yaw_joint', 'thumb_proximal_pitch_joint', 
            'thumb_intermediate_joint', 'thumb_distal_joint']
        
        if not self.results_all.empty():
            qpos = self.results_all.get_nowait()
            qpos = [float(val) for val in qpos]
            msg.position = qpos
            self.all_finger_publisher.publish(msg)

    def publish_joint_angles(self):
        """Check the queue and publish any available data"""
        msg = JointState()
        msg.name = ['pinky_proximal_joint', 'ring_proximal_joint', 
                    'middle_proximal_joint', 'index_proximal_joint', 
                    'thumb_proximal_pitch_joint', 'thumb_proximal_yaw_joint']

        if not self.results_queue.empty():
            qpos = self.results_queue.get_nowait()
            qpos = [float(val) for val in qpos]
            msg.position = qpos
            # self.get_logger().info(f"Publishing qpos: {qpos}")
            self.finger_publisher.publish(msg)

    def realsense_camera_callback(self, msg):
        # get image from realsense camera
        bgr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        self.frame_queue.put(rgb)
        
    def start_retargeting(self, queue: multiprocessing.Queue, results_queue: multiprocessing.Queue, results_all_queue: multiprocessing.Queue, robot_dir: str, config_path: str):
        RetargetingConfig.set_default_urdf_dir(str(robot_dir))
        logger.info(f"Start retargeting with config {config_path}")
        retargeting = RetargetingConfig.load_from_file(config_path).build()

        hand_type = "Right" if "right" in config_path.lower() else "Left"
        detector = SingleHandDetector(hand_type=hand_type, selfie=False)

        sapien.render.set_viewer_shader_dir("default")
        sapien.render.set_camera_shader_dir("default")

        config = RetargetingConfig.load_from_file(config_path)
        target_joints = config.target_joint_names
        # Setup
        scene = sapien.Scene()
        render_mat = sapien.render.RenderMaterial()
        render_mat.base_color = [0.06, 0.08, 0.12, 1]
        render_mat.metallic = 0.0
        render_mat.roughness = 0.9
        render_mat.specular = 0.8
        scene.add_ground(-0.2, render_material=render_mat, render_half_size=[1000, 1000])

        # Lighting
        scene.add_directional_light(np.array([1, 1, -1]), np.array([3, 3, 3]))
        scene.add_point_light(np.array([2, 2, 2]), np.array([2, 2, 2]), shadow=False)
        scene.add_point_light(np.array([2, -2, 2]), np.array([2, 2, 2]), shadow=False)
        scene.set_environment_map(
            create_dome_envmap(sky_color=[0.2, 0.2, 0.2], ground_color=[0.2, 0.2, 0.2])
        )
        scene.add_area_light_for_ray_tracing(
            sapien.Pose([2, 1, 2], [0.707, 0, 0.707, 0]), np.array([1, 1, 1]), 5, 5
        )

        # Camera
        cam = scene.add_camera(
            name="Cheese!", width=600, height=600, fovy=1, near=0.1, far=10
        )
        cam.set_local_pose(sapien.Pose([0.50, 0, 0.0], [0, 0, 0, -1]))

        viewer = Viewer()
        viewer.set_scene(scene)
        viewer.control_window.show_origin_frame = False
        viewer.control_window.move_speed = 0.01
        viewer.control_window.toggle_camera_lines(False)
        viewer.set_camera_pose(cam.get_local_pose())

        # Load robot and set it to a good pose to take picture
        loader = scene.create_urdf_loader()
        print("Loader", loader)

        filepath = Path(config.urdf_path)
        robot_name = filepath.stem
        loader.load_multiple_collisions_from_file = True
        if "ability" in robot_name:
            loader.scale = 1.5
        elif "dclaw" in robot_name:
            loader.scale = 1.25
        elif "allegro" in robot_name:
            loader.scale = 1.4
        elif "shadow" in robot_name:
            loader.scale = 0.9
        elif "bhand" in robot_name:
            loader.scale = 1.5
        elif "leap" in robot_name:
            loader.scale = 1.4
        elif "svh" in robot_name:
            loader.scale = 1.5
        elif "inspire" in robot_name:
            loader.scale = 1.15

        if "glb" not in robot_name:
            filepath = str(filepath).replace(".urdf", "_glb.urdf")
        else:
            filepath = str(filepath)

        robot = loader.load(filepath)

        if "ability" in robot_name:
            robot.set_pose(sapien.Pose([0, 0, -0.15]))
        elif "shadow" in robot_name:
            robot.set_pose(sapien.Pose([0, 0, -0.2]))
        elif "dclaw" in robot_name:
            robot.set_pose(sapien.Pose([0, 0, -0.15]))
        elif "allegro" in robot_name:
            robot.set_pose(sapien.Pose([0, 0, -0.05]))
        elif "bhand" in robot_name:
            robot.set_pose(sapien.Pose([0, 0, -0.2]))
        elif "leap" in robot_name:
            robot.set_pose(sapien.Pose([0, 0, -0.15]))
        elif "svh" in robot_name:
            robot.set_pose(sapien.Pose([0, 0, -0.13]))

        # Different robot loader may have different orders for joints

        sapien_joint_names = [joint.get_name() for joint in robot.get_active_joints()]
        retargeting_joint_names = retargeting.joint_names
        retargeting_to_sapien = np.array(
            [retargeting_joint_names.index(name) for name in sapien_joint_names]
        ).astype(int)
        retarget_to_real = np.array(
            [retargeting_joint_names.index(name) for name in target_joints]
        )
        self.get_logger().info(f"Retargeting joint name order: {retargeting_joint_names}")
        self.get_logger().info(f"Target joint name order: {target_joints}")
        while True:
            try:
                bgr = queue.get(timeout=5)
                rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                rgb = cv2.resize(rgb, (640, 480))
            except Empty:
                logger.error(
                    "Fail to fetch image from camera in 5 secs. Please check your web camera device."
                )
                return


            _, joint_pos, keypoint_2d, _ = detector.detect(rgb)

            bgr = detector.draw_skeleton_on_image(bgr, keypoint_2d, style="default")
            cv2.imshow("realtime_retargeting_demo", bgr)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

            if joint_pos is None:
                logger.warning(f"{hand_type} hand is not detected.")
            else:
                retargeting_type = retargeting.optimizer.retargeting_type
                indices = retargeting.optimizer.target_link_human_indices
                if retargeting_type == "POSITION":
                    indices = indices
                    ref_value = joint_pos[indices, :]
                else:
                    origin_indices = indices[0, :]
                    task_indices = indices[1, :]
                    ref_value = joint_pos[task_indices, :] - joint_pos[origin_indices, :]
                qpos = retargeting.retarget(ref_value)
                results_sapien = qpos[retargeting_to_sapien].tolist()
                results_real = qpos[retarget_to_real].tolist()
                results_queue.put(results_real) 
                results_all_queue.put(results_sapien)
                robot.set_qpos(results_sapien)

            for _ in range(2):
                viewer.render()
        
def main(args=None):
    rclpy.init(args=args)
    node = None
    current_filepath = os.path.abspath(__file__)
    root = Path(current_filepath).parent.parent.parent.parent.parent
    node = RealtimeRetargetingNode(str(root / "src" / "humanoid_hand" / "config.yaml"))
    rclpy.spin(node)
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
