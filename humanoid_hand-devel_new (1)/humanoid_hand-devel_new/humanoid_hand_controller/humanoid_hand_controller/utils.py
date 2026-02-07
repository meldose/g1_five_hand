import rclpy
from rclpy.node import Node
from typing import List
import serial
import time
import numpy as np
from functools import partial
from service_interfaces.srv import Setpos, Setangle, Setforce, Setspeed, Setforceclb
from service_interfaces.msg import GetAngleAct1, GetForceAct1, GetTouchAct1, SetAngle1
from pymodbus.client import ModbusTcpClient
from pymodbus.pdu import ExceptionResponse
import yaml


class BaseHand(Node):
    def __init__(self, node_name: str, config_filepath: str):
        super().__init__(node_name)
        self.hand_id = 1

        with open(config_filepath, 'r') as file:
            config = yaml.safe_load(file)

        self.get_logger().info(f"Hand {self.hand_id} initialized")
        self.tolerance = 20

        self.angle_reached = False
        self.angle_set = [1000, 1000, 1000, 1000, 1000, 1000] # default set angle 
        self.angle_act_subscriber = self.create_subscription(GetAngleAct1, 'angle_data', self.is_angle_complete_callback, 10)
        self.angle_publisher = self.create_publisher(SetAngle1, 'set_angle_data', 10)

        # initialize modbus client
        self.MODBUS_IP = config.get('MODBUS_IP')
        self.MODBUS_PORT = config.get('MODBUS_PORT')
        self.modbus_client = ModbusTcpClient(self.MODBUS_IP, port = self.MODBUS_PORT)
        self.modbus_client.connect()

    def is_angle_complete_callback(self, msg):
        """
        Reads the angle sensor data and checks if the angle is complete
        """
        fingers = msg.finger_ids
        current_angles = np.array(msg.angles)
        set_angles = np.array(self.angle_set)
        self.get_logger().info(f"Current angles: {current_angles}, Set angles: {set_angles}")
        if self.angle_set is not None:
            if np.all(np.abs(current_angles - set_angles) < self.tolerance):
                self.angle_reached = True

                self.get_logger().info(f"Angle reached: {self.angle_reached}")
        
    def set_angle_and_wait(self, angle0, angle1, angle2, angle3, angle4, angle5):
        
        self.get_logger().info(f"Setting angle: [{angle0}, {angle1}, {angle2}, {angle3}, {angle4}, {angle5}]")
        self.angle_reached = False
        self.angle_set = [angle0, angle1, angle2, angle3, angle4, angle5]

        angle_msg = SetAngle1()
        angle_msg.finger_ids = [1, 2, 3, 4, 5, 6]
        angle_msg.angles = [angle0, angle1, angle2, angle3, angle4, angle5]

        self.angle_publisher.publish(angle_msg)

    def calibrate_force(self):
        """
        Calibrates the force sensor via modbus
        """
        self.get_logger().info("Calibrating force sensor")
        self.write_signed_register(1009, 1)


    def call_set_force_service(self, force0, force1, force2, force3, force4, force5):
        self.get_logger().info(f"Calling set_force service with force0: {force0}, force1: {force1}, force2: {force2}, force3: {force3}, force4: {force4}, force5: {force5}")
        client = self.create_client(Setforce, '/Setforce')
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Service not available")
            return
        
        request = Setforce.Request()
        request.status = "set_force"
        request.id = self.hand_id
        request.force0 = force0
        request.force1 = force1
        request.force2 = force2
        request.force3 = force3
        request.force4 = force4
        request.force5 = force5

        future = client.call_async(request)
        future.add_done_callback(self.callback_set_force_service)
    
    def callback_set_force_service(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return False
    
    def call_set_speed_service(self, speed0, speed1, speed2, speed3, speed4, speed5):
        self.get_logger().info(f"Calling set_speed service with speed0: {speed0}, speed1: {speed1}, speed2: {speed2}, speed3: {speed3}, speed4: {speed4}, speed5: {speed5}")
        client = self.create_client(Setspeed, '/Setspeed')
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Service not available")
            return
        
        request = Setspeed.Request()
        request.status = "set_speed"
        request.id = self.hand_id
        request.speed0 = speed0
        request.speed1 = speed1
        request.speed2 = speed2
        request.speed3 = speed3
        request.speed4 = speed4
        request.speed5 = speed5

        future = client.call_async(request)
        future.add_done_callback(self.callback_set_speed_service)

    def callback_set_speed_service(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
        
    def call_force_calibration_service(self):
        client = self.create_client(Setforceclb, '/Setforceclb')
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Service not available")
            return

        request = Setforceclb.Request() 
        request.status = "set_forceclb"
        request.id = self.hand_id

        future = client.call_async(request)
        future.add_done_callback(self.callback_set_force_calibration_service)

    def callback_set_force_calibration_service(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def write_signed_register(self, address, value):
        if value < 0:
            value += 65536
        response = self.modbus_client.write_register(address, value)

        if isinstance(response, ExceptionResponse) or response.isError():
            self.get_logger().error(f"Failed to write register {address}: {response}")
