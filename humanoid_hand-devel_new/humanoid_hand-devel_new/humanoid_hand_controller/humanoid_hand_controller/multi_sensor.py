#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import threading
import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusTcpClient
from pymodbus.pdu import ExceptionResponse
from service_interfaces.msg import GetForceAct1, GetAngleAct1, GetTouchAct1, SetAngle1, SetForce1, SetSpeed1, GetTemp1 
import yaml
import os
from pathlib import Path

FORCE_SENSOR_RANGES = {
    1: (1582,),  # Pinky
    2: (1584,),  # Ring Finger
    3: (1586,),  # Middle Finger
    4: (1588,),  # Index Finger
    5: (1590,),  # Thumb Flexion
    6: (1592,),  # Thumb Abduction
}

TOUCH_ACT_RANGES = {
    1: (3000, 3369),  # Pinky
    2: (3370, 3739),  # Ring Finger
    3: (3740, 4109),  # Middle Finger
    4: (4110, 4479),  # Index Finger
    5: (4480, 4899),  # Thumb
    7: (4900, 5123),  # Palm
}

FORCE_SET_RANGES = {
    1: (1498,),  # Pinky
    2: (1500,),  # Ring Finger
    3: (1502,),  # Middle Finger
    4: (1504,),  # Index Finger
    5: (1506,),  # Thumb Flexion
    6: (1508,),  # Thumb Abduction
}

SPEED_SET_RANGES = {
    1: (1522,),  # Pinky
    2: (1524,),  # Ring Finger
    3: (1526,),  # Middle Finger
    4: (1528,),  # Index Finger
    5: (1530,),  # Thumb Flexion
    6: (1532,),  # Thumb Abduction
}

ANGLE_ACT_RANGES = {
    1: (1546,),  # Pinky
    2: (1548,),  # Ring Finger
    3: (1550,),  # Middle Finger
    4: (1552,),  # Index Finger
    5: (1554,),  # Thumb Flexion
    6: (1556,),  # Thumb Abduction
}

ANGLE_SET_RANGES = {
    1: (1486,),  # Pinky
    2: (1488,),  # Ring Finger
    3: (1490,),  # Middle Finger
    4: (1492,),  # Index Finger
    5: (1494,),  # Thumb Flexion
    6: (1496,),  # Thumb Abduction
}

FINGER_NAMES = {
    1: "Pinky",
    2: "Ring Finger",
    3: "Middle Finger",
    4: "Index Finger",
    5: "Thumb Flexion",
    6: "Thumb Abduction",
    7: "Palm",
}

class MultiSensor(Node):
    def __init__(self, config_filepath: str):
        super().__init__('multi_sensor')
        
        with open(config_filepath, 'r') as file:
            config = yaml.safe_load(file)

        self.MODBUS_IP = config.get('MODBUS_IP')
        self.MODBUS_PORT = config.get('MODBUS_PORT')

        self.force_publisher = self.create_publisher(GetForceAct1, 'force_data', 10)
        self.angle_publisher = self.create_publisher(GetAngleAct1, 'angle_data', 10)
        self.touch_publisher = self.create_publisher(GetTouchAct1, 'touch_data', 10)
        self.temp_publisher = self.create_publisher(GetTemp1, 'temp_data', 10)  
        
        self.create_subscription(SetAngle1, 'set_angle_data', self.angle_callback, 10)
        self.create_subscription(SetForce1, 'set_force_data', self.force_callback, 10)
        self.create_subscription(SetSpeed1, 'set_speed_data', self.speed_callback, 10)

        self.modbus_client = ModbusTcpClient(self.MODBUS_IP, port=self.MODBUS_PORT)
        if not self.modbus_client.connect():
            self.get_logger().error("Unable to connect to Modbus server, please check IP and port configuration")
            return

        self.read_thread = threading.Thread(target=self.data_reading_thread)
        self.read_thread.start()
        self.get_logger().info("Modbus connection successful")

    def speed_callback(self, msg):
        for finger_id, speed in zip(msg.finger_ids, msg.speeds):
            if 0 <= speed <= 1000:
                address = SPEED_SET_RANGES.get(finger_id, (None,))[0]
                if address is not None:
                    self.write_signed_register(address, speed)
                else:
                    self.get_logger().warn(f"Finger ID {finger_id} not found")

    def read_signed_register(self, address):
        response = self.modbus_client.read_holding_registers(address=address, count=1)
        if isinstance(response, ExceptionResponse) or response.isError():
            self.get_logger().error(f"Failed to read register {address}: {response}")
            return 0
        else:
            value = response.registers[0]
            if value > 32767:
                value -= 65536
            return value

    def read_register_range(self, start_addr, end_addr):
        register_values = []
        for addr in range(start_addr, end_addr + 1, 125 * 2):
            current_count = min(125, (end_addr - addr) // 2 + 1)
            response = self.modbus_client.read_holding_registers(address=addr, count=current_count)

            if isinstance(response, ExceptionResponse) or response.isError():
                self.get_logger().error(f"Failed to read register {addr}: {response}")
                register_values.extend([0] * current_count)
            else:
                register_values.extend(response.registers)

        return register_values

    def read_touch_data(self):
        touch_data = {}
        for finger_id, (start_addr, end_addr) in TOUCH_ACT_RANGES.items():
            touch_values = self.read_register_range(start_addr, end_addr)
            touch_data[finger_id] = []

            for i in range(0, len(touch_values), 2):
                if i < len(touch_values):
                    combined_value = int(touch_values[i])
                    touch_data[finger_id].append(combined_value)

            touch_data[finger_id] = [int(value) for value in touch_data[finger_id]]
            # self.get_logger().info(f"Finger ID: {finger_id}, Touch Values: {touch_data[finger_id]}")

        return touch_data

    def read_temperature_data(self):
        temp_data = {}
        for finger_id in FORCE_SENSOR_RANGES.keys():
            address = FORCE_SENSOR_RANGES[finger_id][0]  
            temp_value = self.read_signed_register(address)  
            
            low_byte_value = temp_value & 0xFF
            temp_data[finger_id] = low_byte_value
            # self.get_logger().info(f"Finger ID: {finger_id}, Temperature (Low Byte): {low_byte_value}")

        return temp_data

    def publish_data(self):
        
        if self.force_publisher.get_subscription_count() > 0:
            force_data_msg = GetForceAct1()
            force_data_msg.finger_ids = []
            force_data_msg.force_values = []
            force_data_msg.finger_names = []

            for finger_id, (start_addr,) in FORCE_SENSOR_RANGES.items():
                force_value = self.read_signed_register(start_addr)
                force_data_msg.finger_ids.append(finger_id)
                force_data_msg.force_values.append(force_value)
                force_data_msg.finger_names.append(FINGER_NAMES.get(finger_id, "Unknown Finger"))

            self.force_publisher.publish(force_data_msg)
            # self.get_logger().info(f"force data: {force_data_msg.force_values}, frequency: {1 / (time.time() - start_time):.2f} Hz")

        if self.angle_publisher.get_subscription_count() > 0:
            angle_data_msg = GetAngleAct1()
            angle_data_msg.finger_ids = []
            angle_data_msg.angles = []
            angle_data_msg.finger_names = []

            for finger_id, (start_addr,) in ANGLE_ACT_RANGES.items():
                angle_value = self.read_signed_register(start_addr)
                angle_data_msg.finger_ids.append(finger_id)
                angle_data_msg.angles.append(angle_value)
                angle_data_msg.finger_names.append(FINGER_NAMES.get(finger_id, "Unknown Finger"))

            self.angle_publisher.publish(angle_data_msg)
            # self.get_logger().info(f"angle data: {angle_data_msg.angles}, frequency: {1 / (time.time() - start_time):.2f} Hz")


        if self.touch_publisher.get_subscription_count() > 0:
            touch_data = self.read_touch_data()
            touch_data_msg = GetTouchAct1()
            touch_data_msg.finger_ids = list(touch_data.keys())
            touch_data_msg.touch_values = []
            touch_data_msg.finger_names = []

            for finger_id in touch_data_msg.finger_ids:
                touch_data_msg.touch_values.extend(touch_data[finger_id])
                touch_data_msg.finger_names.append(FINGER_NAMES.get(finger_id, "Unknown Finger"))

            self.touch_publisher.publish(touch_data_msg)
            # self.get_logger().info(f"touch_data: {touch_data_msg.touch_values}, frequency: {1 / (time.time() - start_time):.2f} Hz")
    
        if self.temp_publisher.get_subscription_count() > 0:
            temp_data_msg = GetTemp1()
            temp_data_msg.finger_ids = []
            temp_data_msg.temp_values = []
            temp_data_msg.finger_names = []

            temperature_data = self.read_temperature_data()  
            for finger_id, temp_value in temperature_data.items():
                temp_data_msg.finger_ids.append(finger_id)
                temp_data_msg.temp_values.append(temp_value)
                temp_data_msg.finger_names.append(FINGER_NAMES.get(finger_id, "Unknown Finger"))

            self.temp_publisher.publish(temp_data_msg)
            # self.get_logger().info(f"Temperature data published, reading frequency: {1 / (time.time() - start_time):.2f} Hz")

    def angle_callback(self, msg):
        for finger_id, angle in zip(msg.finger_ids, msg.angles):
            if 0 <= angle <= 1000:
                address = ANGLE_SET_RANGES.get(finger_id, (None,))[0]
                if address is not None:
                    self.write_signed_register(address, angle)
                else:
                    self.get_logger().warn(f"Finger ID {finger_id} not found")

    def force_callback(self, msg):
        for finger_id, force in zip(msg.finger_ids, msg.forces):
            if 0 <= force <= 3000:
                address = FORCE_SET_RANGES.get(finger_id, (None,))[0]
                if address is not None:
                    self.write_signed_register(address, force)
                else:
                    self.get_logger().warn(f"Finger ID {finger_id} not found")

    def speed_callback(self, msg):
        for finger_id, speed in zip(msg.finger_ids, msg.speeds):
            if 0 <= speed <= 1000:
                address = SPEED_SET_RANGES.get(finger_id, (None,))[0]
                if address is not None:
                    self.write_signed_register(address, speed)
                else:
                    self.get_logger().warn(f"Finger ID {finger_id} not found")

    def write_signed_register(self, address, value):
        if value < 0:
            value += 65536
        response = self.modbus_client.write_register(address, value)

        if isinstance(response, ExceptionResponse) or response.isError():
            self.get_logger().error(f"Failed to write register {address}: {response}")

    def data_reading_thread(self):
        while rclpy.ok():
            self.publish_data()
            time.sleep(0.1)

def main(args=None):
    print("Starting multi sensor node")
    rclpy.init(args=args)
    current_filepath = os.path.abspath(__file__)
    root = Path(current_filepath).parent.parent.parent.parent
    node = MultiSensor(str(root / "src" / "humanoid_hand" / "config.yaml"))
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

