<h1>Humanoid Hand Control</h1>

This is tested with ubuntu 22.04 & ros2 humble.

Important file structure.

```plaintext
ros2_ws/
├── build/
├── install/
├── log/
├── config.yaml               # Workspace configuration
├── src/
└── humanoid_hand/            # Main package
    ├── config.yaml           # Package-specific configuration
    ├── dex_retargeting_wrapper/
    │   └── dex_retargeting_wrapper/
    │       └── dex_retargeting/
    │           └── assets/
    │               └── robots/
    │                   └── hands/
    │                       └── inspire_hand/    # Hand models
    │                           ├── meshes/      # 3D model assets
    │                           ├── inspire_hand_left.urdf
    │                           ├── inspire_hand_left_glb.urdf
    │                           ├── inspire_hand_right.urdf
    │                           ├── inspire_hand_right_glb.urdf
    │                           └── LICENSE.txt
    ├── humanoid_hand_controller/  # Control logic
    │   └── humanoid_hand_controller/
    │       ├── gestures.py        # Predefined hand gestures
    │       ├── hand.py            # Core hand control API
    │       ├── __init__.py        # Package initialization
    │       └── multi_sensor.py    # Sensor integration
    ├── README.md                 # Package documentation
    ├── requirements.txt          # Python dependencies
    ├── serial/                   # Serial communication utilities
    └── service_interfaces/       # ROS2 service definitions
```


<h2>Humanoid Hand Controller, hand_node</h2>
<ol>
    <li>install modbus package <b>sudo apt-get install libmodbus-dev</b>. However, what worked for me was <b>pip install pymodbus</b>.</li>
    <li> run <b>colcon build --symlink-install</b>.</li>
    <li> run <b> source install/setup.bash</b>.</li>
    <li> configure the network by going to <b> Settings > Network > Wired (Settings) > IPv4 > Manual > 192.168.11.222 (address) > 255.255.255.0 (netmask)</b>.</li>
    <li> run <b>ping 192.168.11.210</b> to check if the network is established. </li>
    <li> run <b>ros2 run humanoid_hand_controller multi_sensor_node</b> to read sensors and set positions.</li>
    <li> run <b>ros2 run humanoid_hand_controller hand_node</b> to control the hand.</li>
    <li> [Optional] run <b>ros2 run plotjuggler plotjuggler</b> to view plots.</li>
</ol>

<h2>Dex Retargeting Wrapper, realtime_retargeting_node</h2>
<ol>
    <li>from ros2_ws cd into src/humanoid_hand/dex_retargeting_wrapper/dex_retargeting_wrapper and git clone <a href "https://github.com/dexsuite/dex-retargeting.git"> dex retargeting package </a>. </li>
    <li>download the urdf files from <a "https://github.com/dexsuite/dex-urdf/tree/main/robots/hands">here</a> and place shown in the file structure.
    <li> make a copy of the urdf file and rename add _glb right before the extension. For example, inspire_hand_left.urdf -> inspire_hand_left_glb.urdf. 
    <li>install <a href="https://stack-of-tasks.github.io/pinocchio/download.html"> pinnochio </a> for teleoperation by following the steps in the link.</li>
    <li>run <b>ros2 run dex_retargeting_wrapper realtime_retargeting_node</b>. </li>
</ol>

<h2>Realsense2_camera, realsense_camera_node</h2>
<ol>
    <li><b>ros2 run realsense2_camera realsense2_camera_node</b></li>
</ol>

<h1>Example</h1>
![Demo](hand_teleop.gif)