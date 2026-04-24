# Autonomous Mobile Manipulator for Warehouse Operations


## Team Members

| Name              | Student ID | Email            |
|-------------------|------------|------------------|
| Erebus Oh         | 117238105  | eoh12@umd.edu    |
| Ronen Gai Aniti   | 112095116  | raniti@umd.edu   |
| Vaibhav Shende    | 121206817  | svaibhav@umd.edu |

---


## Project Overview

The **USENAV Project** is a ROS 2(humble)-based mobile manipulator robot system to manage warehouse.  
It includes packages for:
- Robot description (URDF/Xacro)
- Simulation in Gazebo  
- Visualization in RViz  
- Control and teleoperation nodes


## Dependencies

### System Requirements
- **Ubuntu 22.04 LTS**
- **ROS 2 Humble Hawksbill**
- **Gazebo Ignition** 
- **colcon** build system


### Install Dependencies
```bash
sudo apt install -y ros-humble-xacro \
                 ros-humble-rviz2 \
                 ros-humble-gazebo-ros \
                 ros-humble-gazebo-ros2-control \
                 ros-humble-ros-gz-bridge \
                 ros-humble-robot-state-publisher \
                 ros-humble-joint-state-publisher \
                 ros-humble-joint-state-publisher-gui \
                 ros-humble-controller-manager \
                 ros-humble-joint-state-broadcaster \
                 ros-humble-position-controllers \
                 ros-humble-velocity-controllers \
                 ros-humble-ros2-control \
                 ros-humble-ros2-controllers \
                 ros-humble-teleop-twist-keyboard

sudo apt install python3-pip -y
pip3 install numpy 
```

## Build Instructions

1. Navigate to your ROS 2 workspace:
```bash
mkdir -p ~/usenav_ws/src 
cd ~/usenav_ws
```
### TODO: github-link update.
2. Clone this repository (if not already cloned):
```bash
git clone git@github.com:ronen-aniti-projects/Project2_Modeling_Erova.git ./src/usenav_project
```

3. ROSDEP Dependencies
```bash
cd ~/usenav_ws
rosdep install --from-paths src -r -y --ignore-src
```

4. Build all packages:
```bash
cd ~/usenav_ws
colcon build --symlink-install
```

5. Source the workspace: 
```bash
cd ~/usenav_ws
source install/setup.bash
```


## Run the Packages

Note: **In each terminal,** before running any nodes or launch files,  **source the workspace**:

```bash
cd ~/usenav_ws
source install/setup.bash
```

### Gazebo Simulation
```bash
# terminal: 1
ros2 launch usenav_description usenav_bringup.launch.py
```

```bash
# terminal: 2
ros2 run usenav_controller simple_controller
```

## Simulation Video link

https://drive.google.com/file/d/176yuGCCNJ2mBm8Qx2VDOYzwvZ2DswPFT/view?usp=sharing

https://drive.google.com/file/d/1qM6gke5bFs-uorRhOewMAFjHjF1z0pxf/view?usp=sharing.

## Project Structure
```
.
├── README.md
├── usenav_controller
│   ├── LICENSE
│   ├── package.xml
│   ├── resource
│   │   └── usenav_controller
│   ├── setup.cfg
│   ├── setup.py
│   ├── test
│   └── usenav_controller
│       ├── feedback_controller.py
│       ├── __init__.py
│       └── simple_controller.py
└── usenav_description
    ├── CMakeLists.txt
    ├── config
    │   ├── bridge_param.yaml
    │   ├── control.yaml
    │   └── display.rviz
    ├── launch
    │   ├── arm_gz.launch.py
    │   ├── arm_rviz.launch.xml
    │   ├── mobile_gz.launch.py
    │   ├── mobile_rviz.launch.xml
    │   ├── usenav_bringup.launch.py
    │   ├── usenav_gz.launch.py
    │   └── usenav_rviz.launch.py
    ├── LICENSE
    ├── package.xml
    ├── urdf
    │   ├── arm
    │   │   ├── arm_gz.xacro
    │   │   ├── arm.xacro
    │   │   └── standalone_arm.xacro
    │   ├── common_properties.xacro
    │   ├── mobile
    │   │   ├── mobile_gz.xacro
    │   │   ├── mobile_ros2control.xacro
    │   │   ├── mobile.xacro
    │   │   └── standalone_mobile.xacro
    │   └── usenav.urdf.xacro
    └── world
        └── final_wh2.world.sdf
```