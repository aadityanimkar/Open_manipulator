# Pick and Place Project with Open Manipulator-X


## Introduction
This project demonstrates a pick and place application using the Open Manipulator in a ROS environment. The goal is to move an object from one location to another using an Open Manipulator.

## Requirements
- **Hardware:**
  - Open Manipulator-X
  - Ubuntu 20.04
  - ROS Noetic

- **Software:**
  - ROS (Robot Operating System)
  - Python 3.x
  - Open Manipulator packages

## Installation

1. Clone the repository  
```bash
    git clone https://github.com/aadityanimkar/Open_manipulator.git
    cd ~/catkin_ws
    catkin_make

```
2. Install following dependencies
```bash
    - sudo apt-get install ros-noetic-ros-controllers ros-noetic-gazebo* ros-noetic-moveit* ros-noetic-industrial-core

    - sudo apt-get install ros-noetic-ros-controllers ros-noetic-gazebo* ros-noetic-moveit* ros-noetic-industrial-core

    - sudo apt install ros-noetic-robotis-manipulator
```

3. Navigate to workspace and **Source**

4. For Gazebo Simulation
    ```bash
    - roslaunch open_manipulator_controller open_manipulator_controller.launch use_platform:=false
    - roslaunch open_manipulastor_gazebo open_manipulator_gazebo.launch
    - rosrun tal multiple_movements.py
    ```
    - Video Demonstration
        [Download and Watch the Video](manipulatox_gazebo.mp4)
5. For actual hardware
    - Refer https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/ - set up dynamixel wizard and servos
    ```bash
    -  roslaunch open_manipulator_controller open_manipulator_controller.launch 
    - rosrun tal pick_place.py
    ```

### Reference
1. https://emanual.robotis.com/docs/en/platform/openmanipulator_x/quick_start_guide/#communication-interface
2. https://github.com/DougUOW/om_service_call_examples