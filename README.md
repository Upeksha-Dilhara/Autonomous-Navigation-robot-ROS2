# Autonomous-Navigation-robot-ROS2
Autonomous Navigation robot ROS2 - Teleoperation, Mapping, Navigation, Auto docking for charging

# Fully Autonomous Navigation Robot

This repository contains the details of our project aimed at developing a fully autonomous navigation robot. The system was implemented and tested using a small R&D robot, leveraging the ROS2 framework for hardware-software interfacing and Python 3 for programming. The project features were simulated in the Gazebo environment and later tested in real-world scenarios.

## Key Features

- **Teleoperation**: Control the robot using a joystick controller.
- **Indoor Mapping**: Create maps of indoor environments using SLAM.
- **Autonomous Navigation**: Navigate using prebuilt maps.
- **Autonomous Docking**: Dock to a charging station automatically when the battery is low.

## Hardware and Software

### Hardware
- **Central Processor**: Raspberry Pi 4 Model B
  - Quad-core ARM Cortex-A72 CPU
  - 4GB RAM
  - 64GB micro-SD card
  - Runs Ubuntu 22.04 and ROS2 Humble

- **Sensors**:
  - **RP Lidar**: 2D mapping for indoor environments.
  - **BNO055 IMU Sensor**: Provides orientation and movement data via I2C communication.

- **Motion Control**:
  - Gear motors with encoders for odometry data.
  - RoboClaw motor controller for smooth movement and precise speed control (configured using Basicmicro Motion Studio).

- **Docking System**:
  - **IR Transmitters and Receivers**: Align the robot with the charging station.
  - **ToF Sensors**: Ensure accurate docking and charging terminal alignment.

### Software
- **ROS2 Humble**: Acts as middleware for hardware-software communication.
- **Simulation**: Gazebo environment for testing and validation.
- **SLAM and Navigation**:
  - `slam_toolbox`: Used for creating maps.
  - `nav2_bringup`: Enables autonomous navigation.
- **Extended Kalman Filter (EKF)**: Fuses odometry data from IMU and wheel encoders for accurate position and orientation estimation.

## Results and Implementation 
- **Indoor Mapping**: A map created by the robot in the Arimac head office.
  ![map_upeksha](https://github.com/user-attachments/assets/7bc7529f-2f96-4d81-9a1b-daa78961253c)

- **Dokcing Implementation**: Photos of the robot and its components during development.

Docking station
![WhatsApp Image 2024-05-30 at 04 20 36_0e787abc](https://github.com/user-attachments/assets/a6273293-603c-4876-9570-ee9905012ac9) 

Robot RI receivers and TOF sensors 
![WhatsApp Image 2024-05-30 at 04 20 37_cf06010c](https://github.com/user-attachments/assets/01e27875-7d1e-4d1e-8a77-51776735270c)

![WhatsApp Image 2024-05-30 at 04 20 38_85da2a3c](https://github.com/user-attachments/assets/95d4628f-7d1b-4c52-8cc0-227b3604e631)



## Repository Structure
```
├── src/
│   ├── robot_control
│   ├── mapping_and_navigation
│   ├── docking_system
├── simulations/
│   ├── gazebo_worlds
│   ├── test_cases
├── docs/
│   ├── hardware_specs.md
│   ├── software_packages.md
├── README.md
```

## How to Run
1. Clone the repository:
   ```bash
   git clone https://github.com/your-repo-url.git
   cd your-repo-url
   ```
2. Install dependencies:
   ```bash
   sudo apt update && sudo apt install -y ros-humble-desktop
   ```
3. Launch the Gazebo simulation:
   ```bash
   ros2 launch simulations gazebo_world.launch.py
   ```
4. Run SLAM:
   ```bash
   ros2 launch slam_toolbox online_sync.launch.py
   ```
5. Start autonomous navigation:
   ```bash
   ros2 launch nav2_bringup navigation_launch.py
   ```


---
Feel free to contribute by opening issues or submitting pull requests!
