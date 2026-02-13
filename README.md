# Controlling a hoverboard_model (ROS2 Jazzy)

A unique robotics project that interfaces a **Hoverboard** with **ROS2 Jazzy** using **respiratory sensors** (breath control). The movement is visualized in **RViz2** with a medium-sized 3D model and real-time wheel rotation.

## 🛠 Features
*   **Respiratory Drive:** Controls hoverboard speed and direction via breath/respiratory input.
*   **FTDI Serial Bridge:** Uses UART communication to send commands to hacked hoverboard firmware.
*   **Real-time Visualization:** 3D URDF model in RViz2 that follows the actual hoverboard movement.
*   **Odometry Tracking:** Calculates and broadcasts the robot's position (`odom` -> `base_link`).

## 📁 Project Structure
*   `brain_driver.py`: The main ROS2 node handling Serial (FTDI) and Odometry logic.
*   `hoverboard_model.urdf`: 3D model definition for the body and wheels.
*   `display.launch.py`: Launch file to start the State Publisher and RViz2 with presets.
*   `hoverboard.rviz`: Saved configuration for instant setup in RViz.

## 🚀 Getting Started

### 1. Hardware Connection
*   Connect the **FTDI** adapter to your computer.
*   Ensure the Hoverboard UART pins (TX/RX) are cross-connected to the FTDI.
*   **Port:** `/dev/ttyUSB0` | **Baud:** `115200`

### 2.  Getting the Driver (The Brain)
Run your main control script:change this file path to your own hoverboard serial python file :
#### A- git it from this link :https://github.com/mohamed4hanon/smart_hover 
- Download it via 
```bash
    cd ~/ros2_ws/src
    git clone https://github.com/mohamed4hanon/smart_hover.git  
```
- colcon build it via 
```bash
    cd ~/ros2_ws
    colcon build --packages-select smart_hover
    source install/setup.bash 
```

#### B- RUN it :
```bash
   ros2 run smart_hover hover_node
```
### Note 🌹 you can lear more in the link above ☝️


### 3. Launching Visualization (The Body)
In a new terminal, navigate to your workspace and launch the URDF model:
```bash
ros2 launch launch/display.launch.py
```
Use code with caution.

### ⚙️ Configuration
#### Logic Level:
            Ensure 3.3V logic for the FTDI to protect the STM32 mainboard.
#### Safety: 
           The driver includes a 0.5s timeout; if no breath command is received, the hoverboard stops automatically.
#### Wheel Rotation:
           URDF uses **axis xyz="0 0 -1"** to ensure wheels spin forward during movement.
#### 📸 Visualization Setup
In RViz2, ensure the following are added:
- Fixed Frame: odom
- RobotModel: Topic /robot_description
- TF: To see coordinate transforms.
- Odometry: To see the movement path.



