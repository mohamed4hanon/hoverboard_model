# 🫁 Respiratory-Controlled Hoverboard (ROS2 Jazzy)

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

### 2. Running the Driver (The Brain)
Run your main control script:
```bash
python3 brain_driver.py
