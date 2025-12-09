# Automated AMR/AGV Safety Test System (ISO-3691-4)

![Python](https://img.shields.io/badge/Python-3.11-blue) ![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%204-red) ![OpenCV](https://img.shields.io/badge/Lib-OpenCV-green)

## 📖 Project Overview
This capstone project is an automated test rig designed to validate the safety and performance of Autonomous Mobile Robots (AMRs) and Automated Guided Vehicles (AGVs). 

The system automates critical test scenarios required by safety standards like **ISO-3691-4**, focusing on braking performance, speed limits, and path tracking accuracy. It utilizes Computer Vision (AruCo markers) for tracking and Time-of-Flight (ToF) sensors for precise distance measurement, sending all test results to a remote IoT server via HTTP.

## ⚙️ Hardware Architecture
The system is built on a **Raspberry Pi 4 (4GB)** using the following peripherals:

| Component | Function | Interface |
|-----------|----------|-----------|
| **USB Web Cam** | Visual Odometry & Tracking | USB |
| **VL53L0X ToF** | Distance measurement (mm precision) | I2C (0x29) |
| **Servo Motor** | Simulates dynamic obstacles (Boom Gate) | GPIO (PWM) |
| **16x2 LCD** | User Interface & Status Display | I2C (0x27) |
| **Push Buttons** | Test Mode Selection | GPIO |

### System Sketch
![System Concept] ['Screenshot 2025-12-09 233341.png'](https://github.com/darkshot25/Capstone109/blob/689a179ed57650c7e1dcd264cd9064e7a84f62c6/Screenshot%202025-12-09%20233341.png)
*(Note: Replace `IMG_20251108_010734.jpg` with the actual path to your uploaded image in the repo)*

## 🧪 Test Scenarios implemented

### 1. Speed Validation Test
* **Method:** Tracks an AruCo marker on the robot. Calculates pixel displacement over time converted to meters.
* **Criteria:** Pass if speed is between `1.0 m/s` and `2.0 m/s`.

### 2. Path Accuracy Test
* **Method:** Compares the vector of the Robot's AruCo marker against a "Reference" marker on the floor.
* **Criteria:** Pass if the deviation angle is **< 15 degrees**.

### 3. Emergency Brake Test (Dynamic Obstacle)
* **Method:** A servo motor introduces an obstacle (boom gate) at a random interval. The ToF sensor monitors the robot's distance.
* **Logic:** Calculates the time delta between obstacle deployment and distance stabilizing (robot stopped).
* **Criteria:** Pass if stop time is **< 5 seconds**.

### 4. Zone Braking Test
* **Method:** Uses spatial logic with three AruCo markers: Robot, Zone S1 (Start), and Zone S2 (End).
* **Criteria:** * **Pass:** Robot stops *between* S1 and S2.
    * **Fail:** Stops before S1 (Over-brake) or after S2 (Under-brake).

## 🚀 Installation & Setup

This project uses **Python 3.11**, which requires a Virtual Environment (`venv`) on Raspberry Pi OS.

### 1. System Dependencies
Install the required system libraries for OpenCV and I2C:
```bash
sudo apt-get update
sudo apt-get install -y libatlas-base-dev libhdf5-dev libopenjp2-7 i2c-tools python3-dev
