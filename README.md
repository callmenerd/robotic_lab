# 🤖 Rafif's Embedded Robotics Projects

This repository contains a collection of my embedded systems projects, focused on **robotics** and **real-time microcontroller programming**. It includes:

- 🧠 Custom Arduino libraries for modular motor control, encoder feedback, IMU sensor filtering, etc.
- 🤖 Full project source codes for robots (e.g., X-Drive autonomous robot).
- 🧩 Real-time ROS node implementations for robotic control (ROS2 compatible).
- 🧪 Tools and examples for sensor calibration, filtering, and PID tuning.

---

## 📁 Repository Structure

This repository uses **branches** to organize related modules:

| Branch                      | Description                                                                |
|-----------------------------|----------------------------------------------------------------------------|
| `main`                      | Main branch with complete source codes for my final project                |
| `arduino-libs   `           | Custom-built Arduino libraries (e.g., MotorController, IMU)                |
| `arduino-project   `        | Source code of my Arduino project                                          |
| `image_processing          `| OpenCV based image processing project                                      |
| `ABU-robocon-24`            | (Optional) ROS/ROS2 packages and nodes                                     |
| `communication-protocol`    | Program for python file outside ROS ecosystem to communicate with ROS file |

---

## 🔧 Requirements

Depending on the project, you may need:

- **Arduino IDE** 2.0.x or newer
- **ROS2** (for plotting, testing, or ROS integration scripts)
- **ESP32**, **STM32**, **Raspberry Pi**, or other microcontrollers

---

## 🚀 Features & Highlights

- ✅ Modular DC motor controller with encoder feedback (Arduino-based)
- ✅ PID-based X-Drive motor speed control
- ✅ IMU sensor filtering with Madgwick filter (MPU6050, BMI160, BMX160, MPU9250)
- ✅ ROS2 node for :
  -> Communicating with other microcontroller using UART protocol
  -> Position control with dead reckoning navigation and PID
  -> Receiving data from non-ROS2 program (python program outside the ROS2 ecosystem)
  -> Early development of custom fusion algorithm using EKF to fuse wheel encoder, freewheel encoder and IMU

---

## 🧰 Technologies Used

- Languages: `C++`, `Python`
- Platforms: `Arduino IDE`, `ROS2`
- Hardware: `ESP32`, `STM32`, `Raspberry Pi`, `IMU`, `DC Motor`, `Encoder`

---

## 🧑‍💻 Author
Rafif Susena
Final-Year Electrical Engineering Student | Instrument and Control System specialization

Robotics, Embedded System & IoT Developer

---

## Contact Me
🔗 https://www.linkedin.com/in/rafif-susena/
📧 rafifsusena1@gmail.com

---

## 🙏 Acknowledgments
These projects were developed as part of:
- My Bachelor Final Project on Autonomous X-Drive Robot
- Contributions to University Robotics Team
