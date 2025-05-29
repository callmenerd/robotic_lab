# 🤖 Rafif's Embedded Robotics Projects

This repository contains a collection of my embedded systems projects, focused on **robotics** and **real-time microcontroller programming**. It includes:

- 🧠 Custom Arduino libraries for modular motor control, encoder feedback, IMU sensor filtering, etc.
- 🤖 Open to public project source codes for robots (e.g., X-Drive autonomous robot).
- 🧩 Real-time ROS node implementations for robotic control (ROS2 compatible).
- 🧪 Tools and examples for sensor calibration, filtering, and PID tuning.

---

## 📁 Repository Structure

This repository uses **branches** to organize related modules:

| Branch                      | Description                                                                               |
|-----------------------------|-------------------------------------------------------------------------------------------|
| `main`                      | Main branch with open to public only source codes for my final project                    |
| `arduino-libs   `           | Custom-built Arduino libraries (e.g., MotorController, IMU)                               |
| `image_processing          `| OpenCV based image processing project, publish image from file outside the ros ecosystem  |
| `ABU-robocon-24`            | Source code for Indonesia's ABU Robot Competition 2024                                    |
| `communication-protocol`    | Source code for multiple python file to communicate with each other                       |

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
- Hardware: `STM32`, `Raspberry Pi`, `IMU`, `DC Motor`, `Encoder`

---

## 🎬 Sneak Peak Documentation Video

https://github.com/user-attachments/assets/927a0f6e-2085-41fd-a10a-1c49939a6dd1

---

## 🧑‍💻 Author
Rafif Susena

Graduated from Electrical Engineering Bachelor Program | Instrument and Control System specialization

Focus on Robotics, Embedded System (Microcontroller Based) & IoT Developer

---

## 📞 Contact Me
🔗 https://www.linkedin.com/in/rafif-susena/
📧 rafifsusena1@gmail.com

---

## 🙏 Acknowledgments
These projects were developed as part of:
- My Bachelor Final Project on Autonomous X-Drive Robot
- Contributions to University Robotics Team
