# 🏖️ Beach-Cleaning Robot – Autonomous Field Robotics Prototype (Pandemic Project)

> This project documents the design and implementation of a **robotic platform for autonomous beach cleaning**, developed independently during the COVID-19 pandemic as a way to explore **robotics, perception, and system integration**.  
> It became my first in-depth experience with **ROS Noetic**, **OpenCV**, and managing a **Raspberry Pi as a robot brain**.

---

## 🧠 Context & Skills Learned

This was my **pandemic robotics project** — a way to stay sharp, build something meaningful, and dive into real robotic systems from home. I used it as a foundation to learn:

- 🧩 **ROS Nodes** and the architecture of modular robotic control
- 🎮 **Teleoperation via ROS topics**
- 🐍 Integration of **OpenCV** for camera streams and future object detection
- 🍓 Configuration of a **Raspberry Pi for ROS Noetic**, SSH, and serial communication
- 🔧 Real-time sensor fusion (ultrasonic + encoder) for motion control

---

## 📸 Project Gallery

### 🛠️ CAD Model – 3D Preview of Design Concept
![image](https://github.com/user-attachments/assets/98f5e689-b0a0-4cce-a2d5-ae2d03693953)

### 🔩 First Chassis Fabrication – Machined Aluminum Frame

![image](https://github.com/user-attachments/assets/26212198-87a6-425c-8a33-ee726ae3f23b)

### 🧱 First Mechanical Assembly – Drive Base + Wheels

![image](https://github.com/user-attachments/assets/41f97150-42cc-4f3f-85af-a940553f7b2c)

### ⚡ Custom PCB for Sensor & Motor Connections

![image](https://github.com/user-attachments/assets/04e84026-ec9a-4035-a889-5671dcd3a38d)

### 🔌 Wiring & Electronics – Power and Motor Controllers

![image](https://github.com/user-attachments/assets/36c6be74-ac10-4870-8fa1-81ee2135a224)

### 🚜 Final Prototype – Fully Mounted System
![image](https://github.com/user-attachments/assets/e71b539a-7c6d-4bd1-b830-17b458212895)

---

## 🔧 System Overview

- **Drive System**: 4-wheel differential drive with large tires for sandy terrain
- **Structure**: Machined aluminum base with 3D-printed mounts
- **Blade**: Passive front scoop for collecting plastic/waste
- **Electronics**:
  - Arduino Nano for motor/sensor control
  - Raspberry Pi running **ROS Noetic**
  - Dual H-bridge motor drivers
  - Ultrasonic sensors and battery regulators
- **Software**:
  - ROS packages for **motor commands**, **sensor reading**, and **teleoperation**
  - Custom launch files and shell scripts for remote access

---

## 📁 What’s Left

Due to data loss, most of the original code and CAD files were lost. This repo preserves:

- 📸 Visual progress documentation
- 🧠 Key learnings from the ROS ecosystem
- 📝 Summary of the robot’s conceptual and technical structure

---

## 🙋‍♂️ My Role

- 🔧 CAD modeling and mechanical fabrication
- 💻 PCB design and electronics wiring
- 🍓










