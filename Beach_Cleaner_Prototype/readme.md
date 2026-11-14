# Beach Cleaner Mobile Platform Prototype

Mobile robotics prototype developed for granular terrain locomotion and waste collection. Integrates an all-wheel differential drive chassis, onboard companion computer running ROS Noetic, and embedded microcontroller actuation.

---

## Hardware Architecture

| Subsystem | Components & Specifications |
| :--- | :--- |
| **Locomotion** | 4-wheel skid-steer differential drive with high-traction terrain tires |
| **Chassis** | CNC/machined aluminum baseplates with modular 3D-printed mounting brackets |
| **Compute** | Raspberry Pi running Ubuntu with ROS Noetic (high-level control and teleoperation) |
| **Embedded Control** | Arduino Nano (PWM motor signal generation, ultrasonic polling, serial bridge) |
| **Motor Drivers** | Dual high-current H-bridges (L298N / Roboclaw compatible) |
| **Waste Intake** | Passive front-mounted collection scoop for surface debris and plastics |

---

## Prototype Assembly & Gallery

### Mechanical Construction
| CAD Model | Machined Aluminum Frame |
| :---: | :---: |
| ![Beach Cleaner CAD Concept](https://github.com/user-attachments/assets/98f5e689-b0a0-4cce-a2d5-ae2d03693953) | ![Machined Aluminum Frame](https://github.com/user-attachments/assets/26212198-87a6-425c-8a33-ee726ae3f23b) |

| Drive Assembly | Fully Mounted System |
| :---: | :---: |
| ![Drive Base Assembly](https://github.com/user-attachments/assets/41f97150-42cc-4f3f-85af-a940553f7b2c) | ![Assembled Beach Cleaner Prototype](https://github.com/user-attachments/assets/e71b539a-7c6d-4bd1-b830-17b458212895) |

### Electronics & Interfacing
| Interface PCB | Wiring & Power Assembly |
| :---: | :---: |
| ![Sensor and Motor Interface Board](https://github.com/user-attachments/assets/04e84026-ec9a-4035-a889-5671dcd3a38d) | ![Electronics Deck and Motor Wiring](https://github.com/user-attachments/assets/36c6be74-ac10-4870-8fa1-81ee2135a224) |

---

## Software Integration (ROS Noetic)

The control stack runs under ROS Noetic:
- **Teleoperation**: Subscribes to `/cmd_vel` (`geometry_msgs/Twist`) via keyboard or gamepad input.
- **Serial Interface**: Translates Twist linear and angular velocity commands into differential PWM signals transmitted over USB serial to the Arduino Nano.
- **Node Archive**: Located in [`Ros_NODES/`](Ros_NODES/).

---

## Author & License

- **Diego Méndez Carter** (Mechanical CAD, chassis machining, electronics wiring, ROS nodes)
- Licensed under the [MIT License](LICENSE).
