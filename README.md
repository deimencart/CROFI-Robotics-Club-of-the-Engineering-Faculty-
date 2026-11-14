# CROFI Robotics Club — Project Archive

Repository archive for robotics platforms, embedded firmware, custom motor driver PCBs, and ROS integration projects developed at the Club de Robótica de la Facultad de Ingeniería (CROFI), UNAM (2018–2022).

---

## Projects Directory

| Directory | Platform / Project | Core Technologies | Description |
| :--- | :--- | :--- | :--- |
| [`avoidance_robot/`](avoidance_robot/) | Reactive Obstacle Avoidance | ATmega328P, Discrete Logic, H-Bridge | Autonomous reactive mobile platform and circular standalone PCB design. |
| [`Beach_Cleaner_Prototype/`](Beach_Cleaner_Prototype/) | 4WD Beach Cleaner Rover | ROS Noetic, Raspberry Pi, Arduino | All-terrain 4WD skid-steer chassis, teleoperation nodes, and motor actuation. |
| [`Prot_Line_Follower-main/`](Prot_Line_Follower-main/) | PID Line Follower & Shield | Arduino Nano, L293D, EAGLE PCB | Closed-loop PID line follower robot and custom motor driver shield PCB. |
| [`Various_Robots/`](Various_Robots/) | Minisumo & Competition Robots | Discrete Drivers, ROS, 3D Printing | Competition prototypes: discrete minisumo and early ROS Soccer Junior platform. |
| [`Repo_RBP_Config_MATE-main/`](Repo_RBP_Config_MATE-main/) | Raspberry Pi & ROS Environment | Ubuntu MATE, ROS Melodic, I2C | System configuration guides, I2C bus communication notes, and ROS setup on RPi 4. |

---

## Technical Scope

- **Embedded Systems**: Standalone ATmega328P, Arduino Nano/UNO, Raspberry Pi 4.
- **Actuation & Drive**: PWM speed control, H-bridges (L293D, L298N, Roboclaw), DC gearmotors with optical encoders.
- **Control & Middleware**: Reactive sensor logic, PID closed-loop control, ROS (Melodic / Noetic) teleoperation nodes and topics.
- **Hardware & PCB Design**: Custom schematics and layouts developed in Autodesk EAGLE.
- **Sensing**: Infrared reflectance arrays (QTR-8A), analog photodetectors, ultrasonic distance sensors.

---

## Author

**Diego Méndez Carter**  
Mechatronics Engineer · M.Eng. Robotics Candidate (UNIZAR)  
Portfolio: [deimencart.github.io](https://deimencart.github.io)

---

## License

All software, schematics, and documentation in this repository are available under the [MIT License](LICENSE), unless otherwise specified in subdirectories.
