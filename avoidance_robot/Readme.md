# Obstacle Avoidance Robot

Autonomous mobile robot prototype built during the robotics coursework at the Club de Robótica de la Facultad de Ingeniería (CROFI), UNAM. The platform implements reactive obstacle avoidance using discrete analog logic and infrared proximity sensors.

---

## Hardware Overview

- **Chassis**: Custom-machined baseplate with dual differential-drive DC gearmotors.
- **Sensing**: Infrared proximity sensors arranged for 180° frontal obstacle detection.
- **Actuation & Logic**: Hardware-based H-bridge motor driver with discrete logic switching (reactive direction control without microcontroller overhead).

---

## Prototype Media

![Obstacle Avoidance Robot Prototype](https://github.com/user-attachments/assets/a29bb5a2-5ea9-40a2-a7e1-3accfdd24146)

---

## Subdirectories

- [`code/`](code/): Firmware and navigation algorithms for the ultrasonic/photodetector labyrinth robot variant (ATmega328P).
- [`Electronics/`](Electronics/): Autodesk EAGLE schematics, board layout, and documentation for the standalone circular ATmega328P PCB.

---

## Author & Attribution

- **Diego Méndez Carter** (Chassis assembly, discrete hardware soldering, testing)
- Course instruction and curriculum by CROFI (UNAM)

---

## License

Shared under the [MIT License](LICENSE) for educational and portfolio reference.
