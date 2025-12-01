# 🔧 Custom Standalone Arduino UNO PCB – Robotics Electronics Project

> This project documents a custom-designed printed circuit board created during an electronics and robotics course. The board is centered around a **standalone ATmega328P microcontroller**, replicating the core of an Arduino UNO, and designed to power and control a small mobile robot platform.

---

## 🖼️ PCB Layout Studied at Course (Designed by CROFI)

![image](https://github.com/user-attachments/assets/49b72e0a-d0d1-4986-bc8e-2942716710a4) 


---

## 📐 Design Overview

- 🧠 **Microcontroller**: Standalone **ATmega328P** (Arduino UNO core)
- 🔋 **Power Supply**: Integrated linear regulator and capacitor filtering
- 📦 **Inputs/Outputs**: Pluggable headers for sensors, motors, and power
- 🧲 **Peripheral Support**: Labeled analog/digital pads, motor drivers, LEDs
- 🛠️ **Board Shape**: Circular design optimized for differential robot chassis

---

## 🧠 My Contributions
- Used design from CROFI - Adrian Ricardez 
- Implemented **standalone ATmega328P** setup (no Arduino board used)
- Assigned proper footprints and trace widths for high-current paths
- Planned component placement for mechanical alignment and routing efficiency
- Labeled and structured I/O headers for ease of debugging and expansion

---

## 💡 Features

| Feature              | Description                                 |
|----------------------|---------------------------------------------|
| MCU Core             | ATmega328P (Arduino UNO bootloader)         |
| Motor Control        | Dual-channel motor driver outputs           |
| Sensor Headers       | Labeled headers around the perimeter        |
| Battery Integration  | Battery bay and voltage regulation onboard  |
| Switches & LEDs      | Power switch, status LEDs, test pads        |

---

## 📘 What I Learned

- How to convert a breadboard Arduino setup into a real PCB
- Designing power rails and decoupling for motor + logic circuits
- How to lay out tracks for high-current devices (motors, drivers)
- Component spacing, clearance, and routing in circular boards

---

## 🧰 Tools Used

- 🖥️ Autodesk Eagle (schematic & layout)
- 📏 Calipers and physical prototyping for fit testing
- 📦 BOM management and passive selection

---

## 📸 Use Case

This PCB was intended to drive:
- A differential robot with 2 DC motors
- Obstacle detection sensors (analog or ultrasonic)
- Basic state-indicating LEDs
- External expansion headers (UART, SPI, ADC, GPIO)

---


## 👤 Mentor

**Adrían Ricardez **  
Robotics Engineer Tutor 

## 👤 Author

**Diego Méndez Carter**  


---

## 📜 License

All assets and documentation are shared for educational and portfolio use. For reuse or manufacturing, please contact the author.





