# Arduino Nano Motor Driver Shield & Line Follower

Compact printed circuit board shield designed for Arduino Nano mobile platforms, providing dual DC motor drive (L293D), integrated 5V voltage regulation, pushbutton interfaces, and status LEDs.

![Motor Driver Shield](https://github.com/user-attachments/assets/23c8fef1-43e2-41d1-98cb-173361cd1247)

---

## Hardware Specifications

| Subsystem | Specification |
| :--- | :--- |
| **Microcontroller Socket** | Arduino Nano (modular 30-pin DIP footprint) |
| **Motor Driver** | L293D dual H-bridge (up to 600 mA continuous per channel) |
| **Input Supply** | 6V–12V DC input via screw terminals / DC jack |
| **Logic Regulation** | Onboard 5V linear regulator with decoupling capacitors (100 µF, 220 µF) |
| **User Interfaces** | 2 tactile pushbuttons, power indicator LED, activity LED |
| **EDA Tool** | Autodesk EAGLE |

---

## Prototype Testing & Media

### Track Evaluation & Bench Assembly
| Autonomous Track Run | Prototype Assembly |
| :---: | :---: |
| ![Line Follower Track Run](media/line_follower_run.gif) | ![Prototype Assembly](media/line_follower_prototype_assembly.jpg) |

### CAD & Circuit Design
| Schematic Diagram | PCB Layout |
| :---: | :---: |
| ![Schematic View](https://github.com/user-attachments/assets/10e82daf-3f52-4ddd-8b8c-d3cd1b74484b) | ![PCB Layout](https://github.com/user-attachments/assets/0209fffe-2345-4404-90be-3449e2af5d79) |

---

## Repository Structure

- [`code/`](code/): Closed-loop PID tracking firmware for 6-channel analog IR reflectance sensor arrays.
- [`electronics/`](electronics/): Autodesk EAGLE schematic (`.sch`), board layout (`.brd`), and hardware pinout documentation.

---

## Author & License

- **Diego Méndez Carter**  
- Licensed under the [MIT License](LICENSE).
