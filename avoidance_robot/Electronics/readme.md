# Standalone ATmega328P Mobile Robot PCB

Custom circular printed circuit board developed for autonomous differential-drive mobile robots. Built around a standalone ATmega328P microcontroller (Arduino UNO architecture), integrating motor drive stages, power regulation, and perimeter sensor headers.

---

## PCB Layout & Schematic

![Standalone ATmega328P Circular PCB Layout](https://github.com/user-attachments/assets/49b72e0a-d0d1-4986-bc8e-2942716710a4)

---

## Hardware Specifications

| Subsystem | Specification |
| :--- | :--- |
| **Microcontroller** | ATmega328P-PU (DIP-28) with 16 MHz external crystal oscillator |
| **Motor Drive** | Dual-channel H-bridge stage (L293D) for bidirectional DC motor control |
| **Power Regulation** | Onboard linear 5V regulation (LM7805 / LDO) with bulk and decoupling capacitors |
| **I/O Distribution** | Perimeter headers for ultrasonic transducers, analog photodetectors, and UART |
| **Form Factor** | Circular layout tailored to match differential-drive robot chassis boundaries |

---

## Circuit Architecture

- **Power Filtering**: Independent decoupling capacitors across the ATmega328P VCC/AVCC pins and the motor supply rail to suppress inductive switching transients.
- **Trace Routing**: Sized power and ground traces to accommodate motor stall currents without ground bounce.
- **Peripheral Expansion**: Dedicated headers for analog inputs (A0–A5), digital control lines, and ISP programming.

---

## CAD Tools

- Designed and routed in **Autodesk EAGLE**.
- Design files located in the repository root / electronics archive.

---

## Attribution & License

- Circuit design and teaching curriculum: Adrián Ricárdez Ortigosa (CROFI UNAM).
- Layout implementation and assembly: Diego Méndez Carter.
- Shared under the [MIT License](LICENSE).
