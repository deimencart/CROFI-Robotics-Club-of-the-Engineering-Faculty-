# Motor Driver Shield — PCB Design & Hardware Reference

Custom printed circuit board layout designed in Autodesk EAGLE for mobile robotics applications, integrating dual DC motor control and modular socket mounting for an Arduino Nano.

---

## Schematic

![Motor Driver Shield Schematic](https://github.com/user-attachments/assets/038c8d2a-8c9b-424b-af76-1df918dd50ae)

### Key Circuit Stages

- **Microcontroller Socket**: 30-pin DIP layout matching Arduino Nano pinout.
- **Motor Driver**: L293D dual H-bridge with dedicated logic (VCC1) and motor (VCC2) power rails.
- **Power Conditioning**: 5V linear regulator with 100 µF and 220 µF electrolytic filter capacitors and reverse-polarity protection diode.
- **User Interface**: 2 tactile pushbuttons with pull-down resistors; power and motor status LEDs.
- **Sensor Breakout**: Direct header access to analog lines A0–A5 and digital lines.

---

## PCB Layout

![PCB Layout](https://github.com/user-attachments/assets/09b4515d-f9eb-4127-974b-038ef3d00909)

### Routing & Geometry Notes

- Form factor shaped to interface directly with differential mobile robot chassis mounting holes.
- Sized power traces for high-current motor transients to prevent voltage drops on the logic rail.
- High-frequency decoupling capacitors placed in close proximity to the L293D supply pins.

---

## Pin Mapping Summary

| Arduino Nano Pin | Connected Function | Hardware Destination |
| :--- | :--- | :--- |
| **D2, D13** | Left Motor Direction | L293D Input 1 & 2 |
| **D7, D12** | Right Motor Direction | L293D Input 3 & 4 |
| **D11** | Left Motor Speed | L293D Enable 1 (PWM) |
| **D5** | Right Motor Speed | L293D Enable 2 (PWM) |
| **D9, D10** | Pushbutton Triggers | Tactile switches with pull-down resistors |
| **A0–A5** | IR Sensor Inputs | Front sensor array header |
| **VIN** | Motor Power Input | 6V–12V Battery Pack |
| **+5V / GND** | Logic Supply & Ground | Regulated power plane |

---

## Electrical Specifications

- **Input Voltage Range**: 6V to 12V DC
- **Logic Level**: 5V regulated
- **Motor Current Output**: Up to 600 mA continuous per channel (1.2A peak)
- **Protection**: Series Schottky reverse-voltage protection diode

---

## License

MIT License.
