# 🛠️ Arduino Nano Motor Driver Shield (Custom PCB)

This project is a custom-designed shield for the **Arduino Nano**, tailored for compact robotic systems. The PCB integrates:
- Dual motor control via an L293D H-Bridge
- Button interface
- Power regulation circuitry
- Status LEDs
- A shape designed for easy integration into mobile platforms

---

## 🖥️ PCB Schematic
![image](https://github.com/user-attachments/assets/9dabec41-9018-40b3-8d3d-0a6489e4e8fd)
### 🔧 Main Features

- **Microcontroller**: Arduino Nano (modular socket)
- **Motor Driver**: L293D for 2-channel DC motor control
- **Power Supply**: Includes 5V regulator, capacitors for filtering
- **Buttons**: 2 push buttons with pull-down resistors
- **LED Indicators**: Power, logic, motor activity
- **Connectivity**: Header pins for I/O, power, and sensors

---

## 🖲️ PCB Layout

![image](https://github.com/user-attachments/assets/0b3555f5-5a2d-417e-b5bc-47653441aa87)


### ⚙️ Design Highlights

- **Form Factor**: Shaped to fit a robotic chassis; motor brackets visible
- **Compact Routing**: Traces efficiently routed to minimize space
- **Component Placement**:
  - Motors on each side
  - Arduino socket in the middle
  - Power regulator and capacitors at the lower center
  - LED indicators and buttons easily accessible on top surface
- **Mounting Holes**: For easy mechanical integration

---

## 📌 Pin Mapping Summary

| Arduino Nano Pin | Function           |
|------------------|--------------------|
| D3, D4           | Motor A direction  |
| D5, D6           | Motor B direction  |
| D9, D10          | Button inputs      |
| A0–A5            | Free for sensors   |
| VIN              | Motor power input  |
| +5V, GND         | Power distribution |

---

## ⚡ Power Section

- Input: 6V–12V through DC jack or terminal
- Regulated to +5V for logic and Arduino
- Filtered with 100µF and 220µF capacitors
- Diode protection included

---

## 🧰 Applications

This board is ideal for:
- Line-followers
- Mini autonomous vehicles
- PID control experiments
- Sensor + motor integration platforms

---

## 🔄 Future Enhancements

- Add screw terminals for battery/motor connection
- Reverse polarity protection
- Add sensor breakout pins for I2C modules

---

## 👤 Author

**Diego Méndez Carter**  
Component Engineer & Robotics Enthusiast  
## 📜 License

MIT License — feel free to use, modify, and share.
