# 🧠 Labyrinth Robot – Code Logic (Arduino UNO)

> This document explains the autonomous navigation code developed for the **Labyrinth Robot**, a group project from the "Curso Robot Laberinto" by the Club de Robótica de la Facultad de Ingeniería.  
> The code was authored by **Adrián Ricárdez Ortigosa** and focuses on sensor-based decision-making using PWM and direction control with a standalone ATmega328P microcontroller.

---

## 🚀 Overview

The robot uses:
- 3 Ultrasonic sensors for obstacle detection
- 4 Photodetectors to detect light sources
- 2 DC motors controlled via an L293D motor driver
- Direction logic based on analog and distance sensor feedback

---

## 🧩 Key Components in the Code

### ⚙️ Pin Mapping

```cpp
const int Luz_pin[4] = {A3, A2, A5, A4};
const int US_trigger_pin[3] = {6, 2, 4};
const int US_echo_pin[3] = {7, 3, 5};
const int Motor_pin[4] = {9, 8, A1, A0};
const int PWM_pin[2] = {11, 10};
```

### 🧠 Control Constants

```cpp
const float K_dist = 1.26;   // Control based on ultrasound difference
const float K_dist_c = 0.2;  // Frontal proximity correction
const float K_luz = 0.18;    // Control based on light differential
```

---

## 🧠 Control Algorithm

### 🔄 Navigation Logic

```cpp
PWM_izq = vi_izq + K_dist*(US_left - US_right) + K_luz*(light_diff);
PWM_der = vi_der + K_dist*(US_right - US_left) + K_luz*(-light_diff);
```

- Left/right power adjusted using the **difference in ultrasonic readings**.
- Light sensor values contribute to fine steering adjustments.
- When facing obstacles directly, the robot **slows down proportionally** using `K_dist_c`.

---

## 📦 Behaviors

- `Dir_Adel()` – Move forward
- `Dir_Atr()` – Reverse
- `Dir_Izq()` – Turn left
- `Dir_Der()` – Turn right
- `Retroceso_Izq()` / `Retroceso_Der()` – Reverse + turn to avoid frontal collision
- `Navegacion()` – Main control loop based on sensor fusion

---

## 🛡️ Watchdog Integration

```cpp
#include <avr/wdt.h>
wdt_enable(WDTO_8S);
```

A watchdog timer is used to **reset the system** if it becomes unresponsive due to obstacle entrapment.

---

## 🧪 Sensor Data Processing

```cpp
US_lectura_mm[i] = valor_US_max - measured_distance;
Luz_valor[i] = analogRead(Luz_pin[i]);
```

- Distances are inverted to simplify decision logic (`higher = closer`).
- Light values are taken from analog photodetectors.

---

## 🧾 Serial Output

```cpp
Serial.print("USizq UScen USder ... PWM_izq PWM_der: ");
```

The code continuously prints distance, light, and PWM values for debugging and analysis.

---

## ✍️ Author of Code

**Adrián Ricárdez Ortigosa**  
Robotics Instructor  
Club de Robótica, Facultad de Ingeniería

---

## 📘 Use Context

This sketch was implemented and tested in a course setting, where students contributed to:
- Electronic assembly
- PCB design
- Mechanical integration
- Real-world robot tuning

---

## 📜 License

This code is educational and belongs to the original author. It is used here for learning documentation only.

