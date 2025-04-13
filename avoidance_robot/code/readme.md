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

These constants define how each sensor and actuator connects to the microcontroller. It’s structured for easy sensor referencing.

---

### 🧠 Control Constants

```cpp
const float K_dist = 1.26;   // Control based on ultrasound difference
const float K_dist_c = 0.2;  // Frontal proximity correction
const float K_luz = 0.18;    // Control based on light differential
```

These constants tune the robot’s response to sensor inputs — allowing for smoother, adaptive movement based on proximity and light direction.

---

## 🧠 Control Algorithm

### 🔄 Navigation Logic

```cpp
PWM_izq = vi_izq + K_dist*(US_left - US_right) + K_luz*(light_diff);
PWM_der = vi_der + K_dist*(US_right - US_left) + K_luz*(-light_diff);
```

- `K_dist` is used to adjust direction based on difference in distance from left and right ultrasonic sensors.
- `K_luz` biases the motion toward or away from light based on photodetector values.
- `K_dist_c` applies a braking effect when something is too close to the front.

---

## 📦 Behaviors

### Motion Control Functions

- `Dir_Adel()` – Move forward  
- `Dir_Atr()` – Reverse  
- `Dir_Izq()` – Turn left  
- `Dir_Der()` – Turn right  
- `Retroceso_Izq()` / `Retroceso_Der()` – Perform a reverse-turn maneuver  
- `Navegacion()` – Core function combining logic from sensors to compute direction and PWM values

---

## 🛡️ Watchdog Integration

```cpp
#include <avr/wdt.h>
wdt_enable(WDTO_8S);
```

A watchdog timer is included to **reset the microcontroller** if it gets stuck or fails to update movement for 8 seconds. This helps avoid full system hangs in real-world applications.

---

## 🧪 Sensor Data Processing

### Ultrasonic Distance

```cpp
Duration[i] = pulseIn(US_echo_pin[i], HIGH, rango_US[i]);
US_lectura_mm[i] = valor_US_max - fDistancia(Duration[i]);
```

- Measures the time it takes for ultrasonic pulses to reflect back.
- Converts it into millimeters and reverses the scale: the **closer the object, the higher the value**.

### Light Detection

```cpp
Luz_valor[i] = analogRead(Luz_pin[i]);
```

- Captures raw analog values from photodetectors for light-following behavior.

---

## 🧾 Serial Output

```cpp
Serial.print("USizq UScen USder ... PWM_izq PWM_der: ");
```

- Regular printing of PWM and sensor values is used for real-time debugging.
- Helps tune constants (`K_dist`, `K_luz`) based on how the robot reacts.

---

## ✍️ Author of Code

**Adrián Ricárdez Ortigosa**  
Robotics Instructor  
Club de Robótica, Facultad de Ingeniería

---

## 📘 Use Context

This code was used during a lab course to help students understand:
- Sensor fusion (ultrasound + light)
- PWM-based motor control
- Direction switching using H-bridges
- System safety using watchdog timers

---

## 📜 License

This code is educational and belongs to the original author. It is used here for learning documentation only.
