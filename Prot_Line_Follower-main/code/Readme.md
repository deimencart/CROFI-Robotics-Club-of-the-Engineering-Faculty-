# 🛴 Line Following Robot (PID Controlled)

This Arduino-based robot uses six analog IR sensors and a PID controller to follow a black line on a white surface. It calibrates itself automatically at startup and determines control signals for the motors to stay aligned with the line.

---

## 📦 Components

- Arduino UNO/Nano
- 2 DC Motors with L298N or H-bridge
- 6 IR analog sensors (connected to A0–A5)
- Push button (start/reset)
- LED (built-in)
- 2 wheels + chassis + power source

---

## ⚙️ Pin Definitions

```cpp
#define motIzq2 13     // Left motor, direction pin 2
#define motIzq1 2      // Left motor, direction pin 1
#define motDer1 12     // Right motor, direction pin 1
#define motDer2 7      // Right motor, direction pin 2
#define pwmIzq 11      // Left motor PWM speed control
#define pwmDer 5       // Right motor PWM speed control
#define button 9       // Start button (with pull-up)
```

---

## 🔧 PID Controller Settings

```cpp
#define setPoint 2.38  // Target position on the line (range 0–5)
#define velStd 255     // Base motor speed (max 255)
#define Kp 550         // Proportional gain
#define Kd 150         // Derivative gain
#define Ki 0           // Integral gain (disabled)
```

These values affect how the robot corrects its path. You can tune them for smoother or faster behavior.

---

## 📊 Sensor Calibration

```cpp
int sensores[6] = {A0, A1, A2, A3, A4, A5};  // Analog IR sensors
long black[6] = {0,0,0,0,0,0};               // Max values recorded during calibration
int white[6] = {1021,1021,1021,1021,1021,1021};  // Min values recorded during calibration
```

Each sensor maps its analog value between the `black` and `white` limits into a range from 0 to 100.

---

## 🧠 Control Variables

```cpp
int velIzq, velDer;
float posicion, error, lastError, derivative, control;
long double integral = 0;
```

These are used by the PID control loop to dynamically adjust the speed of each motor.

---

## 🔁 State Machine & Behavior

### 🔹 Start State

Waits for the button press to enter calibration mode.

### 🔹 Calibration

Reads each sensor thousands of times to determine their white (min) and black (max) response values.

### 🔹 Main Loop

1. Reads all six sensors.
2. Computes position on the line.
3. Calculates error from the target (`setPoint`).
4. Applies PID control to adjust motor speeds.
5. If all sensors are off-line (i.e., no line detected), it attempts to recover by turning in the last known direction.

### 🔹 Line Recovery

If the robot loses the line, it spins in the last known direction until it detects the line again.

---

## 🧮 PID Formula

```cpp
control = Kp * error - Kd * derivative + Ki * integral;

velIzq = velStd - control;
velDer = velStd + control;
```

### Explanation:
- `error`: Difference between target and actual position.
- `derivative`: How fast the error is changing.
- `integral`: Total accumulated error (disabled here).
- The control value adjusts motor speed to keep the robot on track.

---

## 🔁 Sensor Reading Weighting

Position is computed by a weighted average of sensor values:

```cpp
posicion = (0×s[0] + 1×s[1] + 2×s[2] + 3×s[3] + 4×s[4] + 5×s[5]) 
           / (s[0] + s[1] + s[2] + s[3] + s[4] + s[5]);
```

This gives a decimal between 0–5, where 2.38 represents the line centered.

---

## 🚦 Visual Feedback

- LED OFF: Waiting to start
- LED ON: Calibrating
- Blinking: Ready to run after calibration

---

## 🚨 Notes

- Code uses `goto` for transitions (can be refactored into `switch-case` or FSM for better readability).
- Calibrates sensors dynamically based on environment.
- Includes basic failsafe for when the robot loses the line.

---

## 🚀 Future Improvements

- Save calibration values in EEPROM
- Add Bluetooth module for remote PID tuning
- Improve state management structure

---

## 🧾 License

This project is open-source under the MIT License. Feel free to fork, modify, and contribute.

---

## 🙋‍♂️ Author

**Diego Méndez Carter**  
Component Engineer & Robotics Enthusiast  
GitHub: [@yourusername](https://github.com/yourusername)  
LinkedIn: [Your LinkedIn](https://www.linkedin.com/in/yourlinkedin)
