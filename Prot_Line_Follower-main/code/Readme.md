# Line Follower Control Firmware (PID)

Arduino-based control firmware implementing real-time closed-loop PID tracking for a differential-drive line follower equipped with a 6-channel analog infrared reflectance array.

---

## Hardware Pinout

```cpp
// Motor Direction & PWM Control (L293D / L298N)
#define motIzq2 13     // Left motor direction 2
#define motIzq1 2      // Left motor direction 1
#define motDer1 12     // Right motor direction 1
#define motDer2 7      // Right motor direction 2
#define pwmIzq 11      // Left motor PWM enable
#define pwmDer 5       // Right motor PWM enable

// User Inputs
#define button 9       // Start / calibration trigger (internal pull-up)

// Sensor Array (Analog Pins)
const int sensores[6] = {A0, A1, A2, A3, A4, A5};
```

---

## Mathematical Formulation & Control Law

### 1. Weighted Position Estimation
Sensor readings $s_i \in [0, 1000]$ (normalized via startup calibration) are mapped to a continuous line position using a weighted average:

$$\text{pos} = \frac{\sum_{i=0}^{5} i \cdot s_i}{\sum_{i=0}^{5} s_i}$$

Where line centering corresponds to $\text{setPoint} \approx 2.38$.

### 2. Error Signal
$$e(t) = \text{pos}(t) - \text{setPoint}$$

### 3. Discrete PD Control Law
Because steady-state error is negligible and integral windup degrades high-speed line tracking, an anti-windup PD configuration ($K_i = 0$) is used:

$$u(t) = K_p e(t) + K_d \left( \frac{e(t) - e(t-\Delta t)}{\Delta t} \right)$$

Default tuning parameters:
- `Kp = 550` (Proportional gain)
- `Kd = 150` (Derivative gain)
- `Ki = 0` (Integral gain disabled)
- `velStd = 255` (Base motor speed)

### 4. Differential PWM Output
$$PWM_{\text{izq}} = \text{clamp}(v_{\text{std}} - u(t), 0, 255)$$
$$PWM_{\text{der}} = \text{clamp}(v_{\text{std}} + u(t), 0, 255)$$

---

## Startup Calibration Procedure

1. At startup, the robot samples ambient reflectance while swiveling across the track.
2. Minimum reflectance (`white[i]`) and maximum reflectance (`black[i]`) thresholds are recorded per channel.
3. Subsequent runtime readings are mapped linearly:
   $$s_i = \text{map}(\text{raw}_i, \text{white}_i, \text{black}_i, 0, 1000)$$

---

## Compilation & Upload

- Target Board: Arduino Nano (ATmega328P, Old Bootloader / New Bootloader)
- Dependencies: Standard AVR / Arduino core libraries (no external dependencies)
- Compilation tool: Arduino IDE or CLI (`arduino-cli compile --fqbn arduino:avr:nano`)

---

## License

MIT License.
