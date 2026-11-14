# Labyrinth Robot — Navigation Firmware (ATmega328P)

Firmware implementation for autonomous navigation in maze/corridor environments, developed as part of the "Curso Robot Laberinto" at the Club de Robótica de la Facultad de Ingeniería (CROFI), UNAM.

Original code authored by Adrián Ricárdez Ortigosa (Robotics Instructor, CROFI).

---

## System Overview

- **Sensors**: 3 ultrasonic rangefinders (left, center, right) and 4 analog photodetectors (differential light orientation).
- **Actuators**: 2 DC gearmotors driven by an L293D dual H-bridge.
- **Controller**: Standalone ATmega328P running at 16 MHz.

---

## Pin Configuration

```cpp
const int Luz_pin[4] = {A3, A2, A5, A4};
const int US_trigger_pin[3] = {6, 2, 4};
const int US_echo_pin[3] = {7, 3, 5};
const int Motor_pin[4] = {9, 8, A1, A0};
const int PWM_pin[2] = {11, 10};
```

---

## Control Parameters & Navigation Logic

### Gain Definitions

```cpp
const float K_dist = 1.26;   // Proportional gain on ultrasonic lateral differential
const float K_dist_c = 0.2;  // Frontal proximity deceleration gain
const float K_luz = 0.18;    // Differential light tracking gain
```

### Navigation Law

Differential steering commands are computed by combining lateral distance error and light intensity differences:

```cpp
PWM_izq = vi_izq + K_dist * (US_left - US_right) + K_luz * (light_diff);
PWM_der = vi_der + K_dist * (US_right - US_left) + K_luz * (-light_diff);
```

- `K_dist`: Proportional steering response centering the vehicle between corridor walls.
- `K_luz`: Directional bias toward light sources from the photodetector array.
- `K_dist_c`: Attenuates forward velocity when frontal clearance drops below safety threshold.

---

## Motion Control Routines

- `Dir_Adel()`: Forward drive
- `Dir_Atr()`: Reverse drive
- `Dir_Izq()`: Counter-clockwise yaw turn
- `Dir_Der()`: Clockwise yaw turn
- `Retroceso_Izq()` / `Retroceso_Der()`: Reverse clearance maneuvers
- `Navegacion()`: Primary execution loop updating sensor fusion and PWM duty cycles

---

## Watchdog Timer

```cpp
#include <avr/wdt.h>
wdt_enable(WDTO_8S);
```

Hardware watchdog configured with an 8-second timeout (`WDTO_8S`) to reset the microcontroller in the event of code deadlock or stalled sensor polling.

---

## Sensor Acquisition

### Ultrasonic Pulse Timing

```cpp
Duration[i] = pulseIn(US_echo_pin[i], HIGH, rango_US[i]);
US_lectura_mm[i] = valor_US_max - fDistancia(Duration[i]);
```

Distance is computed from echo return pulse width, mapped to millimeters, and inverted for proximity weighting.

### Analog Light Acquisition

```cpp
Luz_valor[i] = analogRead(Luz_pin[i]);
```

Analog readings from the photodetector array provide differential heading references.

---

## Attribution & License

- Firmware Author: Adrián Ricárdez Ortigosa (Club de Robótica, Facultad de Ingeniería, UNAM).
- Educational documentation and reference under MIT License.
