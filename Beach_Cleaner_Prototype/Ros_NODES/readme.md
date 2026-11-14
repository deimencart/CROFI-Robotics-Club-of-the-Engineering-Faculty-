# ROS Nodes — Beach Cleaner Teleoperation & Drive

ROS Noetic nodes for teleoperation and motor actuation on the Beach Cleaner mobile platform.

---

## Included Nodes

### 1. `teleop_twist_keyboard.py`
- Reads keyboard directional input (WASD) and publishes velocity targets.
- Topic: `/cmd_vel`
- Message Type: `geometry_msgs/Twist`
- Parameters: Linear velocity scale ($v_x$), angular velocity scale ($\omega_z$).

### 2. `driver_node.py`
- Subscribes to `/cmd_vel` and converts $[v_x, \omega_z]$ into left/right differential wheel velocities using standard kinematic equations:
  $$v_L = v_x - \frac{L}{2} \omega_z, \quad v_R = v_x + \frac{L}{2} \omega_z$$
- Transmits PWM duty cycles over serial to the motor controller.

---

## Usage Instructions

```bash
# Terminal 1: ROS Master
roscore

# Terminal 2: Hardware Driver Node
rosrun beach_cleaner driver_node.py

# Terminal 3: Keyboard Teleoperation
rosrun beach_cleaner teleop_twist_keyboard.py
```

---

## Dependencies

- ROS Noetic (Ubuntu 20.04)
- `rospy`
- `geometry_msgs`
- `pyserial`

---

## Author & License

- **Diego Méndez Carter**
- Licensed under the [MIT License](LICENSE).
