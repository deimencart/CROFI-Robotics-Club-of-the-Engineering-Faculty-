# 🧠 ROS Nodes – Beach Cleaner Robot

> This folder contains ROS nodes written for the **Beach Cleaner Robot** project, focused on **teleoperation and motor control**. Developed using **ROS Noetic**, the nodes enable real-time driving via keyboard and communication with **Roboclaw motor drivers**.

---

## 📂 Included Nodes

### 1. **Teleop Twist Keyboard**
- Allows driving the robot with WASD keys over ROS topics
- Publishes `geometry_msgs/Twist` to `/cmd_vel`
- Configurable linear and angular velocities

📄 File: `teleop_twist_keyboard.py`

---

### 2. **Roboclaw Movement Node**
- Subscribes to `/cmd_vel` Twist messages
- Converts linear/angular velocity into motor commands
- Communicates with Roboclaw via serial interface
- Sends speed and direction to both motors

📄 File: `roboclaw_twist.py`

---

## 🔧 System Architecture

```
[Keyboard Input] ---> [teleop_twist_keyboard] ---> /cmd_vel ---> [roboclaw_twist] ---> [Roboclaw Motors]
```

- `teleop_twist_keyboard` → publishes velocities  
- `roboclaw_twist` → interprets & sends movement to hardware

---

## 🚀 How to Use

1. Launch ROS core:
   ```bash
   roscore
   ```

2. Run Roboclaw motor node:
   ```bash
   rosrun ros_nodes roboclaw_twist.py
   ```

3. In another terminal, launch teleoperation:
   ```bash
   rosrun ros_nodes teleop_twist_keyboard.py
   ```

4. Use your keyboard to control the robot via terminal.

---

## ⚙️ Requirements

- ROS Noetic (Ubuntu 20.04)
- Roboclaw motor controller
- USB-to-serial adapter for hardware interface
- `geometry_msgs`, `std_msgs`, `rospy`

---

## ✍️ Author

**Diego Méndez Carter**  
Robotics Engineer | ROS Developer | Embedded Systems  
---

## 📜 License

This folder is part of the *Beach Cleaner Robot* project. Use it freely for educational or hobby robotics development.

