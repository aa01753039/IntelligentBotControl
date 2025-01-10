# IntelligentBotControl

This repository contains the implementation of a Fuzzy Logic System, Fuzzy Coordinator, and PID Controller designed for robotics control. The repository includes examples of obstacle avoidance and right-edge following behaviors for a TurtleBot or similar robotic systems.

---

## Features

- **Fuzzy Logic System**:
  - Implements Membership Functions for inputs and outputs.
  - Generates a rule base with user-defined fuzzy rules.
  - Fuzzifies crisp inputs and performs defuzzification.

- **Fuzzy Coordinator**:
  - Coordinates outputs from multiple behaviors using fuzzy logic blending.
  - Handles context-dependent decision-making.

- **PID Controller**:
  - Implements a simple PID controller to manage specific robotic behaviors like maintaining a target distance.

- **ROS Integration**:
  - Uses ROS 2 for subscribing to sensor topics and publishing velocity commands.
  - Includes topics such as `/scan` for laser readings and `/cmd_vel` for velocity commands.

---

## Directory Structure

```
.
├── fuzzy_logic.py              # Implementation of fuzzy logic classes and methods.
├── fuzzy_coordinator.py        # Fuzzy Coordinator for blending behaviors.
├── pid_controller.py           # PID controller implementation.
├── main.py                     # Main script for running the robotics control system.
└── README.md                   # Project documentation.
```

---

## Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/<your-repo-name>.git
   cd <your-repo-name>
   ```

2. Ensure you have ROS 2 installed and properly set up. Follow the [ROS 2 installation guide](https://docs.ros.org/en/foxy/Installation.html) for your platform.

3. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```

---

## Usage

### 1. Fuzzy Logic System
The `fuzzy_logic.py` script provides an implementation of the fuzzy logic system with Membership Functions, Rule Base, and Fuzzifier. It can be used standalone or integrated with ROS.

### 2. Fuzzy Coordinator
The `fuzzy_coordinator.py` script coordinates multiple behaviors and blends them using fuzzy logic. Example applications include obstacle avoidance and edge following.

### 3. PID Controller
The `pid_controller.py` script provides a PID controller for fine-grained control, such as maintaining a target distance.

### 4. Running the ROS Node
The main scripts (`main.py`) demonstrate integrating these components with ROS 2. To run the robot control system:
   ```bash
   ros2 run <package_name> main.py
   ```

---

## Example Applications

1. **Obstacle Avoidance**:
   - Inputs: Front laser ranges.
   - Outputs: Linear and angular velocities.
   - Fuzzy rules determine the robot's speed and direction based on proximity to obstacles.

2. **Right-Edge Following**:
   - Inputs: Right and front-right laser ranges.
   - Outputs: Linear and angular velocities.
   - Fuzzy rules maintain a consistent distance from the wall on the right.

---

## Dependencies

- Python 3.8+
- ROS 2 (e.g., Foxy, Galactic)
- `numpy`
- ROS 2 message types: `LaserScan`, `Twist`

---

## Author

**Lesly Guerrero**  
November 25, 2023  

---

## License

This project is licensed under the MIT License. See the LICENSE file for details.
