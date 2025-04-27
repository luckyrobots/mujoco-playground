# SO100 Robot Control with MuJoCo

This project demonstrates how to load and control the SO100 robot from the MuJoCo Menagerie using Python.

## Requirements

- Python 3.8 or higher
- MuJoCo 2.3.0 or higher
- NumPy
- GLFW

## Installation

1. Install the required packages:
```bash
pip install -r requirements.txt
```

2. Make sure you have MuJoCo installed on your system. If not, follow the installation instructions at [MuJoCo's official website](https://mujoco.org/download).

## Usage

Run the control script:
```bash
python so100_control.py
```

The program will:
1. Load the SO100 robot model
2. Display a visualization window
3. Show available actuators and their ranges
4. Allow you to control actuators through the command line

### Controlling the Robot

- Enter actuator number and value in the format: `actuator_index value`
  - Example: `0 0.5` sets actuator 0 to 0.5
- Values will be automatically clamped to the actuator's valid range
- Press 'q' to quit the program

## Features

- Real-time visualization of the robot
- Interactive command-line control
- Automatic range checking for actuator values
- Clear display of available actuators and their ranges 