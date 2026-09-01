# Dobot Magician E6 ROS 2 Driver

A lightweight ROS 2 interface for the **Dobot Magician E6**, based on the official DOBOT TCP/IP Python SDK.

This repository provides:

- ROS 2 control of the Magician E6
- Joint position commands
- Joint velocity commands
- Robot state feedback through `/joint_states`
- Home and packing motions
- Suction gripper control
- Emergency-stop recovery support
- RViz visualization
- Manual joystick control

The current development environment is based on:

- **Ubuntu 24.04**
- **ROS 2 Jazzy**
- **Python 3.12**
- **DOBOT TCP/IP Python SDK**

---

## Contents

1. [Basic Operation with DobotStudio Pro](#1-basic-operation-with-dobotstudio-pro)
2. [Testing the ES01 Suction Gripper](#2-testing-the-es01-suction-gripper)
3. [Safe Robot Shutdown](#3-safe-robot-shutdown)
4. [ROS 2 Installation](#4-ros-2-installation)
5. [Create the ROS 2 Workspace](#5-create-the-ros-2-workspace)
6. [Build the Workspace](#6-build-the-workspace)
7. [Run the Magician E6 Driver](#7-run-the-magician-e6-driver)
8. [RViz Visualization](#8-rviz-visualization)
9. [Manual Control with a Joystick](#9-manual-control-with-a-joystick)
10. [Manual Joint Position Commands](#10-manual-joint-position-commands)
11. [Manual Joint Velocity Commands](#11-manual-joint-velocity-commands)
12. [Emergency Stop and Robot Recovery](#12-emergency-stop-and-robot-recovery)

---

# 1. Basic Operation with DobotStudio Pro

Before using ROS 2, it is recommended to verify that the robot can be operated normally using **DobotStudio Pro**.

The Magician E6 is connected through **LAN1**.

Default robot IP:

```text
192.168.5.1
```

Configure the computer Ethernet interface with an IPv4 address in the same subnet, for example:

```text
IP address: 192.168.5.10
Subnet mask: 255.255.255.0
```

Open DobotStudio Pro and connect to the robot.

After connection:

1. Clear any active alarms.
2. Enable the robot.
3. Use the joint jog controls to verify motion.
4. Verify that all six joints move correctly.
5. Verify that the robot can reach its normal home configuration.

![Magician E6 basic joint jog](assets/magician_e6_basic_joint_jog.png)

---

# 2. Testing the ES01 Suction Gripper

The **Dobot ES01 suction gripper** can be tested in DobotStudio Pro using the corresponding Dobot+ plugin.

Install and open the ES01 plugin, then test:

- **Adsorb** — activate suction
- **Release** — deactivate suction

Confirm that the gripper responds correctly before using it through ROS 2.

![Dobot ES01 plugin control](assets/dobotes01_plugin_control.png)

---

# 3. Safe Robot Shutdown

Use the following procedure for normal shutdown:

1. Stop any active robot program or motion.
2. Release any workpiece held by the suction gripper when appropriate.
3. Move the robot to a safe configuration if necessary.
4. Disable the robot from DobotStudio Pro.
5. Press and hold the robot power button for approximately **1.5 seconds**.
6. Release the button when the robot begins the shutdown sequence.

The physical emergency stop should **not** be used as the normal shutdown method.

![Magician E6 safe shutdown](assets/magician_e6_safe_shutdown.png)

---

# 4. ROS 2 Installation

The ROS 2 driver communicates with the robot through the official DOBOT TCP/IP Python SDK.

Because the SDK is installed using `pip`, a Python virtual environment is used to isolate it from the system Python installation while still allowing access to the ROS 2 Python packages.

## 4.1 Create the project directory

Create a main directory for the Magician E6 project:

```bash
mkdir -p ~/MagicianE6
cd ~/MagicianE6
```

## 4.2 Create the Python virtual environment

Create a virtual environment named `venv`:

```bash
python3 -m venv --system-site-packages venv
```

The option:

```text
--system-site-packages
```

is important because it allows the virtual environment to access the Python packages installed by ROS 2.

Activate the environment:

```bash
source ~/MagicianE6/venv/bin/activate
```

Optionally update the Python packaging tools:

```bash
python3 -m pip install --upgrade pip setuptools wheel
```

## 4.3 Install the DOBOT TCP/IP Python SDK

Clone the official DOBOT TCP/IP Python SDK:

```bash
cd ~/MagicianE6

git clone \
  -b feature/v4-optimization \
  https://github.com/Dobot-Arm/TCP-IP-Python-V4.git
```

Enter the SDK directory:

```bash
cd ~/MagicianE6/TCP-IP-Python-V4
```

Install the SDK and its Python dependencies in editable mode:

```bash
pip install -e .
```

Verify the installation:

```bash
python3 -c "import dobot_sdk; print(dobot_sdk.__file__)"
```

A valid path inside the cloned `TCP-IP-Python-V4` repository should be displayed.

---

# 5. Create the ROS 2 Workspace

Create the ROS 2 workspace and its `src` directory:

```bash
mkdir -p ~/MagicianE6/magician_ws/src
```

Move into the source directory:

```bash
cd ~/MagicianE6/magician_ws/src
```

Clone this repository:

```bash
git clone https://github.com/WChamorro/simple_dobot_me6_ROS2_driver.git
```

The resulting structure should be similar to:

```text
MagicianE6/
├── venv/
├── TCP-IP-Python-V4/
└── magician_ws/
    └── src/
        └── simple_dobot_me6_ROS2_driver/
```

The repository contains the ROS 2 packages used for:

- custom messages
- robot communication
- robot description
- RViz visualization
- joystick control

---

# 6. Build the Workspace

Every new terminal used with this project should first load ROS 2 and activate the Python virtual environment:

```bash
source /opt/ros/jazzy/setup.bash
source ~/MagicianE6/venv/bin/activate
```

Move to the ROS 2 workspace:

```bash
cd ~/MagicianE6/magician_ws
```

Because the DOBOT SDK is installed in the virtual environment, build the workspace using the virtual environment Python interpreter:

```bash
python3 /usr/bin/colcon build --symlink-install
```

After the build completes:

```bash
source install/setup.bash
```

> **Important:** use `python3 /usr/bin/colcon` instead of calling the system `colcon` directly. This ensures that the ROS 2 Python packages are built using the Python environment where `dobot_sdk` is available.

---

# 7. Run the Magician E6 Driver

Before starting the driver, make sure that:

- the robot is powered on;
- the physical emergency stop is released;
- the computer is connected to **LAN1**;
- the computer is configured in the same subnet as the robot;
- `192.168.5.1` is reachable.

Launch the driver:

```bash
ros2 launch dobot_e6_driver run_dobot_driver.launch.py
```

During controller startup, the robot status light may blink blue.

Wait until the startup sequence has completed and the driver reports that the controller is ready.

## 7.1 Enable the robot

The robot is intentionally not enabled automatically.

Enable it with:

```bash
ros2 topic pub --once \
  /dobot/enable \
  std_msgs/msg/Bool \
  "{data: true}"
```

Disable it with:

```bash
ros2 topic pub --once \
  /dobot/enable \
  std_msgs/msg/Bool \
  "{data: false}"
```

Monitor the robot mode with:

```bash
ros2 topic echo /dobot/robot_mode
```

The normal enabled idle state is:

```text
RobotMode = 5
```

## 7.2 Driver topics

### Command topics

```text
/dobot/enable
    std_msgs/msg/Bool

/dobot/recover
    std_msgs/msg/Empty

/dobot/home
    std_msgs/msg/Empty

/dobot/packing
    std_msgs/msg/Empty

/dobot/joint_position_cmd
    dobot_e6_msgs/msg/JointPositionCommand

/dobot/joint_velocity_cmd
    dobot_e6_msgs/msg/JointVelocityCommand

/dobot/stop
    std_msgs/msg/Empty

/dobot/suction_gripper
    std_msgs/msg/Bool
```

### Feedback and monitor topics

```text
/joint_states
    sensor_msgs/msg/JointState

/dobot/robot_mode
    std_msgs/msg/Int32

/dobot/control_mode_monitor
    std_msgs/msg/String

/dobot/suction_gripper_monitor
    std_msgs/msg/Bool
```

All active topics can be inspected with:

```bash
ros2 topic list
```

## 7.3 Test the default Home position

Before commanding motion, make sure that the robot workspace is clear.

Send the robot to its configured Home position:

```bash
ros2 topic pub --once \
  /dobot/home \
  std_msgs/msg/Empty \
  "{}"
```

## 7.4 Test the default Packing position

Send the robot to its configured Packing position:

```bash
ros2 topic pub --once \
  /dobot/packing \
  std_msgs/msg/Empty \
  "{}"
```

The Home and Packing positions, together with their velocity and acceleration ratios, are configured in the driver YAML file.

---

# 8. RViz Visualization

The package:

```text
dobot_e6_description
```

contains the robot URDF, meshes, and RViz configuration.

The robot driver should already be running because the visualization uses the real joint feedback published on:

```text
/joint_states
```

Launch RViz and `robot_state_publisher` with:

```bash
ros2 launch dobot_e6_description visualization.launch.py
```

The visualization architecture is:

```text
Magician E6
     │
     ▼
dobot_e6_driver
     │
     │ /joint_states
     ▼
robot_state_publisher
     │
     ├── /tf
     └── /tf_static
             │
             ▼
            RViz2
```

The physical robot and the RViz model should move simultaneously.

---

# 9. Manual Control with a Joystick

The Magician E6 can be controlled manually using a USB or wireless joystick through the ROS 2 `joy` package.

## 9.1 Verify the joystick device

Connect the joystick or its wireless receiver.

Check the Linux input devices:

```bash
ls -l /dev/input/
```

The first joystick will normally appear as:

```text
/dev/input/js0
```

Verify the joystick directly:

```bash
jstest /dev/input/js0
```

If the axes and buttons respond correctly, test ROS 2 joystick input:

```bash
ros2 run joy joy_node --ros-args -p device_id:=0
```

In another terminal:

```bash
ros2 topic echo /joy
```

For the first joystick:

```text
/dev/input/js0  ->  device_id = 0
```

## 9.2 Launch joystick control

Launch both the ROS 2 joystick interface and the Magician E6 joystick controller:

```bash
ros2 launch dobot_e6_joystick joystick.launch.py
```

The device ID can also be specified explicitly:

```bash
ros2 launch \
  dobot_e6_joystick \
  joystick.launch.py \
  device_id:=0
```

## 9.3 Joystick mapping

The current manual control mapping is:

```text
Left stick UP / DOWN      -> Joint 1
Left stick LEFT / RIGHT   -> Joint 2

Right stick UP / DOWN     -> Joint 3
Right stick LEFT / RIGHT  -> Joint 4

Arrow UP                  -> Joint 5
Arrow DOWN                -> Joint 6

Y                         -> Suction ON / OFF

X                         -> Joint velocity STOP
                             [0, 0, 0, 0, 0, 0]
```

Joints 1 to 4 use proportional velocity control.

The farther an analog stick is displaced, the larger the commanded joint velocity.

Default maximum analog velocity:

```text
1.0 rad/s
```

Joints 5 and 6 use constant button-controlled velocity.

Default button velocity:

```text
0.5 rad/s
```

These values can be modified in:

```text
dobot_e6_joystick/config/joystick.yaml
```

The `X` button commands zero velocity to all six joints.

> The `X` button is a software motion stop for joystick velocity control. It does **not** replace the physical emergency stop.

---

# 10. Manual Joint Position Commands

Joint positions are specified in **radians**.

Example:

```bash
ros2 topic pub --once \
  /dobot/joint_position_cmd \
  dobot_e6_msgs/msg/JointPositionCommand \
  "{position: [0.0, 0.5, 0.0, 0.0, 0.0, 0.0], velocity_ratio: 20.0, acceleration_ratio: 20.0}"
```

The custom message contains:

```text
position
    Desired J1-J6 positions [rad]

velocity_ratio
    Robot velocity percentage

acceleration_ratio
    Robot acceleration percentage
```

---

# 11. Manual Joint Velocity Commands

Joint velocities are specified in:

```text
rad/s
```

For example, command Joint 1 at `0.2 rad/s`:

```bash
ros2 topic pub -r 30 \
  /dobot/joint_velocity_cmd \
  dobot_e6_msgs/msg/JointVelocityCommand \
  "{velocity: [0.2, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

The driver expects velocity commands continuously while operating in velocity mode.

Press:

```text
Ctrl+C
```

to stop the ROS 2 publisher. The driver's command watchdog stops the velocity mode when commands are no longer received.

A zero-velocity command can also be sent explicitly:

```bash
ros2 topic pub --once \
  /dobot/joint_velocity_cmd \
  dobot_e6_msgs/msg/JointVelocityCommand \
  "{velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

The driver applies its configured software velocity and joint-limit protections before sending motion commands to the robot.

---

# 12. Emergency Stop and Robot Recovery

The physical emergency stop remains the primary safety device.

If the emergency stop is pressed during operation:

1. The robot immediately stops.
2. Release the physical emergency stop.
3. Make sure the robot workspace is safe.
4. Run the driver recovery procedure.

Send:

```bash
ros2 topic pub --once \
  /dobot/recover \
  std_msgs/msg/Empty \
  "{}"
```

Monitor the robot state:

```bash
ros2 topic echo /dobot/robot_mode
```

The recovery procedure handles the required controller operations, including alarm clearing and controller power restoration when necessary.

After an emergency stop, the controller may require approximately one minute to complete its startup sequence.

When the robot reaches:

```text
RobotMode = 4
DISABLED
```

enable it again:

```bash
ros2 topic pub --once \
  /dobot/enable \
  std_msgs/msg/Bool \
  "{data: true}"
```

The expected final state is:

```text
RobotMode = 5
ENABLED / IDLE
```

## 12.1 Robot remains in PAUSE

If the controller remains in:

```text
RobotMode = 10
PAUSE
```

a previous movement may still be paused.

Do **not** automatically resume an old trajectory after an emergency stop.

Stop the previous motion:

```bash
ros2 topic pub --once \
  /dobot/stop \
  std_msgs/msg/Empty \
  "{}"
```

Verify the robot state before sending a new motion command.

---

# Typical Operating Sequence

A normal ROS 2 session follows this sequence:

```text
Power on Magician E6
        │
        ▼
Connect PC to LAN1
        │
        ▼
source ROS 2
        │
        ▼
activate Python venv
        │
        ▼
source workspace
        │
        ▼
launch dobot_e6_driver
        │
        ▼
wait for controller startup
        │
        ▼
enable robot
        │
        ├──────────────► RViz visualization
        │
        ├──────────────► Joystick control
        │
        ├──────────────► Joint position commands
        │
        └──────────────► Joint velocity commands
```

For a new terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/MagicianE6/venv/bin/activate
cd ~/MagicianE6/magician_ws
source install/setup.bash
```

---

# Safety Notes

- Always keep the physical emergency stop accessible.
- Verify that the robot workspace is clear before enabling motion.
- Start new velocity-control tests at low speed.
- Do not use software stop commands as a replacement for the physical emergency stop.
- After an emergency stop, verify the robot state before enabling motion again.
- Do not automatically resume a previously paused trajectory after an emergency stop.
- Make sure the suction gripper is released before normal shutdown when appropriate.

---

# License

This project is licensed under the **Apache License 2.0**.
