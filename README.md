# Autonomous Drone Landing using Fiducial Markers

## 🧩 Project Structure

```bash
quadrotor_simulation/
├── quadrotor_control/         # Contains the ROS Interfacing needed to run the Simulation
│   ├── launch/                # Launch files
│   ├── src/scripts/           # Python scripts: flight control, mission planning, AprilTag detection
│   ├── CMakeLists.txt
│   └── package.xml
├── quadrotor_description/     # Contains model descriptions, plugins, and world setup
│   ├── config/                # AprilTag settings configuration
│   ├── models/                # Drone and Platform URDF/Xacro models
│   ├── src/                   # Custom motor control plugin
│   ├── worlds/                # Simulation environment files
│   ├── CMakeLists.txt
│   └── package.xml
└── .gitignore
```

## 🛠 Dependencies

* ROS Noetic or Melodic
* Gazebo 9+
* `apriltag_ros`

## Setting up the Project

* Create a ROS workspace

```bash
mkdir catkin_ws
```

* Inside the workspcace, clone the repository

```bash
git clone git@github.com:theshauryajha/quadrotor_simulation.git
mv quadrotor_simulation src
```

* Install dependencies

```bash
rosdep install --from-paths src --ignore-src -r -y
```

* Make the scripts and the launch files executable

```bash
chmod +x src/quadrotor_control/src/scripts/*
chmod +x src/quadrotor_control/launch/*
chmod +x src/quadrotor_description/scripts/*
```

* Build and source the workspace

```bash
catkin_make
source devel/setup.bash
```

* Setup the simulation environment
```bash
source src/quadrotor_description/scripts/setup_env.sh
```

## Launching the simulation

* Run the main launch file

```bash
roslaunch quadrotor_control simulation.launch
```

* The simulation can also be launched with custom spawn parameters from the platform

```bash
roslaunch quadrotor_control simulation.launch x:=2.0 y:=3.5 yaw:=1.57
```

* This spawns the platform at (2.0, 3.5) and rotated in the xy plane by 90 degrees

## 🚧 Ongoing Work

The project is currently being extended to support remote control of the landing platform. Additionally, the mission planner is being modified to enable successful drone landings on a *moving* platform, adding dynamic interaction capabilities to the system.
