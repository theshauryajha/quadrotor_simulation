## Quadrotor Simulation

### Decription
- The quadrotor_description package defines the URDF of a quadrotor drone
- The quadrotor_control package contains the controllers for the drone and the launch files to run the simulation

### Use the package
- Create a ROS workspace

  ```bash
  mkdir catkin_ws
  ```
- Inside the workspace, clone this repository

  ```bash
  cd catkin_ws
  git clone https://github.com/theshauryajha/quadrotor_simulation.git
  mv quadrotor_simulation src
  ```
- Make the Python scripts and launch files executable

  ```bash
  chmod +x src/quadrotor_control/src/scripts/*
  chmod +x src/quadrotor_control/launch/*
  ```
- Build and source the workspace

  ```bash
  catkin_make
  source devel/setup.bash
  ```
- Start the simulation

  ```bash
  roslaunch quadrotor_control mission_simulation.launch
  ```