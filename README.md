# Butler-Robot

# Overview
This project lays the foundation for an autonomous butler robot designed to operate in a café environment. The current implementation focuses on establishing the core navigation capabilities using ROS 2 and the Nav2 stack.

The repository contains a simulated café environment built in Gazebo, a differential drive robot model, and the necessary configurations to perform point-to-point autonomous navigation. The robot can successfully navigate between predefined locations such as a "home" position, the "kitchen," and customer tables. This project serves as the essential first step in developing a fully functional robotic butler.

# Features
1.Custom Gazebo Environment: A simulated café world with a kitchen area and tables.
2.Robot Integration: A differential drive robot model is successfully loaded and integrated into the simulation.
3.Autonomous Navigation: Utilizes the ROS 2 Nav2 stack to enable the robot to navigate to any specified goal within the mapped environment while performing dynamic obstacle avoidance.
4.Localization: Employs the AMCL (Adaptive Monte Carlo Localization) package to track the robot's position on the map.

# System Requirements

* OS: Ubuntu 20.04 LTS
* ROS 2 Version: ROS 2 Humble Hawksbill
* Simulation: Gazebo Fortress
* Build Tool: colcon

# Technical Architecture
The system's architecture is centered around the standard ROS 2 navigation stack, providing a robust foundation for future development.

* Navigation (Nav2): All autonomous navigation, including path planning, obstacle avoidance, and localization (AMCL), is handled by the industry-standard Nav2 stack. A pre-built map of the café is used for navigation tasks.
* Simulation (Gazebo): The entire system is validated in a Gazebo simulation that models the café environment, a differential drive robot, and its sensors (e.g., a LiDAR for mapping and obstacle avoidance).
* Visualization (RViz): RViz is used to visualize the robot's state, the sensor data, the map, and to send navigation goals for testing purposes.

# Installation and Setup
This section guides you through the one-time setup required to prepare the project.

* Prerequisites:
Ensure you have a complete installation of ROS 2 Humble on your system.
* Clone the Repository:
Create a ROS 2 workspace (e.g., butler_robot_ws) and clone this repository into its src directory.

```
mkdir -p ~/butler_robot_ws/src
cd ~/butler_robot_ws/src
git clone <your-repository-url>
```

* Install Dependencies:
Navigate to your workspace root and use rosdep to install any missing dependencies.

```
cd ~/butler_robot_ws
rosdep install --from-paths src -y --ignore-src
```

* Build the Workspace:
Use colcon to build the packages in your workspace. This command compiles the code and generates necessary files.

```
cd ~/butler_robot_ws
colcon build --symlink-install
```

# Usage
Follow these steps each time you want to run the simulation.

* Navigate to your Workspace:
Open a new terminal and change to your workspace directory.

```cd ~/butler_robot_ws```

* Source the Setup File:
You must source the workspace's setup file to make its packages available in the terminal.

```source install/setup.bash```

* Launch the Simulation:
Run the main launch file to start Gazebo, load the custom restaurent world, and spawn the robot.

```ros2 launch bumperbot_description gazebo.launch.py world_name:=kitchen```

  * ros2 launch: The command to run a ROS 2 launch file.

  * bumperbot_description: The name of the package containing the launch file.

  * gazebo.launch.py: The specific launch file that starts Gazebo and the robot.

  * world_name:=restaurent: An argument passed to the launch file, telling it to load your custom kitchen.world file       instead of a default one.

* Initialize the Robot's Position:
After launching, RViz will also open. The robot may not know its initial location on the map.

  * In the RViz window, click the "2D Pose Estimate" button in the toolbar.

  * Click and drag on the map at the robot's starting location to set its initial position and orientation.

* Send a Navigation Goal:
You can command the robot to move to a location using RViz.

  * Click the "Nav2 Goal" button in the RViz toolbar.

  * Click and drag on the map to set a destination goal. The robot will begin planning a path and moving towards it.

# Project Structure

```
butler_robot_ws/
└── src/
    └── bumperbot_description/
        ├── launch/             # Launch files (e.g., gazebo.launch.py)
        ├── worlds/             # Gazebo world files (e.g., restaurent.world)
        ├── maps/               # Nav2 maps and configuration
        ├── urdf/               # Robot description files
        ├── rviz/               # Custom RViz configuration files
        └── package.xml         # Package manifest
```
