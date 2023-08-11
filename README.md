# littleBOT
![Prints simulação LittleBOT](https://user-images.githubusercontent.com/37759765/128800773-a2714fbc-2218-4c7c-a7a5-e6070d67b1a1.png)


The Littlebot stack is an exciting and versatile package group to configure the Littlebot robot - a low-cost educational robot developed specifically for learning the principles of robotics. With a focus on simplicity, affordability, and educational value, the Littlebot robot serves as an ideal platform for students, hobbyists, and educators to delve into the fascinating world of robotics.

**Key Features:**

- Educational Focus: The Littlebot robot is purpose-built to help beginners grasp fundamental robotics concepts in a hands-on and engaging manner.
- ROS2 Compatibility: Leveraging the power of ROS2, the stack allows for efficient communication, control, and integration with various robotic components.
- Configurability: Customize the Littlebot robot's behavior and capabilities through easy-to-use configuration options provided by the ROS2 package.
- Extensibility: Expand the Littlebot's capabilities by adding your own ROS2 nodes and integrating additional sensors and actuators.

Whether you are a student embarking on your robotics journey or an educator seeking an accessible platform for teaching robotics, the Littlebot ROS2 stack empowers you to explore, experiment, and learn in a fun and interactive manner.


**Simulation**: For those interested in simulating the Littlebot robot, we have a dedicated repository that provides details and resources for simulating the Littlebot using the powerful Gazebo simulation environment. You can find the simulation repository at the following link:

[lttleBOT Gazebo](https://github.com/NestorDP/littlebot_gazebo)

Please refer to the simulation repository for instructions on setting up the Littlebot simulation and exploring its capabilities virtually.

**Hardware**: This repository serves as the ROS2 package for configuring the Littlebot robot. If you are interested in the hardware specifications and assembly instructions, please refer to the hardware documentation provided separately at the following link:

 [littleBOT firmware](https://github.com/NestorDP/littlebot_firmware)

Get started by following the installation instructions below to set up the Littlebot ROS2 package on your system. We welcome contributions from the community to enhance and improve the Littlebot robot, making robotics education accessible to everyone.


## Table of Contents

- [Installation](#installation)
- [Usage](#usage)
- [Configuration](#configuration)
- [Contributing](#contributing)
- [License](#license)
- [Credits](#credits)
- [Contact](#contact)


## Installation

To get started with the Littlebot ROS2 package, follow the steps below:

  ### 1. Create a ROS2 Workspace:
  Before you begin, make sure you have ROS2 (Robot Operating System 2) installed on your system. If you don't have it, you can follow the official ROS2 installation guide for your operating system. Once ROS2 is installed, create a new ROS2 workspace if you don't have one already. You can create a workspace by running the following commands in your terminal:

  ```bash
  # Create a new ROS2 workspace (if you have not created one yet)
  mkdir -p ~/littlebot_ws/src
  cd ~/littlebot_ws/
  colcon build
  ```

  ### 2. Download the Littlebot ROS2 Package:
  Next, navigate to the src directory of your ROS2 workspace, and clone the Littlebot ROS2 package repository from GitHub:

  ```bash
  # Move to the 'src' directory of your ROS2 workspace
  cd ~/littlebot_ws/src/

  # Clone the Littlebot ROS2 package repository
  git clone https://github.com/NestorDP/littlebot.git
  ```

  ### 3. Resolve Dependencies:
  Once you have cloned the repository, run rosdep to resolve any package dependencies that the Littlebot ROS2 package requires:

  ```bash
  # Navigate to the root of your ROS2 workspace
  cd ~/littlebot_ws/

  # Use rosdep to install dependencies
  rosdep install --from-paths src --ignore-src -r -y
  ```

  ### 4. Build the Package:
  After resolving the dependencies, build the package using colcon:
  
  ```bash 
  # Build the package
  colcon build
  ```
  The build process will compile the Littlebot ROS2 package and make it ready for use.

  ### 5. Source the Workspace:
  To ensure ROS2 can find the Littlebot ROS2 package, you need to source your workspace. Run the following command:

  ```bash
  # Source the workspace (you might want to add this line to your .bashrc or .bash_profile)
  source ~/littlebot_ws/install/setup.bash
  ```

  ### 6. Verify the Installation:
  To verify that the package is correctly installed and accessible, you can list the ROS2 packages available in your workspace:

  ```bash
  # List packages in your ROS2 workspace
  colcon list
  ```
  
  If the Littlebot ROS2 package is listed, you have successfully installed it.

  Congratulations! You have now installed the Littlebot ROS2 package, and you can begin configuring and experimenting with your Littlebot robot using ROS2.
  
## Usage

## Configuration


## Contribution

If ou want to report a bug, submit a feature request, or contribute code, your input is valuable in making this project better.

### Bug Reports and Feature Requests

If you come across any issues or have ideas for new features, please feel free to [open an issue](https://github.com/NestorDP/littlebot/issues). When creating a new issue, please use the available issue template and provide as much detail as possible. This information will help us better understand and address your request.

### Contributing Code

If you want to contribute code to the project, we follow a typical pull request (PR) workflow:

1. Fork the repository and create a new branch from the `devel/ros2-<ros distro>` branch.
2. Make your changes in the new branch and test them thoroughly.
3. Ensure your code follows the project's coding conventions and standards.
4. Submit a pull request (PR) to the `devel/ros2-<ros distro>` branch of this repository.
5. Include a detailed description of your changes and the problem they solve.

I will review your PR, provide feedback if needed, and work with you to ensure your contribution aligns with the project's goals.

### Code Guidelines

To maintain a consistent codebase, please follow these guidelines when contributing:

- Adhere to the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html) and formatting conventions.
- Write clear, concise, and well-documented code.
- Ensure your changes do not break existing functionality.
- Include unit tests to validate your code.

## license
By contributing to this project, you agree that your contributions will be licensed under the [**GNU General Public License v3.0**](https://github.com/NestorDP/littlebot/blob/devel/ros2-foxy/LICENSE). This allows us to maintain an open and collaborative environment for the community.

Thank you for considering contributing to our project. Your involvement makes a significant difference in the development and success of LittleBOT!

## credits
