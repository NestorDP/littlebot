Installation
============

This guide will walk you through installing the LittleBot ROS2 packages and all necessary dependencies.

Prerequisites
-------------

Operating System
~~~~~~~~~~~~~~~~

LittleBot is tested and supported on:

* Ubuntu 22.04 LTS (Jammy Jellyfish) - **Recommended**
* Ubuntu 20.04 LTS (Focal Fossa)

ROS2 Installation
~~~~~~~~~~~~~~~~~

You need ROS2 Humble or later installed on your system.

.. code-block:: bash

   # Install ROS2 Humble (Ubuntu 22.04)
   sudo apt update && sudo apt install curl gnupg lsb-release
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   sudo apt update
   sudo apt install ros-humble-desktop

System Dependencies
~~~~~~~~~~~~~~~~~~~

Install the required system packages:

.. code-block:: bash

   # Development tools
   sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool
   
   # Hardware interface dependencies
   sudo apt install ros-humble-hardware-interface ros-humble-controller-manager
   
   # Simulation dependencies
   sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
   
   # Navigation dependencies
   sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
   
   # Additional utilities
   sudo apt install ros-humble-rqt* ros-humble-rviz2

Hardware Dependencies
~~~~~~~~~~~~~~~~~~~~~

For serial communication with the LittleBot hardware:

.. code-block:: bash

   # Serial communication libraries
   sudo apt install libserial-dev
   
   # Protocol buffers
   sudo apt install protobuf-compiler libprotobuf-dev

Installation Methods
--------------------

Method 1: From Source (Recommended for Development)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. **Create a workspace:**

   .. code-block:: bash

      mkdir -p ~/littlebot_ws/src
      cd ~/littlebot_ws/src

2. **Clone the repository:**

   .. code-block:: bash

      git clone https://github.com/NestorDP/littlebot.git
      git clone https://github.com/NestorDP/littlebot_gazebo.git
      git clone https://github.com/NestorDP/littlebot_rqt_plugin.git

3. **Install dependencies:**

   .. code-block:: bash

      cd ~/littlebot_ws
      rosdep init  # Only if you haven't run this before
      rosdep update
      rosdep install --from-paths src --ignore-src -r -y

4. **Build the workspace:**

   .. code-block:: bash

      colcon build
      source install/setup.bash

Method 2: Using vcstool (Recommended for Users)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. **Create a workspace:**

   .. code-block:: bash

      mkdir -p ~/littlebot_ws/src
      cd ~/littlebot_ws

2. **Import repositories:**

   .. code-block:: bash

      # Create a repos file or download from repository
      vcs import src < littlebot.repos

3. **Install and build:**

   .. code-block:: bash

      rosdep install --from-paths src --ignore-src -r -y
      colcon build
      source install/setup.bash

Verification
------------

Test your installation:

.. code-block:: bash

   # Source the workspace
   source ~/littlebot_ws/install/setup.bash
   
   # Test launch files
   ros2 launch littlebot_description littlebot_description.launch.py
   
   # In another terminal, check that nodes are running
   ros2 node list

Troubleshooting
---------------

Common Issues
~~~~~~~~~~~~~

**Build Errors:**

.. code-block:: bash

   # Clean build if you encounter issues
   rm -rf build install log
   colcon build

**Permission Issues with Serial Port:**

.. code-block:: bash

   # Add user to dialout group
   sudo usermod -a -G dialout $USER
   # Log out and log back in

**Missing Dependencies:**

.. code-block:: bash

   # Re-run rosdep
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y

Next Steps
----------

Once installation is complete, proceed to :doc:`getting_started` to learn how to use LittleBot.