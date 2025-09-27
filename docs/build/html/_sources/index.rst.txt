.. LittleBot documentation master file, created by
   sphinx-quickstart on Sep 27, 2025.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to LittleBot's Documentation!
======================================

LittleBot is a comprehensive ROS2 robotics package designed for educational and research purposes. 
This documentation covers all aspects of the LittleBot system, from installation and setup to 
advanced usage and API reference.

.. image:: https://img.shields.io/badge/ROS-ROS2-blue.svg
   :target: https://docs.ros.org/en/rolling/

.. image:: https://img.shields.io/badge/License-GPL-green.svg
   :target: https://www.gnu.org/licenses/gpl-3.0

Features
--------

* **Hardware Integration**: Complete integration with LittleBot hardware components
* **ROS2 Control**: Full support for ROS2 control framework
* **Simulation**: Gazebo simulation support for testing and development
* **Navigation**: Advanced navigation capabilities with localization
* **Visualization**: RQT plugins for monitoring and control
* **Serial Communication**: Protocol buffer-based communication with firmware

Quick Start
-----------

.. code-block:: bash

   # Clone the repository
   git clone https://github.com/NestorDP/littlebot.git
   
   # Build the workspace
   cd littlebot_ws
   colcon build
   
   # Source the workspace
   source install/setup.bash
   
   # Launch the robot
   ros2 launch littlebot_bringup littlebot_bringup.launch.py

.. toctree::
   :maxdepth: 2
   :caption: Contents:

   installation
   getting_started
   packages/index
   tutorials/index
   api/index
   troubleshooting
   contributing
   changelog

.. toctree::
   :maxdepth: 1
   :caption: Package Documentation:

   packages/littlebot_base
   packages/littlebot_bringup
   packages/littlebot_control
   packages/littlebot_description
   packages/littlebot_localization
   packages/littlebot_navigation
   packages/littlebot_teleop
   packages/littlebot_gazebo
   packages/littlebot_rqt_plugin

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`