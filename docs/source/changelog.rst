Changelog
=========

All notable changes to this project will be documented in this file.

The format is based on `Keep a Changelog <https://keepachangelog.com/en/1.0.0/>`_,
and this project adheres to `Semantic Versioning <https://semver.org/spec/v2.0.0.html>`_.

[Unreleased]
------------

Added
~~~~~
- Comprehensive Sphinx documentation with Read the Docs theme
- API reference documentation
- Troubleshooting guide
- Contributing guidelines

Changed
~~~~~~~
- Improved RQT plugin registration and discovery

[1.0.0] - 2024-09-27
---------------------

Added
~~~~~
- Initial release of LittleBot ROS2 packages
- Hardware interface for robot communication
- Protocol buffer-based serial communication
- ROS2 Control integration
- Gazebo simulation support
- Navigation and localization packages
- RQT plugin for robot monitoring and control
- Teleoperation support
- Comprehensive launch files
- Unit and integration tests

Hardware Interface
~~~~~~~~~~~~~~~~~~
- ``littlebot_base`` package with hardware component
- Serial communication with configurable parameters
- Protocol buffer message serialization
- Fake hardware for testing without physical robot

Control System
~~~~~~~~~~~~~~
- Differential drive controller configuration
- Joint state publishing
- Velocity command handling
- Odometry publication

Simulation
~~~~~~~~~~
- Gazebo Classic support
- Gazebo Ignition support
- Robot model with meshes and materials
- Physics simulation configuration

Navigation
~~~~~~~~~~
- AMCL localization
- Navigation2 integration
- Map-based navigation
- Obstacle avoidance

User Interface
~~~~~~~~~~~~~~
- Keyboard teleoperation
- RQT plugin for GUI control
- RViz configuration files
- Real-time data visualization

Documentation
~~~~~~~~~~~~~
- Package documentation
- Launch file examples
- Configuration guides
- API reference