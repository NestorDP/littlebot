Troubleshooting
===============

This section covers common issues and their solutions when working with LittleBot.

Installation Issues
-------------------

ROS2 Not Found
~~~~~~~~~~~~~~~

**Problem**: ``ros2`` command not found or ROS2 packages not available.

**Solution**:

.. code-block:: bash

   # Make sure ROS2 is sourced
   source /opt/ros/humble/setup.bash
   
   # Add to your .bashrc for permanent sourcing
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

Build Errors
~~~~~~~~~~~~~

**Problem**: ``colcon build`` fails with dependency errors.

**Solution**:

.. code-block:: bash

   # Update rosdep database
   rosdep update
   
   # Install missing dependencies
   rosdep install --from-paths src --ignore-src -r -y
   
   # Clean build and rebuild
   rm -rf build install log
   colcon build

Hardware Issues
---------------

Serial Port Access
~~~~~~~~~~~~~~~~~~

**Problem**: Permission denied when accessing serial port.

**Solution**:

.. code-block:: bash

   # Add user to dialout group
   sudo usermod -a -G dialout $USER
   
   # Log out and back in, or use:
   newgrp dialout
   
   # Verify port exists and permissions
   ls -l /dev/ttyUSB*
   sudo chmod 666 /dev/ttyUSB0  # Temporary fix

Communication Timeout
~~~~~~~~~~~~~~~~~~~~~

**Problem**: Hardware communication timeouts or no response.

**Solutions**:

1. **Check physical connections**
2. **Verify firmware is running**
3. **Check baud rate settings**:

   .. code-block:: yaml

      hardware:
        baud_rate: 115200  # Match firmware settings

4. **Test serial communication manually**:

   .. code-block:: bash

      # Install screen for testing
      sudo apt install screen
      
      # Test communication
      screen /dev/ttyUSB0 115200

Control Issues
--------------

Robot Not Moving
~~~~~~~~~~~~~~~~

**Problem**: Commands sent but robot doesn't move.

**Diagnosis**:

.. code-block:: bash

   # Check if commands are being received
   ros2 topic echo /cmd_vel
   
   # Check joint states
   ros2 topic echo /joint_states
   
   # Check controller status
   ros2 control list_controllers

**Solutions**:

1. **Verify controller is loaded and active**:

   .. code-block:: bash

      ros2 control load_controller diff_drive_controller
      ros2 control switch_controllers --activate diff_drive_controller

2. **Check hardware interface status**:

   .. code-block:: bash

      ros2 control list_hardware_interfaces

Simulation Issues
-----------------

Gazebo Won't Launch
~~~~~~~~~~~~~~~~~~~

**Problem**: Gazebo fails to start or crashes.

**Solutions**:

1. **Update Gazebo**:

   .. code-block:: bash

      sudo apt update
      sudo apt upgrade gazebo

2. **Check graphics drivers**:

   .. code-block:: bash

      # For NVIDIA users
      nvidia-smi
      
      # For Intel integrated graphics
      glxinfo | grep "OpenGL renderer"

3. **Use software rendering** (if hardware acceleration fails):

   .. code-block:: bash

      export LIBGL_ALWAYS_SOFTWARE=1
      ros2 launch littlebot_gazebo littlebot_gazebo_classic.launch.py

Robot Model Issues
~~~~~~~~~~~~~~~~~~

**Problem**: Robot appears incorrectly in simulation or RViz.

**Solutions**:

1. **Check URDF/Xacro files**:

   .. code-block:: bash

      # Validate URDF
      check_urdf src/littlebot_description/urdf/littlebot.urdf.xacro

2. **Update robot description**:

   .. code-block:: bash

      ros2 param set /robot_state_publisher robot_description "$(xacro src/littlebot_description/urdf/littlebot.urdf.xacro)"

Navigation Issues
-----------------

Localization Problems
~~~~~~~~~~~~~~~~~~~~

**Problem**: Robot doesn't localize correctly or loses track of position.

**Solutions**:

1. **Check sensor data**:

   .. code-block:: bash

      # Verify laser scan
      ros2 topic echo /scan
      
      # Check odometry
      ros2 topic echo /odom

2. **Verify map and transforms**:

   .. code-block:: bash

      # Check transform tree
      ros2 run tf2_tools view_frames.py
      evince frames.pdf

3. **Tune localization parameters** in your configuration files.

Path Planning Issues
~~~~~~~~~~~~~~~~~~~

**Problem**: Navigation fails to plan paths or robot gets stuck.

**Solutions**:

1. **Check costmaps**:

   .. code-block:: bash

      # Visualize costmaps in RViz
      ros2 launch nav2_bringup navigation_launch.py

2. **Verify robot footprint** matches physical robot
3. **Adjust planner parameters** for your environment

RQT Plugin Issues
-----------------

Plugin Not Loading
~~~~~~~~~~~~~~~~~~

**Problem**: RQT plugin doesn't appear or fails to load.

**Solutions**:

1. **Force plugin discovery**:

   .. code-block:: bash

      rqt --force-discover

2. **Check plugin registration**:

   .. code-block:: bash

      # Verify plugin.xml is properly exported
      ros2 pkg xml littlebot_rqt_plugin

3. **Clear RQT cache**:

   .. code-block:: bash

      rm -rf ~/.config/ros.org/rqt_gui.ini

Qt Issues
~~~~~~~~~

**Problem**: GUI elements don't display correctly or crash.

**Solutions**:

1. **Update Qt packages**:

   .. code-block:: bash

      sudo apt update
      sudo apt install qt5-default qtbase5-dev

2. **Check environment variables**:

   .. code-block:: bash

      echo $QT_PLUGIN_PATH
      echo $DISPLAY

Log Analysis
------------

Enabling Debug Logging
~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # Set ROS2 logging level
   export RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG
   
   # Or for specific nodes
   ros2 run littlebot_base hardware_component --ros-args --log-level DEBUG

Common Log Messages
~~~~~~~~~~~~~~~~~~

**"Hardware interface not ready"**:
- Check hardware connections
- Verify firmware is running
- Ensure correct serial port configuration

**"Controller failed to switch"**:
- Check controller configuration
- Verify hardware interface is active
- Review joint names in configuration

**"Transform timeout"**:
- Check if all required transforms are being published
- Verify transform tree with ``tf2_tools``

Getting Help
------------

If you're still experiencing issues:

1. **Check the GitHub Issues**: https://github.com/NestorDP/littlebot/issues
2. **Review ROS2 documentation**: https://docs.ros.org/en/humble/
3. **Ask on ROS Discourse**: https://discourse.ros.org/
4. **Join the ROS community**: https://www.ros.org/community/

When reporting issues, please include:
- Operating system and ROS2 version
- Complete error messages
- Steps to reproduce the problem
- Relevant configuration files