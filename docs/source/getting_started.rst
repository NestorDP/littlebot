Getting Started
===============

This guide will help you get up and running with LittleBot quickly.

Basic Usage
-----------

Launching the Robot
~~~~~~~~~~~~~~~~~~~

To start the LittleBot system with all basic components:

.. code-block:: bash

   # Source your workspace
   source ~/littlebot_ws/install/setup.bash
   
   # Launch the complete system
   ros2 launch littlebot_bringup littlebot_bringup.launch.py

This will start:

* Hardware interface
* Controller manager
* Basic sensors
* Transform publishers

Visualization
~~~~~~~~~~~~~

To visualize the robot in RViz:

.. code-block:: bash

   # Launch robot description with RViz
   ros2 launch littlebot_description littlebot_description.launch.py

   # Or launch RViz separately
   rviz2 -d src/littlebot_control/rviz/diff_drive.rviz

Simulation
~~~~~~~~~~

To run LittleBot in Gazebo simulation:

.. code-block:: bash

   # Launch Gazebo Classic simulation
   ros2 launch littlebot_gazebo littlebot_gazebo_classic.launch.py
   
   # Or launch Ignition Gazebo simulation
   ros2 launch littlebot_gazebo littlebot_gazebo_ignition.launch.py

Control
~~~~~~~

Manual Control
^^^^^^^^^^^^^^

Use keyboard teleop to control the robot:

.. code-block:: bash

   # Launch teleop node
   ros2 launch littlebot_teleop littlebot_teleop.launch.py
   
   # Use WASD keys to move the robot
   # Press 'q' to quit

Programmatic Control
^^^^^^^^^^^^^^^^^^^^

Send velocity commands directly:

.. code-block:: bash

   # Publish twist messages
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.0}}'

GUI Control
^^^^^^^^^^^

Use the RQT plugin for advanced control:

.. code-block:: bash

   # Launch the RQT plugin
   ros2 launch littlebot_rqt_plugin littlebot_rqt_plugin.launch.py

Navigation
~~~~~~~~~~

For autonomous navigation:

.. code-block:: bash

   # Launch navigation stack
   ros2 launch littlebot_navigation littlebot_navigation.launch.py
   
   # Launch localization
   ros2 launch littlebot_localization littlebot_localization.launch.py

System Architecture
-------------------

Package Overview
~~~~~~~~~~~~~~~~

The LittleBot system consists of several packages:

* **littlebot**: Meta-package containing common files
* **littlebot_base**: Hardware interface and communication
* **littlebot_bringup**: Launch files for complete system
* **littlebot_control**: Controller configuration and launch files
* **littlebot_description**: Robot model (URDF/Xacro files)
* **littlebot_localization**: Localization algorithms
* **littlebot_navigation**: Navigation stack configuration
* **littlebot_teleop**: Teleoperation nodes
* **littlebot_gazebo**: Simulation worlds and models
* **littlebot_rqt_plugin**: GUI plugin for robot control

Topics and Services
~~~~~~~~~~~~~~~~~~~

Key ROS2 topics:

.. code-block:: bash

   # Robot state
   /joint_states         # Joint positions and velocities
   /robot_description    # Robot model
   
   # Control
   /cmd_vel             # Velocity commands
   /odom                # Odometry data
   
   # Sensors (when connected)
   /scan                # Laser scan data
   /imu                 # IMU data
   /camera/image_raw    # Camera feed

Configuration
-------------

Hardware Configuration
~~~~~~~~~~~~~~~~~~~~~~

Edit the hardware configuration file:

.. code-block:: bash

   # Edit hardware parameters
   nano src/littlebot_base/config/hardware.yaml

Key parameters:

.. code-block:: yaml

   hardware:
     serial_port: "/dev/ttyUSB0"  # Adjust to your serial port
     baud_rate: 115200
     timeout: 1000

Controller Configuration
~~~~~~~~~~~~~~~~~~~~~~~~

Configure the differential drive controller:

.. code-block:: bash

   # Edit controller parameters
   nano src/littlebot_control/config/control.yaml

Key parameters:

.. code-block:: yaml

   diff_drive_controller:
     left_wheel_names: ["left_wheel_joint"]
     right_wheel_names: ["right_wheel_joint"]
     wheel_separation: 0.3
     wheel_radius: 0.05

Common Workflows
----------------

Development Workflow
~~~~~~~~~~~~~~~~~~~

1. **Make changes to source code**
2. **Rebuild the workspace:**

   .. code-block:: bash

      cd ~/littlebot_ws
      colcon build --packages-select <package_name>
      source install/setup.bash

3. **Test your changes**

Testing Workflow
~~~~~~~~~~~~~~~~

1. **Run unit tests:**

   .. code-block:: bash

      colcon test --packages-select <package_name>
      colcon test-result --verbose

2. **Run integration tests:**

   .. code-block:: bash

      # Launch simulation
      ros2 launch littlebot_gazebo littlebot_gazebo_classic.launch.py
      
      # Test navigation
      ros2 launch littlebot_navigation littlebot_navigation.launch.py

Debugging
~~~~~~~~~

Common debugging commands:

.. code-block:: bash

   # Check node status
   ros2 node list
   ros2 node info /node_name
   
   # Monitor topics
   ros2 topic list
   ros2 topic echo /topic_name
   
   # Check transforms
   ros2 run tf2_tools view_frames.py
   ros2 run tf2_ros tf2_echo base_link odom

Next Steps
----------

* :doc:`tutorials/index` - Detailed tutorials for specific tasks
* :doc:`packages/index` - Detailed package documentation
* :doc:`api/index` - API reference documentation