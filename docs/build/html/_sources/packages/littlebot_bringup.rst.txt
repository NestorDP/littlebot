littlebot_bringup
================

The `littlebot_bringup` package provides launch files and configuration for starting the complete LittleBot system.

Overview
--------

This package contains launch files that coordinate the startup of all LittleBot components in the correct order, ensuring proper initialization and communication between different subsystems.

Components
----------

Launch Files
~~~~~~~~~~~~

The package includes launch files for different operational scenarios:

**Main Launch File**: `launch/littlebot_bringup.launch.py`

This is the primary launch file that starts the complete LittleBot system including:

* Hardware interface and communication
* Controller manager and controllers
* Robot state publisher
* Transform publishers
* Basic sensor drivers

Key Features:

* **Configurable Parameters**: Hardware port, controller settings, robot model
* **Conditional Launching**: Can be configured for real hardware or simulation
* **Error Handling**: Includes proper node lifecycle management
* **Resource Management**: Optimized for embedded systems

Usage
-----

Basic System Startup
~~~~~~~~~~~~~~~~~~~~~

Start the complete LittleBot system:

.. code-block:: bash

   # Launch the full system
   ros2 launch littlebot_bringup littlebot_bringup.launch.py

   # Launch with custom parameters
   ros2 launch littlebot_bringup littlebot_bringup.launch.py \
       serial_port:=/dev/ttyUSB0 \
       use_sim_time:=false

   # Launch for simulation
   ros2 launch littlebot_bringup littlebot_bringup.launch.py \
       use_fake_hardware:=true

Available Parameters
~~~~~~~~~~~~~~~~~~~~

The launch file accepts the following parameters:

.. list-table:: Launch Parameters
   :widths: 25 25 50
   :header-rows: 1

   * - Parameter
     - Default Value
     - Description
   * - ``serial_port``
     - ``/dev/ttyUSB0``
     - Serial port for hardware communication
   * - ``use_fake_hardware``
     - ``false``
     - Use simulated hardware instead of real robot
   * - ``use_sim_time``
     - ``false``
     - Use simulation time instead of system time
   * - ``robot_description_file``
     - ``littlebot.urdf.xacro``
     - Robot model file to load
   * - ``controllers_file``
     - ``control.yaml``
     - Controller configuration file

System Architecture
-------------------

Launch Sequence
~~~~~~~~~~~~~~~

The bringup launch file follows this startup sequence:

1. **Load Robot Description**: Parse URDF/Xacro files and publish robot model
2. **Start Hardware Interface**: Initialize communication with robot hardware
3. **Launch Controller Manager**: Start the ROS2 Control framework
4. **Load Controllers**: Configure and activate motion controllers
5. **Start State Publishers**: Begin publishing joint states and transforms
6. **Initialize Sensors**: Start any connected sensor drivers

Node Dependencies
~~~~~~~~~~~~~~~~~

The launch file manages these key nodes:

* ``robot_state_publisher``: Publishes robot transforms
* ``controller_manager``: Manages hardware controllers
* ``littlebot_hardware_component``: Hardware interface
* ``joint_state_broadcaster``: Publishes joint states
* ``diff_drive_controller``: Differential drive control

Configuration
-------------

Hardware Configuration
~~~~~~~~~~~~~~~~~~~~~~

Configure hardware settings by modifying parameters:

.. code-block:: yaml

   # Hardware interface parameters
   hardware:
     plugin: "littlebot_base/LittlebotHardwareComponent"
     parameters:
       serial_port: "/dev/ttyUSB0"
       baud_rate: 115200
       timeout: 1000

Controller Configuration
~~~~~~~~~~~~~~~~~~~~~~~~

Controllers are configured through the control system:

.. code-block:: yaml

   # Differential drive controller
   diff_drive_controller:
     type: diff_drive_controller/DiffDriveController
     left_wheel_names: ["left_wheel_joint"]
     right_wheel_names: ["right_wheel_joint"]
     wheel_separation: 0.3
     wheel_radius: 0.05

Robot Model Configuration
~~~~~~~~~~~~~~~~~~~~~~~~~

The robot description is loaded from URDF/Xacro files:

.. code-block:: xml

   <!-- Robot model parameters -->
   <xacro:property name="wheel_radius" value="0.05"/>
   <xacro:property name="wheel_separation" value="0.3"/>
   <xacro:property name="base_length" value="0.4"/>

Troubleshooting
---------------

Common Issues
~~~~~~~~~~~~~

**Launch Fails to Start Hardware**:

* Check serial port permissions: ``ls -l /dev/ttyUSB*``
* Verify hardware connection and power
* Ensure correct baud rate configuration

**Controllers Don't Activate**:

* Check controller manager status: ``ros2 control list_controllers``
* Verify hardware interface: ``ros2 control list_hardware_interfaces``
* Review controller configuration file

**Transform Errors**:

* Ensure robot description loads correctly
* Check joint names match between URDF and controllers
* Verify transform tree: ``ros2 run tf2_tools view_frames.py``

Debugging
~~~~~~~~~

Enable debug logging for troubleshooting:

.. code-block:: bash

   # Launch with debug logging
   ros2 launch littlebot_bringup littlebot_bringup.launch.py \
       --ros-args --log-level DEBUG

   # Check specific node output
   ros2 node info /controller_manager
   ros2 topic echo /joint_states

Integration with Other Packages
-------------------------------

Navigation Integration
~~~~~~~~~~~~~~~~~~~~~~

The bringup system integrates with navigation:

.. code-block:: bash

   # Start base system
   ros2 launch littlebot_bringup littlebot_bringup.launch.py &
   
   # Add navigation
   ros2 launch littlebot_navigation littlebot_navigation.launch.py

Simulation Integration
~~~~~~~~~~~~~~~~~~~~~~

For simulation environments:

.. code-block:: bash

   # Start Gazebo simulation
   ros2 launch littlebot_gazebo littlebot_gazebo_classic.launch.py &
   
   # Start controllers (simulation mode)
   ros2 launch littlebot_bringup littlebot_bringup.launch.py \
       use_fake_hardware:=true use_sim_time:=true

Development and Testing
-----------------------

Testing the System
~~~~~~~~~~~~~~~~~~

Verify system startup:

.. code-block:: bash

   # Check all nodes are running
   ros2 node list
   
   # Verify controllers are active
   ros2 control list_controllers
   
   # Test robot movement
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
       '{linear: {x: 0.1}, angular: {z: 0.0}}'

System Health Monitoring
~~~~~~~~~~~~~~~~~~~~~~~~

Monitor system status:

.. code-block:: bash

   # Check system resources
   ros2 launch littlebot_bringup littlebot_bringup.launch.py &
   htop  # Monitor CPU/memory usage
   
   # Check communication health
   ros2 topic hz /joint_states
   ros2 topic hz /odom

Dependencies
------------

**ROS2 Packages**:

* ``launch``
* ``launch_ros``
* ``robot_state_publisher``
* ``controller_manager``
* ``littlebot_base``
* ``littlebot_control``
* ``littlebot_description``

**System Dependencies**:

* ROS2 Control framework
* Hardware interface packages
* Transform libraries

API Reference
-------------

For detailed launch file API, see the Python launch file documentation and ROS2 launch system guides.