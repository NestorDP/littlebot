littlebot_base
==============

The `littlebot_base` package provides the hardware interface and communication layer for the LittleBot robot.

Overview
--------

This package contains:

* **Hardware Component**: ROS2 Control hardware interface
* **Communication**: Serial communication with robot firmware
* **Protocol Buffers**: Message serialization for firmware communication
* **Fake Hardware**: Simulation support for testing without physical hardware

Components
----------

Hardware Interface
~~~~~~~~~~~~~~~~~~

The main hardware interface class that integrates with ROS2 Control framework.

**File**: `src/littlebot_hardware_component.cpp`

.. doxygenclass:: littlebot_base::LittlebotHardwareComponent
   :members:

Key Methods:

* ``on_init()``: Initialize hardware interface
* ``on_configure()``: Configure hardware parameters
* ``on_activate()``: Activate hardware communication
* ``read()``: Read sensor data from hardware
* ``write()``: Write commands to hardware

Communication Layer
~~~~~~~~~~~~~~~~~~~

Handles serial communication with the robot's firmware.

**File**: `src/littlebot_communication.cpp`

.. doxygenclass:: littlebot_base::LittlebotCommunication
   :members:

Key Methods:

* ``setCommandVelocities()``: Send velocity commands
* ``getStatusVelocities()``: Read current velocities
* ``getStatusPositionsStatus()``: Read encoder positions
* ``receive()``: Receive data from firmware
* ``send()``: Send data to firmware

Protocol Buffers
~~~~~~~~~~~~~~~~

Message format for communication with firmware.

**File**: `src/littlebot_protocol.proto`

The protocol buffer definitions include:

.. code-block:: protobuf

   message LittlebotCommand {
     float left_velocity = 1;
     float right_velocity = 2;
     int32 timestamp = 3;
   }
   
   message LittlebotStatus {
     float left_velocity = 1;
     float right_velocity = 2;
     float left_position = 3;
     float right_position = 4;
     int32 timestamp = 5;
   }

Configuration
-------------

Hardware Parameters
~~~~~~~~~~~~~~~~~~~

Configure hardware settings in your launch file or parameter file:

.. code-block:: yaml

   hardware:
     plugin: "littlebot_base/LittlebotHardwareComponent"
     parameters:
       serial_port: "/dev/ttyUSB0"
       baud_rate: 115200
       timeout: 1000
       wheel_separation: 0.3
       wheel_radius: 0.05

Launch Files
~~~~~~~~~~~~

**Container Launch**: `launch/littlebot_base_container.launch.py`

Launches the hardware interface in a component container for better performance.

.. code-block:: python

   from launch import LaunchDescription
   from launch_ros.actions import ComposableNodeContainer
   from launch_ros.descriptions import ComposableNode

   def generate_launch_description():
       container = ComposableNodeContainer(
           name='littlebot_base_container',
           namespace='',
           package='rclcpp_components',
           executable='component_container',
           composable_node_descriptions=[
               ComposableNode(
                   package='littlebot_base',
                   plugin='littlebot_base::LittlebotHardwareComponent',
                   name='littlebot_hardware',
               ),
           ],
           output='screen',
       )
       return LaunchDescription([container])

Scripts
-------

Fake Hardware Script
~~~~~~~~~~~~~~~~~~~~

**File**: `scripts/littlebot_fake.py`

Provides a fake hardware interface for testing and simulation:

.. code-block:: bash

   # Run fake hardware
   ros2 run littlebot_base littlebot_fake.py

This script:

* Simulates sensor readings
* Responds to velocity commands
* Publishes fake odometry data
* Useful for testing without physical hardware

Testing
-------

Unit Tests
~~~~~~~~~~

The package includes comprehensive unit tests:

**Files**:
* `test/test_littlebot_communication.cpp`
* `test/test_littlebot_hardware_component.cpp`
* `test/mock_littlebot_communication.cpp`

Run tests with:

.. code-block:: bash

   colcon test --packages-select littlebot_base
   colcon test-result --verbose

Integration Tests
~~~~~~~~~~~~~~~~~

Test the hardware interface with real or simulated hardware:

.. code-block:: bash

   # Test with fake hardware
   ros2 run littlebot_base littlebot_fake.py &
   ros2 launch littlebot_base littlebot_base_container.launch.py

   # Check communication
   ros2 topic echo /joint_states
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}}'

Troubleshooting
---------------

Common Issues
~~~~~~~~~~~~~

**Serial Port Permission Denied**:

.. code-block:: bash

   # Add user to dialout group
   sudo usermod -a -G dialout $USER
   # Log out and back in

**Communication Timeouts**:

* Check serial port connection
* Verify baud rate settings
* Ensure firmware is running on robot

**Build Errors**:

.. code-block:: bash

   # Install protocol buffer compiler
   sudo apt install protobuf-compiler libprotobuf-dev
   
   # Rebuild
   colcon build --packages-select littlebot_base

API Reference
-------------

For detailed API documentation, see the generated Doxygen documentation.

Dependencies
------------

**ROS2 Packages**:
* hardware_interface
* controller_manager
* rclcpp
* rclcpp_components

**System Dependencies**:
* libserial-dev
* protobuf-compiler
* libprotobuf-dev