littlebot_control
================

The `littlebot_control` package provides ROS2 Control configuration, controllers, and launch files for managing robot motion and actuators.

Overview
--------

This package integrates with the ROS2 Control framework to provide:

* Controller configuration files
* Launch files for controller management
* RViz configuration for visualization
* Motion control interfaces
* Real-time control capabilities

Components
----------

Controllers
~~~~~~~~~~~

**Differential Drive Controller**

The primary motion controller for LittleBot's differential drive system.

**File**: Configured in `config/control.yaml`

Key features:
* Velocity command input via ``/cmd_vel`` topic
* Odometry publication on ``/odom`` topic
* Transform broadcasting (``base_link`` → ``odom``)
* Configurable wheel parameters

**Joint State Broadcaster**

Publishes current joint positions and velocities.

* Publishes to ``/joint_states`` topic
* Compatible with ``robot_state_publisher``
* Provides real-time joint information

Configuration Files
~~~~~~~~~~~~~~~~~~~

**Control Configuration**: `config/control.yaml`

Defines controller parameters:

.. code-block:: yaml

   controller_manager:
     ros__parameters:
       update_rate: 100  # Hz
       
   diff_drive_controller:
     ros__parameters:
       left_wheel_names: ["left_wheel_joint"]
       right_wheel_names: ["right_wheel_joint"]
       wheel_separation: 0.3  # meters
       wheel_radius: 0.05     # meters
       publish_rate: 50.0     # Hz
       pose_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
       twist_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]

**Twist Mux Configuration**: `config/twist_mux.yaml`

Manages multiple velocity command sources:

.. code-block:: yaml

   twist_mux:
     ros__parameters:
       topics:
         navigation:
           topic: /nav_vel
           timeout: 0.5
           priority: 10
         teleop:
           topic: /teleop_vel
           timeout: 0.5
           priority: 100

Launch Files
~~~~~~~~~~~~

**Main Control Launch**: `launch/littlebot_control.launch.py`

Starts the complete control system:

* Controller manager
* Differential drive controller
* Joint state broadcaster
* Transform publishers

**Spawn Controller Launch**: `launch/spawn_diffdriver_controller.launch.py`

Utility launch file for loading controllers:

* Loads controller configuration
* Spawns and activates controllers
* Handles controller lifecycle

Usage
-----

Starting Controllers
~~~~~~~~~~~~~~~~~~~~

Launch the complete control system:

.. code-block:: bash

   # Start controllers with hardware
   ros2 launch littlebot_control littlebot_control.launch.py

   # Start controllers for simulation
   ros2 launch littlebot_control littlebot_control.launch.py \
       use_sim_time:=true

Managing Controllers
~~~~~~~~~~~~~~~~~~~~

Control individual controllers:

.. code-block:: bash

   # List available controllers
   ros2 control list_controllers

   # Load a controller
   ros2 control load_controller diff_drive_controller

   # Start a controller
   ros2 control switch_controllers \
       --activate diff_drive_controller

   # Stop a controller
   ros2 control switch_controllers \
       --deactivate diff_drive_controller

Sending Commands
~~~~~~~~~~~~~~~~

Control the robot programmatically:

.. code-block:: bash

   # Send velocity commands
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
       '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}'

   # Stop the robot
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
       '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

Visualization
-------------

RViz Configuration
~~~~~~~~~~~~~~~~~~

**File**: `rviz/diff_drive.rviz`

Pre-configured RViz setup including:

* Robot model display
* Joint state visualization
* Odometry path tracking
* Velocity command visualization
* Transform tree display

Launch RViz with configuration:

.. code-block:: bash

   # Launch RViz with robot visualization
   rviz2 -d src/littlebot_control/rviz/diff_drive.rviz

Monitoring Topics
~~~~~~~~~~~~~~~~~

Key topics for monitoring:

.. code-block:: bash

   # Monitor joint states
   ros2 topic echo /joint_states

   # Monitor odometry
   ros2 topic echo /odom

   # Monitor velocity commands
   ros2 topic echo /cmd_vel

   # Check controller status
   ros2 topic echo /controller_manager/robot_description

Configuration
-------------

Tuning Controller Parameters
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Wheel Parameters**:

.. code-block:: yaml

   diff_drive_controller:
     ros__parameters:
       wheel_separation: 0.3    # Distance between wheels (m)
       wheel_radius: 0.05       # Wheel radius (m)
       left_wheel_names: ["left_wheel_joint"]
       right_wheel_names: ["right_wheel_joint"]

**Control Rates**:

.. code-block:: yaml

   controller_manager:
     ros__parameters:
       update_rate: 100        # Controller update frequency (Hz)
   
   diff_drive_controller:
     ros__parameters:
       publish_rate: 50.0      # Odometry publish rate (Hz)

**Covariance Tuning**:

.. code-block:: yaml

   diff_drive_controller:
     ros__parameters:
       # Position covariance [x, y, z, roll, pitch, yaw]
       pose_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
       # Velocity covariance [vx, vy, vz, wx, wy, wz]
       twist_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]

Advanced Features
-----------------

Twist Multiplexer
~~~~~~~~~~~~~~~~~

The twist mux allows multiple sources to send velocity commands:

* **Navigation**: Autonomous navigation commands (priority 10)
* **Teleop**: Manual teleoperation commands (priority 100)
* **Safety**: Emergency stop commands (highest priority)

Priority system ensures safety and proper command arbitration.

Real-time Performance
~~~~~~~~~~~~~~~~~~~~~

Controller optimization:

* High-frequency control loops (100 Hz)
* Low-latency command processing
* Efficient memory usage
* Real-time scheduling support

Safety Features
~~~~~~~~~~~~~~~

Built-in safety mechanisms:

* Command timeout detection
* Velocity limiting
* Emergency stop capability
* Hardware fault detection

Troubleshooting
---------------

Common Issues
~~~~~~~~~~~~~

**Controllers Not Loading**:

.. code-block:: bash

   # Check controller manager status
   ros2 node info /controller_manager
   
   # Verify configuration file
   ros2 param list /controller_manager

**Incorrect Odometry**:

* Verify wheel parameters match physical robot
* Check encoder calibration
* Validate transform tree

**Commands Not Executed**:

.. code-block:: bash

   # Check if controller is active
   ros2 control list_controllers
   
   # Verify command topic
   ros2 topic info /cmd_vel
   
   # Test hardware interface
   ros2 control list_hardware_interfaces

Performance Tuning
~~~~~~~~~~~~~~~~~~~

Optimize controller performance:

.. code-block:: yaml

   # Increase update rates for better response
   controller_manager:
     ros__parameters:
       update_rate: 200  # Higher frequency

   # Reduce covariance for more confident odometry
   diff_drive_controller:
     ros__parameters:
       pose_covariance_diagonal: [0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.001]

Integration Examples
--------------------

With Navigation
~~~~~~~~~~~~~~~

.. code-block:: bash

   # Start control system
   ros2 launch littlebot_control littlebot_control.launch.py &
   
   # Start navigation (will send commands to /cmd_vel)
   ros2 launch littlebot_navigation littlebot_navigation.launch.py

With Teleop
~~~~~~~~~~~

.. code-block:: bash

   # Start control system
   ros2 launch littlebot_control littlebot_control.launch.py &
   
   # Start teleop (manual control)
   ros2 launch littlebot_teleop littlebot_teleop.launch.py

Programming Interface
--------------------

C++ Interface
~~~~~~~~~~~~~

Create custom controllers:

.. code-block:: cpp

   #include <controller_interface/controller_interface.hpp>
   
   class CustomController : public controller_interface::ControllerInterface
   {
   public:
     controller_interface::InterfaceConfiguration command_interface_configuration() const override;
     controller_interface::InterfaceConfiguration state_interface_configuration() const override;
     controller_interface::return_type update(
       const rclcpp::Time & time, const rclcpp::Duration & period) override;
   };

Python Interface
~~~~~~~~~~~~~~~~

Control via Python:

.. code-block:: python

   import rclpy
   from geometry_msgs.msg import Twist
   
   def send_velocity_command():
       node = rclpy.create_node('velocity_publisher')
       pub = node.create_publisher(Twist, '/cmd_vel', 10)
       
       twist = Twist()
       twist.linear.x = 0.5
       twist.angular.z = 0.2
       pub.publish(twist)

Dependencies
------------

**ROS2 Packages**:

* ``controller_manager``
* ``diff_drive_controller``
* ``joint_state_broadcaster``
* ``twist_mux``
* ``robot_state_publisher``

**System Dependencies**:

* ROS2 Control framework
* Real-time kernel (optional, for hard real-time)

API Reference
-------------

For detailed controller API documentation, see the ROS2 Control documentation and controller-specific references.