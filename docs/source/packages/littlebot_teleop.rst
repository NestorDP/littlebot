littlebot_teleop
===============

The `littlebot_teleop` package provides teleoperation (remote control) capabilities for manually controlling the LittleBot robot using various input devices.

Overview
--------

This package implements:

* Keyboard-based teleoperation
* Joystick/gamepad control support
* Twist message publishing for robot control
* Safety features and velocity limiting
* Multi-input device support
* Configurable control mappings

Components
----------

Teleoperation Nodes
~~~~~~~~~~~~~~~~~~~

**Keyboard Teleop Node**

Manual control using keyboard input:

* WASD or arrow key movement
* Adjustable velocity increments
* Emergency stop functionality
* Real-time velocity display

**Joystick Teleop Node**

Gamepad/joystick control:

* Analog stick movement control
* Button-based functions
* Deadzone handling
* Multiple joystick support

**Twist Keyboard Node**

Simple keyboard control interface:

* Direct velocity publishing
* Configurable key mappings
* Safety timeouts
* Debug output

Launch Files
~~~~~~~~~~~~

**Main Teleop Launch**: `launch/littlebot_teleop.launch.py`

Starts keyboard-based teleoperation:

* Keyboard input handler
* Velocity command publisher
* Safety monitoring
* User interface display

**Joystick Teleop Launch**: `launch/joystick_teleop.launch.py`

Gamepad/joystick control interface:

* Joystick driver initialization
* Button and axis mapping
* Velocity scaling configuration
* Emergency stop handling

Usage
-----

Keyboard Control
~~~~~~~~~~~~~~~~

Start keyboard teleoperation:

.. code-block:: bash

   # Launch keyboard teleop
   ros2 launch littlebot_teleop littlebot_teleop.launch.py

   # Launch with custom velocity limits
   ros2 launch littlebot_teleop littlebot_teleop.launch.py \
       max_linear_vel:=0.5 max_angular_vel:=1.0

**Control Keys**:

.. list-table:: Keyboard Controls
   :widths: 20 80
   :header-rows: 1

   * - Key
     - Action
   * - ``W`` / ``↑``
     - Move forward
   * - ``S`` / ``↓``
     - Move backward  
   * - ``A`` / ``←``
     - Turn left
   * - ``D`` / ``→``
     - Turn right
   * - ``Q``
     - Increase linear velocity
   * - ``Z``
     - Decrease linear velocity
   * - ``E``
     - Increase angular velocity
   * - ``C``
     - Decrease angular velocity
   * - ``Space``
     - Emergency stop
   * - ``R``
     - Reset to default velocities

**Usage Instructions**:

.. code-block:: text

   LittleBot Teleop Control
   ------------------------
   Moving around:
     W/↑ : Forward
     S/↓ : Backward
     A/← : Turn Left
     D/→ : Turn Right
     
   Speed Control:
     Q : Increase Linear Speed
     Z : Decrease Linear Speed  
     E : Increase Angular Speed
     C : Decrease Angular Speed
     
   Emergency:
     SPACE : Emergency Stop
     R : Reset to Default
     
   Current Speeds:
     Linear: 0.2 m/s
     Angular: 0.5 rad/s

Joystick Control
~~~~~~~~~~~~~~~~

Connect and configure joystick:

.. code-block:: bash

   # Check joystick connection
   ls /dev/input/js*
   jstest /dev/input/js0

   # Launch joystick teleop
   ros2 launch littlebot_teleop joystick_teleop.launch.py

   # Launch with custom joystick device
   ros2 launch littlebot_teleop joystick_teleop.launch.py \
       joy_dev:=/dev/input/js1

**Joystick Mapping** (Xbox/PlayStation controller):

.. list-table:: Joystick Controls
   :widths: 30 70
   :header-rows: 1

   * - Control
     - Action
   * - Left Stick Y-axis
     - Linear velocity (forward/backward)
   * - Left Stick X-axis
     - Angular velocity (turn left/right)
   * - Right Trigger (RT/R2)
     - Speed boost (hold for faster movement)
   * - Left Trigger (LT/L2)
     - Precision mode (hold for slower movement)
   * - A Button (X on PS)
     - Emergency stop
   * - B Button (Circle on PS)
     - Reset/resume normal operation
   * - Start/Options Button
     - Toggle teleop enable/disable

Configuration
-------------

Velocity Limits
~~~~~~~~~~~~~~~

**Default Configuration**:

.. code-block:: yaml

   teleop_node:
     ros__parameters:
       # Linear velocity limits (m/s)
       max_linear_velocity: 0.5
       min_linear_velocity: 0.1
       linear_velocity_step: 0.05
       
       # Angular velocity limits (rad/s)  
       max_angular_velocity: 1.0
       min_angular_velocity: 0.2
       angular_velocity_step: 0.1
       
       # Safety timeouts
       timeout: 0.5  # Stop robot if no command received
       
       # Key repeat settings
       key_timeout: 0.1
       repeat_rate: 10.0

**Custom Velocity Configuration**:

.. code-block:: yaml

   # High-speed configuration
   teleop_node:
     ros__parameters:
       max_linear_velocity: 1.0   # 1 m/s max
       max_angular_velocity: 2.0  # 2 rad/s max
       
   # Precision configuration  
   teleop_node:
     ros__parameters:
       max_linear_velocity: 0.2   # Slow and precise
       max_angular_velocity: 0.3
       linear_velocity_step: 0.01
       angular_velocity_step: 0.02

Joystick Configuration
~~~~~~~~~~~~~~~~~~~~~~

**Axis and Button Mapping**:

.. code-block:: yaml

   joy_teleop:
     ros__parameters:
       # Axis mappings (0-based index)
       axis_linear: 1        # Left stick Y-axis
       axis_angular: 0       # Left stick X-axis
       
       # Button mappings (0-based index)
       enable_button: 0      # A button (continuous hold)
       enable_turbo_button: 5 # Right trigger
       
       # Scaling factors
       scale_linear: 0.5     # Max linear velocity multiplier
       scale_angular: 1.0    # Max angular velocity multiplier
       scale_linear_turbo: 1.0  # Turbo mode multiplier
       scale_angular_turbo: 1.5
       
       # Deadzone settings
       deadzone: 0.1         # Ignore small stick movements

**Multi-Joystick Support**:

.. code-block:: yaml

   # Player 1 controller
   joy_teleop_1:
     ros__parameters:
       joy_topic: /joy1
       cmd_vel_topic: /robot1/cmd_vel
       
   # Player 2 controller  
   joy_teleop_2:
     ros__parameters:
       joy_topic: /joy2
       cmd_vel_topic: /robot2/cmd_vel

Safety Features
---------------

Emergency Stop
~~~~~~~~~~~~~~

Multiple emergency stop mechanisms:

.. code-block:: python

   # Keyboard emergency stop
   if key == 'space':
       self.emergency_stop()
       
   # Joystick emergency stop
   if joy_msg.buttons[EMERGENCY_BUTTON]:
       self.emergency_stop()
       
   # Timeout-based stop
   if time.time() - self.last_command_time > self.timeout:
       self.emergency_stop()

   def emergency_stop(self):
       """Immediately stop the robot and clear all velocities"""
       stop_msg = Twist()
       stop_msg.linear.x = 0.0
       stop_msg.angular.z = 0.0
       self.cmd_vel_pub.publish(stop_msg)

Velocity Limiting
~~~~~~~~~~~~~~~~~

Enforce safe velocity bounds:

.. code-block:: python

   def limit_velocity(self, linear, angular):
       """Apply velocity limits and safety checks"""
       # Limit linear velocity
       linear = max(min(linear, self.max_linear_vel), -self.max_linear_vel)
       
       # Limit angular velocity  
       angular = max(min(angular, self.max_angular_vel), -self.max_angular_vel)
       
       # Apply acceleration limits
       linear = self.apply_acceleration_limit(linear, self.last_linear_vel)
       angular = self.apply_acceleration_limit(angular, self.last_angular_vel)
       
       return linear, angular

Deadman Switch
~~~~~~~~~~~~~~

Require continuous input for operation:

.. code-block:: python

   def joystick_callback(self, joy_msg):
       """Process joystick input with deadman switch"""
       if not joy_msg.buttons[self.enable_button]:
           # Deadman switch not pressed - stop robot
           self.publish_stop_command()
           return
           
       # Deadman switch active - process movement commands
       linear = joy_msg.axes[self.axis_linear] * self.scale_linear
       angular = joy_msg.axes[self.axis_angular] * self.scale_angular
       
       self.publish_velocity_command(linear, angular)

Advanced Features
-----------------

Multi-Input Priority
~~~~~~~~~~~~~~~~~~~~

Handle multiple control sources:

.. code-block:: yaml

   # Twist multiplexer configuration
   twist_mux:
     ros__parameters:
       topics:
         joystick:
           topic: /joy_vel
           timeout: 0.5
           priority: 100    # Highest priority
         keyboard:
           topic: /key_vel  
           timeout: 1.0
           priority: 90     # Medium priority
         navigation:
           topic: /nav_vel
           timeout: 2.0  
           priority: 10     # Lowest priority

Custom Key Bindings
~~~~~~~~~~~~~~~~~~~

Create custom control schemes:

.. code-block:: python

   class CustomTeleopNode(Node):
       def __init__(self):
           super().__init__('custom_teleop')
           
           # Custom key mappings
           self.key_bindings = {
               # WASD movement
               'w': (1.0, 0.0),   # Forward
               's': (-1.0, 0.0),  # Backward
               'a': (0.0, 1.0),   # Left
               'd': (0.0, -1.0),  # Right
               
               # Diagonal movement
               'q': (1.0, 1.0),   # Forward-left
               'e': (1.0, -1.0),  # Forward-right
               'z': (-1.0, 1.0),  # Backward-left  
               'c': (-1.0, -1.0), # Backward-right
               
               # Special functions
               'r': 'reset',
               ' ': 'stop',
               'h': 'help'
           }

Voice Control Integration
~~~~~~~~~~~~~~~~~~~~~~~~~

Add voice command support:

.. code-block:: python

   def voice_command_callback(self, msg):
       """Process voice commands"""
       command = msg.data.lower()
       
       if 'forward' in command:
           self.publish_velocity(0.3, 0.0)
       elif 'backward' in command:
           self.publish_velocity(-0.3, 0.0)
       elif 'left' in command:
           self.publish_velocity(0.0, 0.5)
       elif 'right' in command:
           self.publish_velocity(0.0, -0.5)
       elif 'stop' in command:
           self.publish_velocity(0.0, 0.0)

Programming Interface
--------------------

Python Teleop Node
~~~~~~~~~~~~~~~~~~~

Create custom teleop nodes:

.. code-block:: python

   import rclpy
   from rclpy.node import Node
   from geometry_msgs.msg import Twist
   from sensor_msgs.msg import Joy
   import sys, select, termios, tty

   class TeleopNode(Node):
       def __init__(self):
           super().__init__('teleop_node')
           
           # Publishers
           self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
           
           # Subscribers
           self.joy_sub = self.create_subscription(
               Joy, '/joy', self.joy_callback, 10)
           
           # Parameters
           self.declare_parameter('max_linear_vel', 0.5)
           self.declare_parameter('max_angular_vel', 1.0)
           
           self.max_linear_vel = self.get_parameter('max_linear_vel').value
           self.max_angular_vel = self.get_parameter('max_angular_vel').value
           
           # Timer for keyboard input
           self.timer = self.create_timer(0.1, self.keyboard_callback)
           
       def keyboard_callback(self):
           """Handle keyboard input"""
           if self.is_key_pressed():
               key = self.get_key()
               self.process_key(key)
               
       def joy_callback(self, msg):
           """Handle joystick input"""
           if msg.buttons[0]:  # Enable button pressed
               linear = msg.axes[1] * self.max_linear_vel
               angular = msg.axes[0] * self.max_angular_vel
               self.publish_velocity(linear, angular)

C++ Teleop Node
~~~~~~~~~~~~~~~

.. code-block:: cpp

   #include <rclcpp/rclcpp.hpp>
   #include <geometry_msgs/msg/twist.hpp>
   #include <sensor_msgs/msg/joy.hpp>

   class TeleopNode : public rclcpp::Node
   {
   public:
     TeleopNode() : Node("teleop_node")
     {
       // Publishers
       cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
       
       // Subscribers  
       joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
         "/joy", 10, std::bind(&TeleopNode::joy_callback, this, std::placeholders::_1));
         
       // Parameters
       this->declare_parameter("max_linear_vel", 0.5);
       this->declare_parameter("max_angular_vel", 1.0);
       
       max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
       max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
     }
     
   private:
     void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
     {
       if (msg->buttons[0]) // Enable button
       {
         auto twist = geometry_msgs::msg::Twist();
         twist.linear.x = msg->axes[1] * max_linear_vel_;
         twist.angular.z = msg->axes[0] * max_angular_vel_;
         cmd_vel_pub_->publish(twist);
       }
     }
     
     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
     rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
     double max_linear_vel_, max_angular_vel_;
   };

Troubleshooting
---------------

Common Issues
~~~~~~~~~~~~~

**Keyboard Not Responding**:

.. code-block:: bash

   # Check terminal settings
   stty -echo
   
   # Reset terminal
   reset
   
   # Run in correct terminal
   gnome-terminal -- ros2 launch littlebot_teleop littlebot_teleop.launch.py

**Joystick Not Detected**:

.. code-block:: bash

   # Check joystick connection
   ls /dev/input/js*
   
   # Test joystick
   jstest /dev/input/js0
   
   # Check permissions
   sudo chmod 666 /dev/input/js0
   
   # Install joystick tools
   sudo apt install joystick

**Robot Not Moving**:

.. code-block:: bash

   # Check velocity commands
   ros2 topic echo /cmd_vel
   
   # Check robot controllers
   ros2 control list_controllers
   
   # Verify topic connections
   ros2 topic info /cmd_vel

Performance Optimization
~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: yaml

   # Reduce publishing rate for efficiency
   teleop_node:
     ros__parameters:
       publish_rate: 20.0  # Hz
       
   # Optimize deadzone settings
   joy_teleop:
     ros__parameters:
       deadzone: 0.05  # Smaller deadzone for responsiveness

Integration Examples
--------------------

Complete Teleop System
~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # Start robot
   ros2 launch littlebot_bringup littlebot_bringup.launch.py &
   
   # Start teleop
   ros2 launch littlebot_teleop littlebot_teleop.launch.py

With Navigation System
~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # Start complete system
   ros2 launch littlebot_bringup littlebot_bringup.launch.py &
   ros2 launch littlebot_navigation littlebot_navigation.launch.py &
   
   # Teleop will have lower priority than navigation
   ros2 launch littlebot_teleop littlebot_teleop.launch.py

Dependencies
------------

**ROS2 Packages**:

* ``geometry_msgs``
* ``sensor_msgs``
* ``joy``
* ``teleop_twist_keyboard``
* ``teleop_twist_joy``

**System Dependencies**:

* Joystick drivers
* Input device permissions
* Terminal access

API Reference
-------------

For detailed teleop API documentation, see the teleop_twist_keyboard and teleop_twist_joy package documentation.