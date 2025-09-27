littlebot_rqt_plugin
==================

The `littlebot_rqt_plugin` package provides a custom RQT GUI plugin for monitoring, controlling, and debugging the LittleBot robot system.

Overview
--------

This package implements:

* Real-time robot monitoring interface
* Manual control capabilities
* System diagnostics and status display
* Log message visualization
* Parameter adjustment interface
* Custom Qt-based GUI components

Components
----------

GUI Plugin Structure
~~~~~~~~~~~~~~~~~~~~

**Main Plugin Class**: `src/littlebot_rqt_plugin.cpp`

Core plugin implementation:

* RQT framework integration
* ROS2 node management
* Message publishing/subscribing
* GUI event handling

**GUI Implementation**: `src/littlebot_gui.cpp`

User interface implementation:

* Qt widget management
* Control button handling
* Real-time data display
* User input processing

**UI Definition**: `ui/littlebot_gui.ui`

Qt Designer interface layout:

* Visual widget arrangement
* Control element positioning
* Style and appearance settings
* Layout management

Plugin Registration
~~~~~~~~~~~~~~~~~~~

**Plugin Configuration**: `plugin.xml`

RQT plugin registration:

.. code-block:: xml

   <library path="littlebot_rqt_plugin">
     <class name="littlebot_rqt_plugin/LittlebotRqtPlugin" 
            type="littlebot_rqt_plugin::LittlebotRqtPlugin" 
            base_class_type="rqt_gui_cpp::Plugin">
       <description>
         A GUI plugin for monitoring and controlling LittleBot robot
       </description>
       <qtgui>
         <group>
           <label>Inspection</label>
           <statustip>Inspect the robot</statustip>
           <icon type="theme">folder</icon>      
         </group>
         <label>Littlebot GUI</label>
         <icon type="theme">system-help</icon>
         <statustip>GUI to control littlebot robot.</statustip>
       </qtgui>
     </class>
   </library>

Launch Files
~~~~~~~~~~~~

**Plugin Launch**: `launch/littlebot_rqt_plugin.launch.py`

Starts the RQT plugin:

* Plugin initialization
* ROS2 node setup
* GUI display launch
* Parameter configuration

Usage
-----

Starting the Plugin
~~~~~~~~~~~~~~~~~~~

Launch the RQT plugin:

.. code-block:: bash

   # Launch standalone plugin
   ros2 launch littlebot_rqt_plugin littlebot_rqt_plugin.launch.py

   # Launch within RQT framework
   rqt --standalone littlebot_rqt_plugin

   # Force plugin discovery (if needed)
   rqt --force-discover

   # Start RQT and manually load plugin
   rqt
   # Then: Plugins → Inspection → Littlebot GUI

Interface Features
~~~~~~~~~~~~~~~~~~

**Control Panel**:

* **Emergency Stop**: Immediate robot halt
* **Reset**: Clear error states and restart
* **Speed Control**: Adjust maximum velocities
* **Mode Selection**: Switch between manual/auto modes

**Status Display**:

* **Connection Status**: Hardware communication state
* **Battery Level**: Power remaining (if available)
* **System Health**: Overall robot status
* **Error Messages**: Real-time error reporting

**Data Visualization**:

* **Sensor Readings**: Live sensor data display
* **Velocity Commands**: Current movement commands
* **Odometry**: Position and orientation data
* **Joint States**: Motor positions and velocities

GUI Components
--------------

Control Widgets
~~~~~~~~~~~~~~~

**Command Buttons**:

.. code-block:: cpp

   // Emergency stop button
   connect(ui_.emergency_stop_button, &QPushButton::clicked, 
           this, &LittlebotGui::onEmergencyStop);

   // Reset button  
   connect(ui_.reset_button, &QPushButton::clicked,
           this, &LittlebotGui::onReset);

   // Manual control buttons
   connect(ui_.forward_button, &QPushButton::pressed,
           this, &LittlebotGui::onForwardPressed);
   connect(ui_.forward_button, &QPushButton::released,
           this, &LittlebotGui::onButtonReleased);

**Speed Control Sliders**:

.. code-block:: cpp

   // Linear velocity slider
   connect(ui_.linear_velocity_slider, &QSlider::valueChanged,
           this, &LittlebotGui::onLinearVelocityChanged);

   // Angular velocity slider
   connect(ui_.angular_velocity_slider, &QSlider::valueChanged,
           this, &LittlebotGui::onAngularVelocityChanged);

Display Widgets
~~~~~~~~~~~~~~~

**Status Indicators**:

.. code-block:: cpp

   // Update connection status
   void LittlebotGui::updateConnectionStatus(bool connected)
   {
       if (connected) {
           ui_.connection_status_label->setText("Connected");
           ui_.connection_status_label->setStyleSheet("color: green;");
       } else {
           ui_.connection_status_label->setText("Disconnected");
           ui_.connection_status_label->setStyleSheet("color: red;");
       }
   }

**Data Display**:

.. code-block:: cpp

   // Update sensor data display
   void LittlebotGui::updateSensorData(const SensorData& data)
   {
       ui_.battery_voltage_label->setText(
           QString("Battery: %1V").arg(data.battery_voltage, 0, 'f', 2));
       
       ui_.temperature_label->setText(
           QString("Temp: %1°C").arg(data.temperature, 0, 'f', 1));
       
       ui_.distance_label->setText(
           QString("Distance: %1m").arg(data.ultrasonic_distance, 0, 'f', 2));
   }

**Log Display**:

.. code-block:: cpp

   // Add log message with timestamp
   void LittlebotGui::addLogMessage(const std::string& message, 
                                   const std::string& level)
   {
       QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss");
       QString formatted_msg = QString("[%1] %2: %3")
           .arg(timestamp)
           .arg(QString::fromStdString(level))
           .arg(QString::fromStdString(message));
       
       ui_.log_text_browser->append(formatted_msg);
       
       // Auto-scroll to bottom
       QScrollBar* scrollbar = ui_.log_text_browser->verticalScrollBar();
       scrollbar->setValue(scrollbar->maximum());
   }

ROS2 Integration
----------------

Node Communication
~~~~~~~~~~~~~~~~~~

**Publisher Setup**:

.. code-block:: cpp

   void LittlebotRqtPlugin::createPublishers()
   {
       // Velocity command publisher
       cmd_vel_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>(
           "/cmd_vel", 10);
       
       // Robot command publisher
       robot_command_publisher_ = node_->create_publisher<std_msgs::msg::String>(
           "/robot_command", 10);
   }

**Subscriber Setup**:

.. code-block:: cpp

   void LittlebotRqtPlugin::createSubscribers()
   {
       // Joint state subscriber
       joint_state_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
           "/joint_states", 10,
           std::bind(&LittlebotRqtPlugin::jointStateCallback, this, std::placeholders::_1));
       
       // Odometry subscriber
       odom_subscriber_ = node_->create_subscription<nav_msgs::msg::Odometry>(
           "/odom", 10,
           std::bind(&LittlebotRqtPlugin::odometryCallback, this, std::placeholders::_1));
       
       // System status subscriber
       status_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
           "/robot_status", 10,
           std::bind(&LittlebotRqtPlugin::statusCallback, this, std::placeholders::_1));
   }

Message Handling
~~~~~~~~~~~~~~~~

**Velocity Command Publishing**:

.. code-block:: cpp

   void LittlebotRqtPlugin::publishVelocityCommand(double linear, double angular)
   {
       auto twist_msg = geometry_msgs::msg::Twist();
       twist_msg.linear.x = linear;
       twist_msg.angular.z = angular;
       
       cmd_vel_publisher_->publish(twist_msg);
       
       // Update GUI display
       emit velocityCommandSent(linear, angular);
   }

**Status Message Processing**:

.. code-block:: cpp

   void LittlebotRqtPlugin::statusCallback(const std_msgs::msg::String::SharedPtr msg)
   {
       // Parse status message
       std::string status = msg->data;
       
       // Update GUI on main thread
       QMetaObject::invokeMethod(widget_, "updateStatus", 
           Qt::QueuedConnection,
           Q_ARG(QString, QString::fromStdString(status)));
   }

Advanced Features
-----------------

Parameter Interface
~~~~~~~~~~~~~~~~~~~

**Dynamic Parameter Control**:

.. code-block:: cpp

   // Parameter client setup
   void LittlebotRqtPlugin::setupParameterInterface()
   {
       parameter_client_ = std::make_shared<rclcpp::SyncParametersClient>(
           node_, "/diff_drive_controller");
       
       // Update parameter values
       connect(ui_.max_velocity_spinbox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
               this, &LittlebotRqtPlugin::updateMaxVelocityParameter);
   }

   void LittlebotRqtPlugin::updateMaxVelocityParameter(double value)
   {
       parameter_client_->set_parameters({
           rclcpp::Parameter("max_velocity", value)
       });
   }

Diagnostic Interface
~~~~~~~~~~~~~~~~~~~~

**System Health Monitoring**:

.. code-block:: cpp

   void LittlebotRqtPlugin::updateSystemDiagnostics()
   {
       // Check node status
       auto node_names = node_->get_node_names();
       bool controller_running = std::find(node_names.begin(), node_names.end(), 
           "/controller_manager") != node_names.end();
       
       // Update diagnostic display
       widget_->updateDiagnostic("Controller Manager", 
           controller_running ? "Running" : "Stopped");
       
       // Check topic health
       auto topic_names = node_->get_topic_names_and_types();
       bool cmd_vel_available = topic_names.find("/cmd_vel") != topic_names.end();
       
       widget_->updateDiagnostic("Command Topic", 
           cmd_vel_available ? "Available" : "Missing");
   }

Data Logging
~~~~~~~~~~~~

**Log Message Management**:

.. code-block:: cpp

   void LittlebotRqtPlugin::setupLogging()
   {
       // Log file setup
       log_file_.setFileName(QDir::homePath() + "/littlebot_logs.txt");
       log_file_.open(QIODevice::WriteOnly | QIODevice::Append);
       log_stream_.setDevice(&log_file_);
       
       // Connect log signal
       connect(this, &LittlebotRqtPlugin::logMessageReceived,
               widget_, &LittlebotGui::displayLogMessage);
   }

   void LittlebotRqtPlugin::logMessage(const std::string& message, 
                                      const std::string& level)
   {
       // Write to file
       QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
       log_stream_ << timestamp << " [" << QString::fromStdString(level) 
                   << "] " << QString::fromStdString(message) << Qt::endl;
       
       // Display in GUI
       emit logMessageReceived(message, level);
   }

Customization
-------------

Adding New Controls
~~~~~~~~~~~~~~~~~~~

**Custom Widget Integration**:

.. code-block:: cpp

   // Add custom joystick widget
   class JoystickWidget : public QWidget
   {
   public:
       JoystickWidget(QWidget* parent = nullptr);
       
   protected:
       void paintEvent(QPaintEvent* event) override;
       void mousePressEvent(QMouseEvent* event) override;
       void mouseMoveEvent(QMouseEvent* event) override;
       
   signals:
       void joystickMoved(double x, double y);
       
   private:
       QPoint center_;
       QPoint current_pos_;
       int radius_;
   };

**Integration with Main GUI**:

.. code-block:: cpp

   void LittlebotGui::setupCustomWidgets()
   {
       // Add joystick widget
       joystick_widget_ = new JoystickWidget(this);
       ui_.control_layout->addWidget(joystick_widget_);
       
       // Connect signals
       connect(joystick_widget_, &JoystickWidget::joystickMoved,
               this, &LittlebotGui::onJoystickMoved);
   }

Custom Themes
~~~~~~~~~~~~~

**GUI Styling**:

.. code-block:: cpp

   void LittlebotGui::applyCustomTheme()
   {
       QString stylesheet = 
           "QMainWindow { background-color: #2b2b2b; color: #ffffff; }"
           "QPushButton { "
           "  background-color: #3c3c3c; "
           "  border: 2px solid #555555; "
           "  border-radius: 5px; "
           "  padding: 5px; "
           "}"
           "QPushButton:hover { background-color: #4c4c4c; }"
           "QPushButton:pressed { background-color: #1c1c1c; }";
       
       setStyleSheet(stylesheet);
   }

Configuration Files
~~~~~~~~~~~~~~~~~~~

**Plugin Settings**:

.. code-block:: yaml

   # Plugin configuration
   littlebot_rqt_plugin:
     ros__parameters:
       # Update rates
       status_update_rate: 10.0    # Hz
       gui_refresh_rate: 30.0      # Hz
       
       # Topic names
       cmd_vel_topic: "/cmd_vel"
       status_topic: "/robot_status"
       joint_states_topic: "/joint_states"
       
       # GUI settings
       window_title: "LittleBot Control"
       enable_logging: true
       log_file_path: "~/littlebot_logs.txt"
       
       # Control limits
       max_linear_velocity: 1.0    # m/s
       max_angular_velocity: 2.0   # rad/s

Building and Installation
------------------------

Build Requirements
~~~~~~~~~~~~~~~~~~

**Dependencies**:

.. code-block:: bash

   # Install Qt development packages
   sudo apt install qtbase5-dev qttools5-dev-tools
   
   # Install RQT development packages
   sudo apt install ros-humble-rqt-gui-cpp ros-humble-qt-gui-cpp

**CMake Configuration**:

.. code-block:: cmake

   # Find required packages
   find_package(rclcpp REQUIRED)
   find_package(rqt_gui_cpp REQUIRED)
   find_package(qt_gui_cpp REQUIRED)
   find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
   
   # Generate UI headers
   qt5_wrap_ui(UIC_FILES ui/littlebot_gui.ui)
   
   # Build plugin library
   add_library(${PROJECT_NAME} SHARED
     src/littlebot_rqt_plugin.cpp
     src/littlebot_gui.cpp
     ${UIC_FILES}
   )

Plugin Installation
~~~~~~~~~~~~~~~~~~~

**Package Installation**:

.. code-block:: bash

   # Build the plugin
   cd ~/littlebot_ws
   colcon build --packages-select littlebot_rqt_plugin
   
   # Source the workspace
   source install/setup.bash
   
   # Verify plugin installation
   rqt --list-plugins | grep littlebot

Troubleshooting
---------------

Common Issues
~~~~~~~~~~~~~

**Plugin Not Found**:

.. code-block:: bash

   # Force plugin discovery
   rqt --force-discover
   
   # Clear RQT cache
   rm -rf ~/.config/ros.org/rqt_gui.ini
   
   # Check plugin registration
   ros2 pkg xml littlebot_rqt_plugin

**GUI Display Issues**:

.. code-block:: bash

   # Check Qt installation
   qmake --version
   
   # Verify display environment
   echo $DISPLAY
   
   # Test basic Qt functionality
   qtdiag

**Communication Errors**:

.. code-block:: bash

   # Check ROS2 node status
   ros2 node list | grep rqt
   
   # Verify topic connections
   ros2 topic info /cmd_vel
   
   # Monitor message flow
   ros2 topic echo /robot_status

Performance Optimization
~~~~~~~~~~~~~~~~~~~~~~~~

**GUI Responsiveness**:

.. code-block:: cpp

   // Limit update frequency
   void LittlebotRqtPlugin::setupTimers()
   {
       // Status update timer (10 Hz)
       status_timer_ = new QTimer(this);
       connect(status_timer_, &QTimer::timeout, 
               this, &LittlebotRqtPlugin::updateStatus);
       status_timer_->start(100); // 100ms
       
       // GUI refresh timer (30 Hz)  
       gui_timer_ = new QTimer(this);
       connect(gui_timer_, &QTimer::timeout,
               widget_, &LittlebotGui::refreshDisplay);
       gui_timer_->start(33); // 33ms
   }

Dependencies
------------

**ROS2 Packages**:

* ``rqt_gui_cpp``
* ``qt_gui_cpp``
* ``rclcpp``
* ``geometry_msgs``
* ``sensor_msgs``
* ``std_msgs``

**System Dependencies**:

* Qt5 development libraries
* RQT framework
* C++ compiler with C++17 support

API Reference
-------------

For detailed RQT plugin development and Qt integration, see the RQT documentation and Qt API reference guides.