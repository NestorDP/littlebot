littlebot_description
===================

The `littlebot_description` package contains the robot model definition, including URDF/Xacro files, meshes, materials, and visualization configurations.

Overview
--------

This package provides:

* Complete robot model in URDF/Xacro format
* 3D meshes and visual materials
* Robot description launch files
* RViz configuration for visualization
* Gazebo integration files
* ROS2 Control hardware descriptions

Components
----------

Robot Model Files
~~~~~~~~~~~~~~~~~

**Main URDF/Xacro Files**:

* ``urdf/littlebot.urdf.xacro``: Main robot description
* ``gazebo/littlebot.gazebo.xacro``: Gazebo-specific plugins and sensors
* ``ros2_control/``: Hardware interface descriptions

**Structure**:

.. code-block:: xml

   <!-- Main robot structure -->
   <robot name="littlebot" xmlns:xacro="http://www.ros.org/wiki/xacro">
     
     <!-- Robot properties -->
     <xacro:property name="base_width" value="0.3"/>
     <xacro:property name="base_length" value="0.4"/>
     <xacro:property name="wheel_radius" value="0.05"/>
     
     <!-- Include components -->
     <xacro:include filename="$(find littlebot_description)/urdf/base.xacro"/>
     <xacro:include filename="$(find littlebot_description)/urdf/wheels.xacro"/>
     <xacro:include filename="$(find littlebot_description)/urdf/sensors.xacro"/>
   </robot>

3D Assets
~~~~~~~~~

**Mesh Directory**: `meshes/`

Contains CAD-based 3D models:

* ``littlebot.dae``: Main robot body
* ``roda.dae``: Wheel meshes
* ``Caster_Wheel_Plate.dae``: Caster wheel assembly
* ``HC-SR04.dae``: Ultrasonic sensor model
* ``MPU_6050.dae``: IMU sensor model

**Material Properties**:

* Realistic visual appearance
* Physics collision meshes
* Optimized for real-time rendering

Launch Files
~~~~~~~~~~~~

**Robot Description Launch**: `launch/littlebot_description.launch.py`

Launches robot model visualization:

* Loads robot description parameter
* Starts ``robot_state_publisher``
* Launches RViz with robot model
* Publishes static transforms

**Simulation Description Launch**: `launch/littlebot_description_sim.launch.py`

Specialized for simulation environments:

* Includes Gazebo plugins
* Configures simulation parameters
* Sets up sensor simulation

Usage
-----

Viewing Robot Model
~~~~~~~~~~~~~~~~~~~

Visualize the robot in RViz:

.. code-block:: bash

   # Launch robot visualization
   ros2 launch littlebot_description littlebot_description.launch.py

   # Launch with custom configuration
   ros2 launch littlebot_description littlebot_description.launch.py \
       rviz_config:=custom_config.rviz

   # Launch for simulation
   ros2 launch littlebot_description littlebot_description_sim.launch.py

Robot Model Parameters
~~~~~~~~~~~~~~~~~~~~~~

Available launch parameters:

.. list-table:: Launch Parameters
   :widths: 25 25 50
   :header-rows: 1

   * - Parameter
     - Default Value
     - Description
   * - ``use_sim_time``
     - ``false``
     - Use simulation time
   * - ``robot_description_file``
     - ``littlebot.urdf.xacro``
     - Robot model file
   * - ``rviz_config``
     - ``littlebot_description.rviz``
     - RViz configuration file
   * - ``use_gui``
     - ``true``
     - Show joint state publisher GUI

Working with URDF
~~~~~~~~~~~~~~~~~~

Generate and validate URDF:

.. code-block:: bash

   # Generate URDF from Xacro
   xacro src/littlebot_description/urdf/littlebot.urdf.xacro > robot.urdf

   # Validate URDF structure
   check_urdf robot.urdf

   # View URDF graphically
   urdf_to_graphiz robot.urdf

Robot Structure
---------------

Physical Properties
~~~~~~~~~~~~~~~~~~~

**Base Dimensions**:

.. code-block:: xml

   <xacro:property name="base_width" value="0.3"/>      <!-- 30 cm -->
   <xacro:property name="base_length" value="0.4"/>     <!-- 40 cm -->
   <xacro:property name="base_height" value="0.1"/>     <!-- 10 cm -->

**Wheel Specifications**:

.. code-block:: xml

   <xacro:property name="wheel_radius" value="0.05"/>   <!-- 5 cm -->
   <xacro:property name="wheel_width" value="0.02"/>    <!-- 2 cm -->
   <xacro:property name="wheel_separation" value="0.3"/> <!-- 30 cm -->

**Mass Properties**:

.. code-block:: xml

   <xacro:property name="base_mass" value="2.0"/>       <!-- 2 kg -->
   <xacro:property name="wheel_mass" value="0.1"/>      <!-- 100 g -->
   <xacro:property name="caster_mass" value="0.05"/>    <!-- 50 g -->

Joint Configuration
~~~~~~~~~~~~~~~~~~~

**Drive Wheels**:

.. code-block:: xml

   <joint name="left_wheel_joint" type="continuous">
     <parent link="base_link"/>
     <child link="left_wheel_link"/>
     <origin xyz="0 0.15 0" rpy="0 0 0"/>
     <axis xyz="0 1 0"/>
   </joint>

**Caster Wheel**:

.. code-block:: xml

   <joint name="caster_joint" type="fixed">
     <parent link="base_link"/>
     <child link="caster_link"/>
     <origin xyz="-0.15 0 -0.05" rpy="0 0 0"/>
   </joint>

Sensors and Hardware
--------------------

Sensor Integration
~~~~~~~~~~~~~~~~~~

**IMU Sensor**:

.. code-block:: xml

   <link name="imu_link">
     <visual>
       <geometry>
         <mesh filename="package://littlebot_description/meshes/MPU_6050.dae"/>
       </geometry>
     </visual>
   </link>

**Ultrasonic Sensors**:

.. code-block:: xml

   <link name="ultrasonic_front_link">
     <visual>
       <geometry>
         <mesh filename="package://littlebot_description/meshes/HC-SR04.dae"/>
       </geometry>
     </visual>
   </link>

ROS2 Control Integration
~~~~~~~~~~~~~~~~~~~~~~~~

Hardware interface definition:

.. code-block:: xml

   <ros2_control name="LittlebotHardware" type="system">
     <hardware>
       <plugin>littlebot_base/LittlebotHardwareComponent</plugin>
       <param name="serial_port">/dev/ttyUSB0</param>
       <param name="baud_rate">115200</param>
     </hardware>
     
     <joint name="left_wheel_joint">
       <command_interface name="velocity"/>
       <state_interface name="position"/>
       <state_interface name="velocity"/>
     </joint>
     
     <joint name="right_wheel_joint">
       <command_interface name="velocity"/>
       <state_interface name="position"/>
       <state_interface name="velocity"/>
     </joint>
   </ros2_control>

Gazebo Integration
------------------

Simulation Plugins
~~~~~~~~~~~~~~~~~~

**Physics Properties**:

.. code-block:: xml

   <gazebo reference="base_link">
     <material>Gazebo/Blue</material>
     <mu1>0.2</mu1>
     <mu2>0.2</mu2>
     <selfCollide>true</selfCollide>
   </gazebo>

**Differential Drive Plugin**:

.. code-block:: xml

   <gazebo>
     <plugin name="differential_drive_controller" 
             filename="libgazebo_ros_diff_drive.so">
       <ros>
         <namespace>/</namespace>
       </ros>
       <left_joint>left_wheel_joint</left_joint>
       <right_joint>right_wheel_joint</right_joint>
       <wheel_separation>0.3</wheel_separation>
       <wheel_diameter>0.1</wheel_diameter>
     </plugin>
   </gazebo>

Sensor Simulation
~~~~~~~~~~~~~~~~~

**IMU Plugin**:

.. code-block:: xml

   <gazebo reference="imu_link">
     <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
       <ros>
         <namespace>/</namespace>
         <remapping>~/out:=imu</remapping>
       </ros>
       <update_rate>100</update_rate>
     </plugin>
   </gazebo>

Visualization
-------------

RViz Configuration
~~~~~~~~~~~~~~~~~~

**File**: `config/littlebot_description.rviz`

Pre-configured displays:

* **RobotModel**: 3D robot visualization
* **TF**: Transform frame display
* **Axes**: Coordinate frame indicators
* **Grid**: Reference grid
* **LaserScan**: Sensor data (when available)

Custom Materials
~~~~~~~~~~~~~~~~

Enhanced visual appearance:

.. code-block:: xml

   <material name="blue">
     <color rgba="0.2 0.2 1.0 1.0"/>
   </material>
   
   <material name="black">
     <color rgba="0.1 0.1 0.1 1.0"/>
   </material>
   
   <material name="white">
     <color rgba="1.0 1.0 1.0 1.0"/>
   </material>

Customization
-------------

Modifying Robot Model
~~~~~~~~~~~~~~~~~~~~~

**Adding New Sensors**:

1. Create sensor link and joint
2. Add visual and collision geometry
3. Include sensor plugin (for simulation)
4. Update launch files if needed

**Changing Dimensions**:

.. code-block:: xml

   <!-- Edit properties in main xacro file -->
   <xacro:property name="base_width" value="0.35"/>  <!-- Wider base -->
   <xacro:property name="wheel_radius" value="0.06"/> <!-- Larger wheels -->

**Custom Meshes**:

1. Place new mesh files in ``meshes/`` directory
2. Update visual geometry references
3. Ensure proper scale and orientation

Testing and Validation
----------------------

Model Validation
~~~~~~~~~~~~~~~~

.. code-block:: bash

   # Check URDF syntax
   check_urdf $(xacro src/littlebot_description/urdf/littlebot.urdf.xacro)

   # Validate joint limits
   ros2 run robot_state_publisher robot_state_publisher \
     --ros-args -p robot_description:="$(xacro src/littlebot_description/urdf/littlebot.urdf.xacro)"

Visual Inspection
~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # Launch with joint state publisher GUI
   ros2 launch littlebot_description littlebot_description.launch.py use_gui:=true

   # Check transforms
   ros2 run tf2_tools view_frames.py
   evince frames.pdf

Troubleshooting
---------------

Common Issues
~~~~~~~~~~~~~

**URDF Parse Errors**:

* Check XML syntax and structure
* Verify file paths for meshes and includes
* Validate property definitions

**Missing Meshes**:

* Ensure mesh files exist in correct directory
* Check file permissions and paths
* Verify mesh file formats (DAE, STL supported)

**Transform Issues**:

* Validate joint definitions and origins
* Check parent-child link relationships
* Ensure no circular dependencies

**Simulation Problems**:

* Verify Gazebo plugin configurations
* Check physics properties and collisions
* Validate sensor plugin parameters

Performance Optimization
~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: xml

   <!-- Use simplified collision meshes -->
   <collision>
     <geometry>
       <box size="${base_length} ${base_width} ${base_height}"/>
     </geometry>
   </collision>
   
   <!-- Optimize mesh complexity for real-time rendering -->
   <visual>
     <geometry>
       <mesh filename="package://littlebot_description/meshes/littlebot_simplified.dae"/>
     </geometry>
   </visual>

Dependencies
------------

**ROS2 Packages**:

* ``robot_state_publisher``
* ``joint_state_publisher``
* ``joint_state_publisher_gui``
* ``xacro``
* ``urdf``

**Visualization**:

* ``rviz2``
* ``tf2_tools``

**Simulation**:

* ``gazebo_ros_pkgs``
* ``gazebo_ros``

API Reference
-------------

For detailed URDF specification and ROS2 robot description standards, see the official ROS2 documentation.