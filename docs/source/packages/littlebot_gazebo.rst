littlebot_gazebo
===============

The `littlebot_gazebo` package provides comprehensive simulation support for the LittleBot robot using both Gazebo Classic and Gazebo Ignition (Garden) simulators.

Overview
--------

This package implements:

* Complete robot simulation in realistic environments
* Physics-based robot modeling and dynamics
* Sensor simulation (LiDAR, IMU, cameras)
* Multiple world environments
* Integration with ROS2 Control framework
* Support for both Gazebo Classic and Ignition

Components
----------

Simulation Environments
~~~~~~~~~~~~~~~~~~~~~~~

**World Files**:

* ``worlds/floor.sdf``: Simple flat floor environment
* ``worlds/warehouse.sdf``: Indoor warehouse scenario
* ``worlds/outdoor.sdf``: Outdoor terrain environment
* ``worlds/maze.sdf``: Navigation challenge environment

**Model Assets**:

* ``models/floor/``: Ground plane models
* ``models/obstacles/``: Various obstacle models  
* ``models/furniture/``: Indoor environment objects
* ``urdf/littlebot_description.urdf.xacro``: Robot model for simulation

Launch Files
~~~~~~~~~~~~

**Gazebo Classic Launch**: `launch/littlebot_gazebo_classic.launch.py`

Starts simulation with Gazebo Classic:

* World environment loading
* Robot model spawning
* Sensor plugin initialization
* ROS2 Control integration

**Gazebo Ignition Launch**: `launch/littlebot_gazebo_ignition.launch.py`

Modern Gazebo Ignition simulation:

* Enhanced physics simulation
* Improved rendering performance
* Advanced sensor models
* Better multi-robot support

Robot Model Integration
~~~~~~~~~~~~~~~~~~~~~~~

**Simulation-specific URDF**: `urdf/littlebot.urdf.xacro`

Enhanced robot model for simulation:

.. code-block:: xml

   <robot name="littlebot" xmlns:xacro="http://www.ros.org/wiki/xacro">
     
     <!-- Simulation-specific properties -->
     <xacro:property name="wheel_mu" value="0.8"/>
     <xacro:property name="wheel_kp" value="10000000.0"/>
     <xacro:property name="wheel_kd" value="1.0"/>
     
     <!-- Include base robot description -->
     <xacro:include filename="$(find littlebot_description)/urdf/littlebot.urdf.xacro"/>
     
     <!-- Gazebo plugins and materials -->
     <xacro:include filename="$(find littlebot_gazebo)/urdf/littlebot_gazebo.xacro"/>
     
   </robot>

Usage
-----

Starting Simulation
~~~~~~~~~~~~~~~~~~~

Launch Gazebo Classic simulation:

.. code-block:: bash

   # Start with default world
   ros2 launch littlebot_gazebo littlebot_gazebo_classic.launch.py

   # Start with custom world
   ros2 launch littlebot_gazebo littlebot_gazebo_classic.launch.py \
       world:=warehouse

   # Start with custom robot pose
   ros2 launch littlebot_gazebo littlebot_gazebo_classic.launch.py \
       x:=2.0 y:=1.0 z:=0.0 yaw:=1.57

Launch Gazebo Ignition simulation:

.. code-block:: bash

   # Start Ignition simulation
   ros2 launch littlebot_gazebo littlebot_gazebo_ignition.launch.py

   # Start with custom configuration
   ros2 launch littlebot_gazebo littlebot_gazebo_ignition.launch.py \
       world:=outdoor gui:=true

Available Parameters
~~~~~~~~~~~~~~~~~~~~

.. list-table:: Launch Parameters
   :widths: 25 25 50
   :header-rows: 1

   * - Parameter
     - Default Value
     - Description
   * - ``world``
     - ``floor``
     - World environment to load
   * - ``gui``
     - ``true``
     - Show Gazebo GUI
   * - ``x``, ``y``, ``z``
     - ``0.0``
     - Initial robot position
   * - ``roll``, ``pitch``, ``yaw``
     - ``0.0``
     - Initial robot orientation
   * - ``robot_name``
     - ``littlebot``
     - Robot model name
   * - ``use_sim_time``
     - ``true``
     - Use simulation time

Controlling the Robot
~~~~~~~~~~~~~~~~~~~~~

Control the simulated robot:

.. code-block:: bash

   # Start robot controllers
   ros2 control load_controller diff_drive_controller
   ros2 control switch_controllers --activate diff_drive_controller

   # Send velocity commands
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
       '{linear: {x: 0.5}, angular: {z: 0.0}}'

   # Start teleoperation
   ros2 launch littlebot_teleop littlebot_teleop.launch.py

Physics Configuration
---------------------

Gazebo Classic Physics
~~~~~~~~~~~~~~~~~~~~~~

**World Physics Settings**:

.. code-block:: xml

   <physics name="default_physics" default="0" type="ode">
     <gravity>0 0 -9.8066</gravity>
     <ode>
       <solver>
         <type>quick</type>
         <iters>150</iters>
         <sor>1.3</sor>
         <use_dynamic_moi_rescaling>0</use_dynamic_moi_rescaling>
       </solver>
       <constraints>
         <cfm>0.00001</cfm>
         <erp>0.2</erp>
         <contact_max_correcting_vel>1000</contact_max_correcting_vel>
         <contact_surface_layer>0.01</contact_surface_layer>
       </constraints>
     </ode>
     <max_step_size>0.004</max_step_size>
     <real_time_factor>1.0</real_time_factor>
     <real_time_update_rate>250</real_time_update_rate>
   </physics>

**Robot Physics Properties**:

.. code-block:: xml

   <!-- Base link physics -->
   <gazebo reference="base_link">
     <material>Gazebo/Blue</material>
     <mu1>0.2</mu1>
     <mu2>0.2</mu2>
     <selfCollide>true</selfCollide>
     <gravity>true</gravity>
   </gazebo>

   <!-- Wheel physics -->
   <gazebo reference="left_wheel_link">
     <material>Gazebo/Black</material>
     <mu1>0.8</mu1>
     <mu2>0.8</mu2>
     <kp>10000000.0</kp>
     <kd>1.0</kd>
     <minDepth>0.001</minDepth>
     <maxVel>0.1</maxVel>
   </gazebo>

Ignition Physics
~~~~~~~~~~~~~~~~

**Advanced Physics Engine**:

.. code-block:: xml

   <physics name="1ms" type="ignored">
     <max_step_size>0.001</max_step_size>
     <real_time_factor>1.0</real_time_factor>
   </physics>

   <!-- DART physics engine -->
   <plugin filename="libignition-gazebo-physics-system.so" 
           name="ignition::gazebo::systems::Physics">
     <engine>
       <filename>libdart-physics-plugin.so</filename>
     </engine>
   </plugin>

Sensor Plugins
--------------

LiDAR Simulation
~~~~~~~~~~~~~~~~

**Gazebo Classic LiDAR**:

.. code-block:: xml

   <gazebo reference="laser_link">
     <sensor type="ray" name="rplidar">
       <pose>0 0 0 0 0 0</pose>
       <visualize>true</visualize>
       <update_rate>40</update_rate>
       <ray>
         <scan>
           <horizontal>
             <samples>720</samples>
             <resolution>1</resolution>
             <min_angle>-3.14159</min_angle>
             <max_angle>3.14159</max_angle>
           </horizontal>
         </scan>
         <range>
           <min>0.12</min>
           <max>12.0</max>
           <resolution>0.015</resolution>
         </range>
         <noise>
           <type>gaussian</type>
           <mean>0.0</mean>
           <stddev>0.01</stddev>
         </noise>
       </ray>
       <plugin name="gazebo_ros_rplidar_controller" filename="libgazebo_ros_ray_sensor.so">
         <ros>
           <namespace>/</namespace>
           <remapping>~/out:=scan</remapping>
         </ros>
         <output_type>sensor_msgs/LaserScan</output_type>
       </plugin>
     </sensor>
   </gazebo>

**Ignition LiDAR**:

.. code-block:: xml

   <sensor name="lidar" type="gpu_lidar">
     <pose>0 0 0.1 0 0 0</pose>
     <topic>scan</topic>
     <update_rate>10</update_rate>
     <ray>
       <scan>
         <horizontal>
           <samples>640</samples>
           <resolution>1</resolution>
           <min_angle>-3.14</min_angle>
           <max_angle>3.14</max_angle>
         </horizontal>
       </scan>
       <range>
         <min>0.08</min>
         <max>10.0</max>
         <resolution>0.01</resolution>
       </range>
     </ray>
     <alwaysOn>1</alwaysOn>
     <visualize>true</visualize>
   </sensor>

IMU Simulation
~~~~~~~~~~~~~~

**IMU Sensor Plugin**:

.. code-block:: xml

   <gazebo reference="imu_link">
     <gravity>true</gravity>
     <sensor name="imu_sensor" type="imu">
       <always_on>true</always_on>
       <update_rate>100</update_rate>
       <visualize>true</visualize>
       <topic>imu</topic>
       <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
         <ros>
           <namespace>/</namespace>
           <remapping>~/out:=imu</remapping>
         </ros>
         <initial_orientation_as_reference>false</initial_orientation_as_reference>
       </plugin>
       <pose>0 0 0 0 0 0</pose>
     </sensor>
   </gazebo>

Camera Simulation
~~~~~~~~~~~~~~~~~

**RGB Camera Plugin**:

.. code-block:: xml

   <gazebo reference="camera_link">
     <sensor type="camera" name="camera">
       <update_rate>30.0</update_rate>
       <camera name="head">
         <horizontal_fov>1.3962634</horizontal_fov>
         <image>
           <width>800</width>
           <height>600</height>
           <format>R8G8B8</format>
         </image>
         <clip>
           <near>0.02</near>
           <far>300</far>
         </clip>
         <noise>
           <type>gaussian</type>
           <mean>0.0</mean>
           <stddev>0.007</stddev>
         </noise>
       </camera>
       <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
         <ros>
           <namespace>/</namespace>
           <remapping>image_raw:=camera/image_raw</remapping>
           <remapping>camera_info:=camera/camera_info</remapping>
         </ros>
         <camera_name>camera</camera_name>
         <frame_name>camera_link</frame_name>
         <hack_baseline>0.07</hack_baseline>
       </plugin>
     </sensor>
   </gazebo>

ROS2 Control Integration
------------------------

Hardware Interface
~~~~~~~~~~~~~~~~~~

**Gazebo ROS2 Control Plugin**:

.. code-block:: xml

   <gazebo>
     <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
       <parameters>$(find littlebot_control)/config/control.yaml</parameters>
       <ros>
         <namespace>/</namespace>
       </ros>
     </plugin>
   </gazebo>

**Joint Interface Configuration**:

.. code-block:: xml

   <ros2_control name="GazeboSystem" type="system">
     <hardware>
       <plugin>gazebo_ros2_control/GazeboSystem</plugin>
     </hardware>
     <joint name="left_wheel_joint">
       <command_interface name="velocity">
         <param name="min">-10</param>
         <param name="max">10</param>
       </command_interface>
       <state_interface name="velocity"/>
       <state_interface name="position"/>
     </joint>
     <joint name="right_wheel_joint">
       <command_interface name="velocity">
         <param name="min">-10</param>
         <param name="max">10</param>
       </command_interface>
       <state_interface name="velocity"/>
       <state_interface name="position"/>
     </joint>
   </ros2_control>

World Environments
------------------

Floor World
~~~~~~~~~~~

**Simple Testing Environment**:

.. code-block:: xml

   <sdf version="1.6">
     <world name="floor_world">
       
       <!-- Physics settings -->
       <include>
         <uri>model://sun</uri>
       </include>
       
       <!-- Ground plane -->
       <include>
         <uri>model://ground_plane</uri>
       </include>
       
       <!-- Custom floor model -->
       <model name="floor">
         <static>true</static>
         <link name="link">
           <collision name="collision">
             <geometry>
               <plane>
                 <normal>0 0 1</normal>
                 <size>20 20</size>
               </plane>
             </geometry>
           </collision>
           <visual name="visual">
             <geometry>
               <plane>
                 <normal>0 0 1</normal>
                 <size>20 20</size>
               </plane>
             </geometry>
             <material>
               <script>
                 <uri>file://media/materials/scripts/gazebo.material</uri>
                 <name>Gazebo/Grey</name>
               </script>
             </material>
           </visual>
         </link>
       </model>
       
     </world>
   </sdf>

Warehouse World
~~~~~~~~~~~~~~~

**Indoor Navigation Environment**:

.. code-block:: xml

   <world name="warehouse">
     <!-- Lighting -->
     <light type="directional" name="sun">
       <cast_shadows>true</cast_shadows>
       <pose>0 0 10 0 0 0</pose>
       <diffuse>0.8 0.8 0.8 1</diffuse>
       <specular>0.2 0.2 0.2 1</specular>
       <attenuation>
         <range>1000</range>
         <constant>0.9</constant>
         <linear>0.01</linear>
         <quadratic>0.001</quadratic>
       </attenuation>
       <direction>-0.5 0.1 -0.9</direction>
     </light>
     
     <!-- Warehouse structure -->
     <include>
       <uri>model://warehouse_walls</uri>
       <pose>0 0 0 0 0 0</pose>
     </include>
     
     <!-- Obstacles and furniture -->
     <include>
       <uri>model://table</uri>
       <pose>2 2 0 0 0 0</pose>
     </include>
     
   </world>

Advanced Features
-----------------

Multi-Robot Simulation
~~~~~~~~~~~~~~~~~~~~~~

Support for multiple robots:

.. code-block:: bash

   # Launch first robot
   ros2 launch littlebot_gazebo littlebot_gazebo_classic.launch.py \
       robot_name:=robot1 \
       namespace:=robot1 \
       x:=0.0 y:=0.0

   # Launch second robot  
   ros2 launch littlebot_gazebo littlebot_gazebo_classic.launch.py \
       robot_name:=robot2 \
       namespace:=robot2 \
       x:=2.0 y:=2.0

Dynamic World Loading
~~~~~~~~~~~~~~~~~~~~~

Change worlds during runtime:

.. code-block:: python

   # Python script to change worlds
   import rclpy
   from gazebo_msgs.srv import LoadWorld

   def load_new_world():
       client = node.create_client(LoadWorld, '/gazebo/load_world')
       request = LoadWorld.Request()
       request.world_name = 'warehouse'
       request.world_filename = '/path/to/warehouse.world'
       
       future = client.call_async(request)
       return future

Sensor Noise Modeling
~~~~~~~~~~~~~~~~~~~~~

Realistic sensor behavior:

.. code-block:: xml

   <!-- LiDAR noise -->
   <noise>
     <type>gaussian</type>
     <mean>0.0</mean>
     <stddev>0.01</stddev>
   </noise>
   
   <!-- IMU noise -->
   <noise>
     <type>gaussian</type>
     <accel>
       <mean>0.0</mean>
       <stddev>1.7e-2</stddev>
       <bias_mean>0.1</bias_mean>
       <bias_stddev>0.001</bias_stddev>
     </accel>
     <rate>
       <mean>0.0</mean>
       <stddev>1.6968e-04</stddev>
       <bias_mean>0.0</bias_mean>
       <bias_stddev>0.0</bias_stddev>
     </rate>
   </noise>

Performance Optimization
------------------------

Rendering Optimization
~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: xml

   <!-- Reduce visual complexity -->
   <gazebo reference="base_link">
     <visual>
       <geometry>
         <box size="0.4 0.3 0.1"/>  <!-- Use simple geometry -->
       </geometry>
     </visual>
   </gazebo>

   <!-- Disable shadows for performance -->
   <light name="sun" type="directional">
     <cast_shadows>false</cast_shadows>
   </light>

Physics Optimization
~~~~~~~~~~~~~~~~~~~~

.. code-block:: xml

   <!-- Adjust physics parameters for performance -->
   <physics type="ode">
     <max_step_size>0.01</max_step_size>  <!-- Larger step size -->
     <real_time_factor>1.0</real_time_factor>
     <real_time_update_rate>100</real_time_update_rate>  <!-- Lower rate -->
   </physics>

Memory Management
~~~~~~~~~~~~~~~~~

.. code-block:: xml

   <!-- Limit sensor range to reduce computation -->
   <sensor type="ray" name="rplidar">
     <ray>
       <range>
         <max>8.0</max>  <!-- Reduce from 12.0 -->
       </range>
       <scan>
         <horizontal>
           <samples>360</samples>  <!-- Reduce from 720 -->
         </horizontal>
       </scan>
     </ray>
   </sensor>

Troubleshooting
---------------

Common Issues
~~~~~~~~~~~~~

**Gazebo Won't Start**:

.. code-block:: bash

   # Check Gazebo installation
   gazebo --version
   
   # Clear Gazebo cache
   rm -rf ~/.gazebo/models/*
   
   # Reset Gazebo configuration  
   rm -rf ~/.gazebo/gui.ini

**Robot Falls Through Ground**:

.. code-block:: bash

   # Check collision meshes
   gz model -m littlebot -i
   
   # Verify physics parameters
   gz world -i floor_world

**Poor Performance**:

.. code-block:: bash

   # Run without GUI
   ros2 launch littlebot_gazebo littlebot_gazebo_classic.launch.py gui:=false
   
   # Use headless mode
   export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models
   gzserver worlds/floor.world

**Sensors Not Working**:

.. code-block:: bash

   # Check sensor topics
   ros2 topic list | grep -E "(scan|imu|camera)"
   
   # Verify plugin loading
   gz plugin -l
   
   # Check sensor data
   ros2 topic echo /scan

Integration Examples
--------------------

Complete Simulation System
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # Start Gazebo simulation
   ros2 launch littlebot_gazebo littlebot_gazebo_classic.launch.py &
   
   # Start navigation in simulation
   ros2 launch littlebot_navigation littlebot_navigation.launch.py \
       use_sim_time:=true &
   
   # Start teleop for manual control
   ros2 launch littlebot_teleop littlebot_teleop.launch.py

SLAM in Simulation
~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # Start simulation
   ros2 launch littlebot_gazebo littlebot_gazebo_classic.launch.py world:=warehouse &
   
   # Start SLAM
   ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true &
   
   # Control robot to build map
   ros2 launch littlebot_teleop littlebot_teleop.launch.py

Dependencies
------------

**ROS2 Packages**:

* ``gazebo_ros_pkgs``
* ``gazebo_ros2_control``
* ``robot_state_publisher``
* ``joint_state_publisher``

**System Dependencies**:

* Gazebo Classic (11+)
* Gazebo Ignition Garden (optional)
* Graphics drivers
* OGRE rendering engine

API Reference
-------------

For detailed Gazebo plugin API and world file specifications, see the official Gazebo documentation and ROS2 integration guides.