littlebot_localization
=====================

The `littlebot_localization` package provides localization capabilities for the LittleBot robot using various algorithms and sensor fusion techniques.

Overview
--------

This package implements:

* AMCL (Adaptive Monte Carlo Localization) for map-based localization
* Sensor fusion for improved position estimation
* Launch files for localization system startup
* Configuration files for different environments
* Integration with navigation and mapping systems

Components
----------

Localization Algorithms
~~~~~~~~~~~~~~~~~~~~~~~

**AMCL (Adaptive Monte Carlo Localization)**

Primary localization method using particle filters:

* Probabilistic localization in known maps
* Laser scan matching
* Odometry integration
* Adaptive particle count optimization

**Sensor Fusion**

Combines multiple sensor inputs:

* Wheel odometry from encoders
* IMU data for orientation
* Laser scan data for position correction
* Visual odometry (optional)

Configuration Files
~~~~~~~~~~~~~~~~~~~

**AMCL Configuration**: `config/amcl.yaml`

.. code-block:: yaml

   amcl:
     ros__parameters:
       # Filter parameters
       min_particles: 500
       max_particles: 2000
       kld_err: 0.05
       kld_z: 0.99
       
       # Odometry model parameters
       odom_model_type: "diff"
       odom_alpha1: 0.2    # rotation noise from rotation
       odom_alpha2: 0.2    # rotation noise from translation
       odom_alpha3: 0.8    # translation noise from translation
       odom_alpha4: 0.2    # translation noise from rotation
       
       # Laser model parameters
       laser_model_type: "likelihood_field"
       laser_max_beams: 60
       laser_sigma_hit: 0.2
       laser_z_hit: 0.5
       laser_z_short: 0.05
       laser_z_max: 0.05
       laser_z_rand: 0.5

**EKF Configuration**: `config/ekf.yaml`

Extended Kalman Filter for sensor fusion:

.. code-block:: yaml

   ekf_filter_node:
     ros__parameters:
       frequency: 30.0
       sensor_timeout: 0.1
       two_d_mode: true
       publish_tf: true
       
       # Odometry input
       odom0: /odom
       odom0_config: [false, false, false,
                      false, false, false,
                      true,  true,  false,
                      false, false, true,
                      false, false, false]
       
       # IMU input  
       imu0: /imu
       imu0_config: [false, false, false,
                     true,  true,  true,
                     false, false, false,
                     true,  true,  true,
                     true,  true,  true]

Launch Files
~~~~~~~~~~~~

**Main Localization Launch**: `launch/littlebot_localization.launch.py`

Starts the complete localization system:

* AMCL node with configuration
* Map server (if map provided)
* Transform publishers
* Sensor fusion nodes

**EKF Launch**: `launch/ekf_localization.launch.py`

Launches Extended Kalman Filter for sensor fusion:

* Robot localization EKF node
* Sensor data subscribers
* Transform broadcasting

Usage
-----

Starting Localization
~~~~~~~~~~~~~~~~~~~~~~

Launch the localization system:

.. code-block:: bash

   # Start with default configuration
   ros2 launch littlebot_localization littlebot_localization.launch.py

   # Start with custom map
   ros2 launch littlebot_localization littlebot_localization.launch.py \
       map:=/path/to/your/map.yaml

   # Start EKF sensor fusion
   ros2 launch littlebot_localization ekf_localization.launch.py

Setting Initial Pose
~~~~~~~~~~~~~~~~~~~~~

Set the robot's initial position on the map:

.. code-block:: bash

   # Using command line
   ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
   '{
     header: {frame_id: "map"},
     pose: {
       pose: {
         position: {x: 0.0, y: 0.0, z: 0.0},
         orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
       },
       covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
     }
   }'

   # Using RViz "2D Pose Estimate" tool
   # Click the "2D Pose Estimate" button and click/drag on the map

Monitoring Localization
~~~~~~~~~~~~~~~~~~~~~~~

Check localization status:

.. code-block:: bash

   # Monitor particle cloud
   ros2 topic echo /particlecloud

   # Check pose estimates
   ros2 topic echo /amcl_pose

   # Monitor transform quality
   ros2 run tf2_tools view_frames.py

System Architecture
-------------------

Node Structure
~~~~~~~~~~~~~~

**Core Nodes**:

* ``amcl``: Adaptive Monte Carlo Localization
* ``map_server``: Provides map data
* ``ekf_filter_node``: Extended Kalman Filter for sensor fusion
* ``robot_localization``: Multi-sensor fusion framework

**Data Flow**:

1. **Sensors** → Raw sensor data (odometry, IMU, laser)
2. **Sensor Fusion** → Filtered estimates
3. **AMCL** → Map-based localization
4. **Transform Publisher** → Global pose estimation

Transform Frames
~~~~~~~~~~~~~~~~

**Coordinate Frames**:

* ``map``: Fixed world coordinate frame
* ``odom``: Odometry coordinate frame (drift-prone)
* ``base_link``: Robot body coordinate frame
* ``laser``: Laser scanner coordinate frame

**Transform Tree**:

.. code-block::

   map → odom → base_link → laser
                    ↓
                sensors (IMU, etc.)

Configuration
-------------

AMCL Tuning
~~~~~~~~~~~

**Particle Filter Parameters**:

.. code-block:: yaml

   amcl:
     ros__parameters:
       # Particle count adaptation
       min_particles: 500      # Minimum particles
       max_particles: 5000     # Maximum particles
       kld_err: 0.01          # KLD error threshold
       kld_z: 0.99            # KLD z-score
       
       # Update thresholds
       update_min_d: 0.2       # Min translation for update
       update_min_a: 0.5       # Min rotation for update
       resample_interval: 1    # Resampling frequency

**Motion Model Tuning**:

.. code-block:: yaml

   # Differential drive model
   odom_model_type: "diff"
   
   # Noise parameters (tune based on your robot)
   odom_alpha1: 0.2    # Expected noise in rotation as a function of rotation
   odom_alpha2: 0.2    # Expected noise in rotation as a function of translation  
   odom_alpha3: 0.8    # Expected noise in translation as a function of translation
   odom_alpha4: 0.2    # Expected noise in translation as a function of rotation

**Laser Model Tuning**:

.. code-block:: yaml

   laser_model_type: "likelihood_field"
   laser_max_beams: 60         # Number of laser beams to use
   laser_sigma_hit: 0.2        # Standard deviation for hit
   laser_z_hit: 0.5           # Mixing parameter for hit
   laser_z_short: 0.05        # Mixing parameter for short
   laser_z_max: 0.05          # Mixing parameter for max
   laser_z_rand: 0.5          # Mixing parameter for random

EKF Configuration
~~~~~~~~~~~~~~~~~

**State Vector Configuration**:

.. code-block:: yaml

   # State vector: [x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]
   # Configuration array specifies which states to estimate from each sensor

   odom0_config: [false, false, false,    # position (x,y,z)
                  false, false, false,    # orientation (r,p,y)  
                  true,  true,  false,    # linear velocity (x,y,z)
                  false, false, true,     # angular velocity (r,p,y)
                  false, false, false]    # linear acceleration (x,y,z)

**Process Noise**:

.. code-block:: yaml

   # Process noise covariance (Q matrix)
   process_noise_covariance: [0.05, 0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                              0,    0.05, 0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                              0,    0,    0.06, 0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                              # ... continue for all 15 states
                              ]

Advanced Features
-----------------

Multi-Robot Localization
~~~~~~~~~~~~~~~~~~~~~~~~

Support for multiple robots:

.. code-block:: yaml

   amcl:
     ros__parameters:
       # Use robot-specific topics
       scan_topic: /robot1/scan
       initial_pose_topic: /robot1/initialpose
       
       # Separate transform frames
       base_frame_id: "robot1/base_link"
       odom_frame_id: "robot1/odom"

Dynamic Reconfiguration
~~~~~~~~~~~~~~~~~~~~~~~

Adjust parameters at runtime:

.. code-block:: bash

   # List available parameters
   ros2 param list /amcl

   # Change particle count
   ros2 param set /amcl min_particles 1000
   ros2 param set /amcl max_particles 3000

   # Update motion model parameters
   ros2 param set /amcl odom_alpha1 0.1

Localization Quality Assessment
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Monitor localization performance:

.. code-block:: bash

   # Check pose covariance
   ros2 topic echo /amcl_pose --field pose.covariance

   # Monitor particle spread
   ros2 topic echo /particlecloud --field poses

   # Evaluate transform accuracy
   ros2 run tf2_ros tf2_echo map base_link

Troubleshooting
---------------

Common Issues
~~~~~~~~~~~~~

**Poor Localization Accuracy**:

* Tune AMCL parameters for your environment
* Check laser scan quality and range
* Verify map accuracy and resolution
* Adjust particle count and update thresholds

**Localization Failure**:

.. code-block:: bash

   # Check sensor data
   ros2 topic echo /scan
   ros2 topic echo /odom
   
   # Verify map loading
   ros2 topic echo /map
   
   # Check initial pose setting
   ros2 topic echo /initialpose

**Transform Issues**:

.. code-block:: bash

   # Check transform tree
   ros2 run tf2_tools view_frames.py
   
   # Monitor specific transforms
   ros2 run tf2_ros tf2_echo map odom
   ros2 run tf2_ros tf2_echo odom base_link

Performance Optimization
~~~~~~~~~~~~~~~~~~~~~~~~

**Computational Efficiency**:

.. code-block:: yaml

   # Reduce particle count for better performance
   min_particles: 200
   max_particles: 1000
   
   # Reduce laser beam count
   laser_max_beams: 30
   
   # Increase update thresholds
   update_min_d: 0.5
   update_min_a: 1.0

**Memory Usage**:

.. code-block:: yaml

   # Limit particle history
   save_pose_rate: 0.5
   
   # Reduce map resolution if possible
   # (configure in map server)

Integration Examples
--------------------

With Navigation
~~~~~~~~~~~~~~~

.. code-block:: bash

   # Start localization
   ros2 launch littlebot_localization littlebot_localization.launch.py &
   
   # Start navigation (uses localization)
   ros2 launch littlebot_navigation littlebot_navigation.launch.py

With Mapping
~~~~~~~~~~~~

.. code-block:: bash

   # SLAM mode (simultaneous localization and mapping)
   ros2 launch littlebot_localization slam_localization.launch.py

Custom Sensor Integration
~~~~~~~~~~~~~~~~~~~~~~~~~

Add custom sensors to EKF:

.. code-block:: yaml

   # GPS input (if available)
   navsat0: /gps/fix
   navsat0_config: [true,  true,  false,  # Use x,y from GPS
                    false, false, false,
                    false, false, false,
                    false, false, false,
                    false, false, false]

Dependencies
------------

**ROS2 Packages**:

* ``nav2_amcl``
* ``robot_localization``
* ``nav2_map_server``
* ``tf2_ros``
* ``sensor_msgs``
* ``geometry_msgs``

**System Dependencies**:

* Laser scan driver
* IMU driver
* Odometry source

API Reference
-------------

For detailed API documentation, see the Nav2 AMCL and Robot Localization package documentation.