littlebot_navigation  
===================

The `littlebot_navigation` package provides autonomous navigation capabilities using the ROS2 Navigation2 (Nav2) stack for path planning, obstacle avoidance, and goal execution.

Overview
--------

This package implements:

* Global path planning for long-distance navigation
* Local path planning for obstacle avoidance
* Behavior trees for complex navigation behaviors
* Recovery behaviors for error handling
* Dynamic obstacle avoidance
* Integration with localization and mapping systems

Components
----------

Navigation Stack
~~~~~~~~~~~~~~~~

**Core Navigation Nodes**:

* **Planner Server**: Global path planning algorithms
* **Controller Server**: Local trajectory following
* **Recoveries Server**: Recovery behavior execution  
* **BT Navigator**: Behavior tree-based navigation logic
* **Waypoint Follower**: Sequential waypoint navigation
* **Lifecycle Manager**: Node lifecycle coordination

**Path Planning Algorithms**:

* **NavFn Planner**: Dijkstra-based global planner
* **Smac Planner**: State-space search planner
* **TEB Local Planner**: Timed Elastic Band local planner
* **DWB Local Planner**: Dynamic Window Approach planner

Configuration Files
~~~~~~~~~~~~~~~~~~~

**Navigation Parameters**: `config/nav2_params.yaml`

.. code-block:: yaml

   bt_navigator:
     ros__parameters:
       use_sim_time: false
       global_frame: map
       robot_base_frame: base_link
       odom_topic: /odom
       bt_loop_duration: 10
       default_server_timeout: 20
       enable_groot_monitoring: true
       groot_zmq_publisher_port: 1666
       groot_zmq_server_port: 1667
       default_nav_to_pose_bt_xml: ""
       default_nav_through_poses_bt_xml: ""

   global_costmap:
     global_costmap:
       ros__parameters:
         update_frequency: 1.0
         publish_frequency: 1.0
         global_frame: map
         robot_base_frame: base_link
         use_sim_time: false
         resolution: 0.05
         track_unknown_space: true
         plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

**Costmap Configuration**: `config/costmap_params.yaml`

.. code-block:: yaml

   # Static layer (map-based obstacles)
   static_layer:
     plugin: "nav2_costmap_2d::StaticLayer"
     map_subscribe_transient_local: true

   # Obstacle layer (sensor-based obstacles)  
   obstacle_layer:
     plugin: "nav2_costmap_2d::ObstacleLayer"
     enabled: true
     observation_sources: scan
     scan:
       topic: /scan
       max_obstacle_height: 2.0
       clearing: true
       marking: true
       data_type: "LaserScan"

   # Inflation layer (safety margins)
   inflation_layer:
     plugin: "nav2_costmap_2d::InflationLayer"
     cost_scaling_factor: 3.0
     inflation_radius: 0.55

Launch Files
~~~~~~~~~~~~

**Main Navigation Launch**: `launch/littlebot_navigation.launch.py`

Starts the complete Navigation2 stack:

* All navigation servers and nodes
* Costmap configuration
* Behavior tree setup
* Recovery behaviors

**Waypoint Navigation Launch**: `launch/waypoint_navigation.launch.py`

Specialized for waypoint following:

* Sequential waypoint execution
* Custom behavior trees
* Waypoint management interface

Usage
-----

Starting Navigation
~~~~~~~~~~~~~~~~~~~

Launch the navigation system:

.. code-block:: bash

   # Start with default configuration
   ros2 launch littlebot_navigation littlebot_navigation.launch.py

   # Start with custom parameters
   ros2 launch littlebot_navigation littlebot_navigation.launch.py \
       params_file:=/path/to/custom_params.yaml

   # Start with custom map
   ros2 launch littlebot_navigation littlebot_navigation.launch.py \
       map:=/path/to/your/map.yaml

Setting Navigation Goals
~~~~~~~~~~~~~~~~~~~~~~~~

**Using Command Line**:

.. code-block:: bash

   # Send a navigation goal
   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
   '{
     pose: {
       header: {frame_id: "map"},
       pose: {
         position: {x: 2.0, y: 1.0, z: 0.0},
         orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
       }
     }
   }'

   # Follow waypoints
   ros2 action send_goal /follow_waypoints nav2_msgs/action/FollowWaypoints \
   '{
     poses: [
       {
         header: {frame_id: "map"},
         pose: {
           position: {x: 1.0, y: 0.0, z: 0.0},
           orientation: {w: 1.0}
         }
       },
       {
         header: {frame_id: "map"}, 
         pose: {
           position: {x: 2.0, y: 1.0, z: 0.0},
           orientation: {w: 1.0}
         }
       }
     ]
   }'

**Using RViz**:

1. Open RViz with navigation configuration
2. Click "2D Nav Goal" button
3. Click and drag on the map to set goal pose

**Programmatic Interface**:

.. code-block:: python

   import rclpy
   from nav2_simple_commander.robot_navigator import BasicNavigator
   from geometry_msgs.msg import PoseStamped

   def navigate_to_goal():
       navigator = BasicNavigator()
       
       # Set initial pose
       initial_pose = PoseStamped()
       initial_pose.header.frame_id = 'map'
       initial_pose.header.stamp = navigator.get_clock().now().to_msg()
       initial_pose.pose.position.x = 0.0
       initial_pose.pose.position.y = 0.0
       initial_pose.pose.orientation.w = 1.0
       navigator.setInitialPose(initial_pose)
       
       # Navigate to goal
       goal_pose = PoseStamped()
       goal_pose.header.frame_id = 'map'
       goal_pose.header.stamp = navigator.get_clock().now().to_msg()
       goal_pose.pose.position.x = 2.0
       goal_pose.pose.position.y = 1.0
       goal_pose.pose.orientation.w = 1.0
       
       navigator.goToPose(goal_pose)

Path Planning
-------------

Global Planning
~~~~~~~~~~~~~~~

**NavFn Planner Configuration**:

.. code-block:: yaml

   planner_server:
     ros__parameters:
       expected_planner_frequency: 20.0
       use_sim_time: false
       planner_plugins: ["GridBased"]
       GridBased:
         plugin: "nav2_navfn_planner/NavfnPlanner"
         tolerance: 0.5
         use_astar: false
         allow_unknown: true

**Smac Planner Configuration**:

.. code-block:: yaml

   GridBased:
     plugin: "nav2_smac_planner/SmacPlannerHybrid"
     tolerance: 0.5
     downsample_costmap: false
     downsampling_factor: 1
     allow_unknown: true
     max_iterations: 1000000
     max_on_approach_iterations: 1000
     max_planning_time: 3.5
     motion_model_for_search: "DUBIN"
     cost_travel_multiplier: 2.0

Local Planning
~~~~~~~~~~~~~~

**DWB Local Planner**:

.. code-block:: yaml

   controller_server:
     ros__parameters:
       use_sim_time: false
       controller_frequency: 20.0
       min_x_velocity_threshold: 0.001
       min_y_velocity_threshold: 0.5
       min_theta_velocity_threshold: 0.001
       controller_plugins: ["FollowPath"]
       
       FollowPath:
         plugin: "dwb_core::DWBLocalPlanner"
         debug_trajectory_details: true
         min_vel_x: 0.0
         min_vel_y: 0.0
         max_vel_x: 0.26
         max_vel_y: 0.0
         max_vel_theta: 1.0
         min_speed_xy: 0.0
         max_speed_xy: 0.26
         min_speed_theta: 0.0
         acc_lim_x: 2.5
         acc_lim_y: 0.0
         acc_lim_theta: 3.2
         decel_lim_x: -2.5
         decel_lim_y: 0.0
         decel_lim_theta: -3.2

**TEB Local Planner**:

.. code-block:: yaml

   FollowPath:
     plugin: "teb_local_planner::TebLocalPlannerROS"
     teb_autosize: true
     dt_ref: 0.3
     dt_hysteresis: 0.1
     max_samples: 500
     global_plan_overwrite_orientation: true
     allow_init_with_backwards_motion: false
     max_global_plan_lookahead_dist: 3.0
     global_plan_viapoint_sep: -1
     global_plan_prune_distance: 1
     exact_arc_length: false
     feasibility_check_no_poses: 2

Costmaps
--------

Global Costmap
~~~~~~~~~~~~~~

Long-range planning costmap:

.. code-block:: yaml

   global_costmap:
     global_costmap:
       ros__parameters:
         update_frequency: 1.0
         publish_frequency: 1.0
         global_frame: map
         robot_base_frame: base_link
         use_sim_time: false
         robot_radius: 0.2
         resolution: 0.05
         track_unknown_space: true
         plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
         always_send_full_costmap: true

Local Costmap
~~~~~~~~~~~~~

Short-range obstacle avoidance costmap:

.. code-block:: yaml

   local_costmap:
     local_costmap:
       ros__parameters:
         update_frequency: 5.0
         publish_frequency: 2.0
         global_frame: odom
         robot_base_frame: base_link
         use_sim_time: false
         rolling_window: true
         width: 3
         height: 3
         resolution: 0.05
         robot_radius: 0.2
         plugins: ["voxel_layer", "inflation_layer"]

Behavior Trees
--------------

Navigation Logic
~~~~~~~~~~~~~~~~

**Default Behavior Tree**:

The navigation system uses behavior trees for decision making:

.. code-block:: xml

   <root main_tree_to_execute="MainTree">
     <BehaviorTree ID="MainTree">
       <RecoveryNode number_of_retries="6" name="NavigateRecovery">
         <PipelineSequence name="NavigateWithReplanning">
           <RateController hz="1.0">
             <RecoveryNode number_of_retries="1" name="ComputePathToPose">
               <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
               <ReactiveFallback name="ComputePathToPoseRecovery">
                 <GoalUpdated/>
                 <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
               </ReactiveFallback>
             </RecoveryNode>
           </RateController>
           <RecoveryNode number_of_retries="1" name="FollowPath">
             <FollowPath path="{path}" controller_id="FollowPath"/>
             <ReactiveFallback name="FollowPathRecovery">
               <GoalUpdated/>
               <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
             </ReactiveFallback>
           </RecoveryNode>
         </PipelineSequence>
         <ReactiveFallback name="RecoveryFallback">
           <GoalUpdated/>
           <RoundRobin name="RecoveryActions">
             <Sequence name="ClearingActions">
               <ClearEntireCostmap name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap"/>
               <ClearEntireCostmap name="ClearGlobalCostmap-Subtree" service_name="global_costmap/clear_entirely_global_costmap"/>
             </Sequence>
             <Spin spin_dist="1.57"/>
             <Wait wait_duration="5"/>
             <BackUp backup_dist="0.15" backup_speed="0.025"/>
           </RoundRobin>
         </ReactiveFallback>
       </RecoveryNode>
     </BehaviorTree>
   </root>

**Custom Behavior Trees**:

Create custom navigation behaviors:

.. code-block:: xml

   <BehaviorTree ID="FollowWaypoints">
     <RecoveryNode number_of_retries="1" name="FollowWaypoints">
       <FollowWaypoints waypoints="{waypoints}"/>
       <ReactiveFallback name="FollowWaypointsRecovery">
         <GoalUpdated/>
         <ClearEntireCostmap name="ClearLocalCostmap" service_name="local_costmap/clear_entirely_local_costmap"/>
       </ReactiveFallback>
     </RecoveryNode>
   </BehaviorTree>

Recovery Behaviors
------------------

Error Handling
~~~~~~~~~~~~~~

**Available Recovery Actions**:

* **ClearCostmap**: Clear obstacle information
* **Spin**: Rotate in place to clear sensor data
* **BackUp**: Move backwards to escape tight spaces
* **Wait**: Pause navigation temporarily

**Recovery Configuration**:

.. code-block:: yaml

   recoveries_server:
     ros__parameters:
       costmap_topic: local_costmap/costmap_raw
       footprint_topic: local_costmap/published_footprint
       cycle_frequency: 10.0
       recovery_plugins: ["spin", "backup", "wait"]
       
       spin:
         plugin: "nav2_recoveries/Spin"
         simulate_ahead_time: 2.0
         max_rotational_vel: 1.0
         min_rotational_vel: 0.4
         rotational_acc_lim: 3.2

Advanced Features
-----------------

Dynamic Obstacles
~~~~~~~~~~~~~~~~~

Real-time obstacle detection and avoidance:

.. code-block:: yaml

   # Enable dynamic obstacle layer
   obstacle_layer:
     plugin: "nav2_costmap_2d::ObstacleLayer"
     enabled: true
     observation_sources: scan
     scan:
       topic: /scan
       max_obstacle_height: 2.0
       clearing: true
       marking: true
       data_type: "LaserScan"
       raytrace_max_range: 3.0
       raytrace_min_range: 0.0
       obstacle_max_range: 2.5
       obstacle_min_range: 0.0

Multi-Robot Navigation
~~~~~~~~~~~~~~~~~~~~~~

Support for multiple robots:

.. code-block:: yaml

   # Robot-specific namespaces
   bt_navigator:
     ros__parameters:
       global_frame: /robot1/map
       robot_base_frame: /robot1/base_link
       odom_topic: /robot1/odom

Navigation Monitoring
~~~~~~~~~~~~~~~~~~~~~

Monitor navigation performance:

.. code-block:: bash

   # Check navigation status
   ros2 topic echo /navigate_to_pose/_action/status

   # Monitor path execution
   ros2 topic echo /plan
   ros2 topic echo /local_plan

   # Check costmap data
   ros2 topic echo /global_costmap/costmap

Troubleshooting
---------------

Common Issues
~~~~~~~~~~~~~

**Path Planning Failures**:

.. code-block:: bash

   # Check costmap for obstacles
   ros2 topic echo /global_costmap/costmap
   
   # Verify map data
   ros2 topic echo /map
   
   # Check planner configuration
   ros2 param list /planner_server

**Controller Issues**:

.. code-block:: bash

   # Monitor controller commands
   ros2 topic echo /cmd_vel
   
   # Check local costmap
   ros2 topic echo /local_costmap/costmap
   
   # Verify robot footprint
   ros2 param get /local_costmap/local_costmap robot_radius

**Recovery Behavior Problems**:

.. code-block:: bash

   # Check recovery server status
   ros2 node info /recoveries_server
   
   # Monitor recovery actions
   ros2 topic echo /spin/_action/feedback

Performance Optimization
~~~~~~~~~~~~~~~~~~~~~~~~

**Computational Efficiency**:

.. code-block:: yaml

   # Reduce costmap update frequencies
   global_costmap:
     global_costmap:
       ros__parameters:
         update_frequency: 0.5  # Reduce from 1.0
         publish_frequency: 0.5

   # Optimize planner settings
   planner_server:
     ros__parameters:
       expected_planner_frequency: 10.0  # Reduce from 20.0

**Memory Usage**:

.. code-block:: yaml

   # Reduce costmap size
   local_costmap:
     local_costmap:
       ros__parameters:
         width: 2  # Reduce from 3
         height: 2

Integration Examples
--------------------

Complete Navigation System
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # Start complete system
   ros2 launch littlebot_bringup littlebot_bringup.launch.py &
   ros2 launch littlebot_localization littlebot_localization.launch.py &
   ros2 launch littlebot_navigation littlebot_navigation.launch.py

Simulation Integration
~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # Start Gazebo simulation
   ros2 launch littlebot_gazebo littlebot_gazebo_classic.launch.py &
   
   # Start navigation in simulation
   ros2 launch littlebot_navigation littlebot_navigation.launch.py \
       use_sim_time:=true

Dependencies
------------

**ROS2 Packages**:

* ``nav2_bringup``
* ``nav2_planner``
* ``nav2_controller``
* ``nav2_recoveries``
* ``nav2_bt_navigator``
* ``nav2_waypoint_follower``
* ``nav2_lifecycle_manager``

**System Dependencies**:

* Laser scanner driver
* Localization system
* Map data

API Reference
-------------

For detailed API documentation, see the Navigation2 official documentation and behavior tree guides.