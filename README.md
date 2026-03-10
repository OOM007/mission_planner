# mission_planner

`mission_planner` is a ROS 2 node that runs a simple autonomous mission state machine for a MAVROS-controlled vehicle. It commands the companion [`mission_sequencer`](https://github.com/aau-cns/mission_sequencer?tab=readme-ov-file) node with waypoints and high-level requests, monitors vehicle state and battery, performs a perimeter search over a configurable area, validates target detections, and returns home for landing.

## Node Summary

The node executable is `mission_planner_node` and runs as `mission_planner`.

Main responsibilities:

- wait for a valid local pose and a vehicle that can be armed
- start a mission automatically or from an external request
- take off from `home`, fly to a `transit` sector, descend to search altitude, and execute a search pattern
- validate target detections before committing to approach
- hover above the confirmed target for a fixed duration
- return to home and land
- trigger emergency landing on critical battery or explicit abort

The internal decision loop runs at `1 Hz`.

## Mission Flow

The node uses the following phases:

1. `IDLE`
2. `TAKEOFF`
3. `TRANSIT`
4. `DESCEND`
5. `SEARCH`
6. `TARGET_DETECTED`
7. `APPROACH`
8. `HOVER`
9. `RETURN`
10. `EXIT`
11. `LAND`
12. `EMERGENCY`

Typical nominal flow:

`IDLE -> TAKEOFF -> TRANSIT -> DESCEND -> SEARCH -> TARGET_DETECTED -> APPROACH -> HOVER -> RETURN -> EXIT -> LAND -> IDLE`

Fallback paths:

- `SEARCH -> RETURN` when the search timeout expires
- `TARGET_DETECTED -> SEARCH` when detections cannot be confirmed
- `APPROACH/HOVER/RETURN/EXIT -> EMERGENCY` on critical battery
- any supported phase -> `EMERGENCY` on `ABORT_MISSION`

## Search And Target Handling

Search behavior is perimeter-based:

- the planner builds a rectangular search path from `search.X_extent`, `search.Y_extent`, and `search.step`
- `search.start_pos` selects the first waypoint on that perimeter
- each search waypoint yaws toward the rectangle center, with an optional `search.zero_offset_angle`
- each waypoint is advanced only after the mission sequencer reports completion or the vehicle is already within `waypoint_reached_tolerance`

Target handling is intentionally conservative:

- detections arrive on `bbox_points_map`
- detections outside the active search bounds are ignored
- a raw detection first becomes a candidate track
- a candidate is promoted to a confirmed target only after enough repeated hits
- if confirmation takes too long or misses accumulate, the node falls back to `SEARCH`

After confirmation:

- if battery is above `battery_warn_threshold`, the node approaches the target and hovers
- otherwise it skips the approach and returns home

## Landing And Frame Alignment

During `EXIT` and `LAND`, the node can estimate a `map -> odom` correction from landing-pad observations:

- landing pad observations are read from `/aruco/drone_pose`
- the current vehicle pose in odom is paired with the pad pose
- a filtered transform estimate is computed and used to convert map-frame mission waypoints into odom-frame waypoints

This allows late mission phases near home to compensate for drift between the global search frame and the vehicle's odometry frame.

### How `LandingPadEstimator` Works

[`include/mission_planner/landing_pad_estimator.hpp`](./include/mission_planner/landing_pad_estimator.hpp) estimates the static transform between the landing-pad/map frame and the vehicle odometry frame.

For each synchronized pair of observations it receives:

- `pose_map_base`: vehicle pose expressed in the landing-pad or map-like frame
- `pose_odom_base`: vehicle pose expressed in odom

It computes one candidate transform:

`T_map_odom = T_map_base * inverse(T_odom_base)`

The estimator then keeps a rolling buffer of these candidates and produces a robust estimate in three stages:

1. compute the median translation and circular-mean yaw across the buffer
2. reject samples whose XY translation or yaw deviate too much from that center
3. average the remaining inliers into the final `4x4` transform

Only translation and yaw are estimated. Roll and pitch are intentionally ignored because the planner only needs a planar correction for waypoint projection near home.

### What `waypoint_transformer.hpp` Does

[`include/mission_planner/waypoint_transformer.hpp`](./include/mission_planner/waypoint_transformer.hpp) contains the lightweight frame-conversion helpers used when the planner needs to express a waypoint in a different frame.

It provides:

- `T_from_xyz_yaw()`: builds a `4x4` transform from `x, y, z, yaw`
- `xyz_yaw_from_T()`: extracts `x, y, z, yaw` from a transform
- `mapToOdom()`: converts a waypoint from the planner's map frame into the odom frame expected by `mission_sequencer`
- `odomToMap()`: inverse conversion, useful for debugging

This is used in `publishWaypoint()`: waypoints are planned in map coordinates, but once `T_map_odom` has been estimated, they are projected into odom before being published. Yaw is kept in radians inside the helper and later converted to degrees only at the mission message boundary.

## ROS Interfaces

### Subscribed Topics

| Topic | Type | Purpose |
| --- | --- | --- |
| `bbox_points_map` | `geometry_msgs/msg/PointStamped` | Target detections used during search and confirmation |
| `mavros/local_position/pose` | `geometry_msgs/msg/PoseStamped` | Current vehicle pose |
| `/mavros/battery` | `sensor_msgs/msg/BatteryState` | Battery percentage input |
| `/mavros/state` | `mavros_msgs/msg/State` | Armed state monitoring |
| `mission/request` | `std_msgs/msg/String` | External mission control commands |
| `/autonomy/response` | `mission_sequencer/msg/MissionResponse` | Waypoint completion feedback |
| `/aruco/drone_pose` | `geometry_msgs/msg/PoseStamped` | Landing-pad pose used for map/odom alignment |

### Published Topics

| Topic | Type | Purpose |
| --- | --- | --- |
| `/mission_sequencer/waypoint_list` | `mission_sequencer/msg/MissionWaypointArray` | Waypoint commands for the sequencer |
| `/autonomy/request` | `mission_sequencer/msg/MissionRequest` | High-level mission sequencer requests such as arm, takeoff, and land |
| `mission_planner/debug_waypoint` | `geometry_msgs/msg/PoseStamped` | Optional debug visualization of the active waypoint |
| `mission_planner/current_status` | `std_msgs/msg/String` | Current FSM phase as a string |
| `mission_planner/current_decision_info` | `std_msgs/msg/String` | Reserved publisher; currently created but not populated by the node |

### External Commands

The node accepts the following strings on `mission/request`:

- `START_MISSION`: starts the mission from `IDLE` if pose is available
- `ABORT_MISSION`: transitions to `EMERGENCY` and requests landing

They can be executed by running this command in an external terminal:
```bash
ros2 topic pub /mission/request std_msgs/msg/String "{data: <command_string>}" --once
```

## Parameters

The package ships a default parameter file at [`config/missionPlanner-config.yaml`](./config/missionPlanner-config.yaml).

### Mission Control

| Parameter | Default | Meaning |
| --- | --- | --- |
| `debug_waypoint_publishing` | `true` | Publish `mission_planner/debug_waypoint` |
| `battery_check_enabled` | `false` | Ignore battery input when `false` |
| `do_mission_on_start` | `true` | Automatically leave `IDLE` once armed and ready |
| `do_mission_once` | `true` | Run only one mission cycle |
| `mission_repetitions` | `2` | Maximum number of completed mission cycles |

### Battery And Navigation Thresholds

| Parameter | Default | Meaning |
| --- | --- | --- |
| `battery_warn_threshold` | `15.0` | Below this, the planner prefers return over target approach |
| `battery_critical_threshold` | `5.0` | Below this, the planner enters `EMERGENCY` |
| `approach_distance` | `1.0` | Hover altitude above the target during approach/hover |
| `waypoint_reached_tolerance` | `0.2` | Position tolerance for waypoint completion checks |
| `takeoff_altitude` | `2.0` | Target altitude after takeoff |
| `search_altitude` | `1.5` | Altitude used during the search pattern |
| `land_altitude` | `0.0` | Reference landing-pad altitude offset |
| `target_hover_time` | `20.0` | Hover duration above the confirmed target |

### Search Area

| Parameter | Default | Meaning |
| --- | --- | --- |
| `search.center.x` | `0.0` | Declared but currently not used in the search generation logic |
| `search.center.y` | `0.0` | Declared but currently not used in the search generation logic |
| `search.start_pos.x` | `-1` | Search perimeter start point in grid coordinates |
| `search.start_pos.y` | `0` | Search perimeter start point in grid coordinates |
| `search.zero_offset_angle` | `0.0` | Additional yaw offset in degrees |
| `search.X_extent` | `1` | Search rectangle width in meters |
| `search.Y_extent` | `1` | Search rectangle height in meters |
| `search.step` | `1` | Waypoint spacing along the perimeter |
| `search.max_waypoints` | `20` | Declared and loaded, but not currently enforced in the search generation logic |
| `search.max_search_time` | `30.0` | Maximum time spent in `SEARCH` before returning |

### Mission Geometry

| Parameter | Default | Meaning |
| --- | --- | --- |
| `home.position.x` | `0.0` | Home position in map/world frame |
| `home.position.y` | `0.0` | Home position in map/world frame |
| `home.position.z` | `0.0` | Home position in map/world frame |
| `transit.position.x` | `2.0` | Transit offset from home in map/world frame |
| `transit.position.y` | `0.0` | Transit offset from home in map/world frame |
| `transit.position.z` | `0.0` | Additional transit altitude offset |

## Launch

Start the mission planner alone:

```bash
ros2 launch mission_planner mission_planner.launch.py
```

This launch file starts:

- `mission_planner_node`
- parameters from `config/missionPlanner-config.yaml`

The package also contains [`launch/simulation.launch.py`](./launch/simulation.launch.py), which starts a PX4 SITL-oriented stack for simulation setup. It launches:

- `MicroXRCEAgent`
- PX4 SITL with `gz_x500_mono_cam_aruco`
- `mavros`
- `mission_sequencer`

## Dependencies

### ROS Package Dependencies

To build and run `mission_planner_node`, the package depends on:

- `rclcpp`
- `geometry_msgs`
- `std_msgs`
- `sensor_msgs`
- `mavros_msgs`
- `mission_sequencer`
- `tf2`
- `tf2_ros`
- `tf2_geometry_msgs`
- `eigen`
- `ament_index_python`
- `launch`
- `launch_ros`

### Runtime Expectations

For the node to function correctly at runtime, the surrounding system must also provide:

- a MAVROS pose stream on `mavros/local_position/pose`
- MAVROS state and battery topics on `/mavros/state` and `/mavros/battery`
- a `mission_sequencer` node listening on `/mission_sequencer/waypoint_list` and `/autonomy/request`
- a detector publishing `geometry_msgs/msg/PointStamped` on `bbox_points_map`
- optionally, landing-pad pose estimates on `/aruco/drone_pose` for map/odom alignment near home

### Extra Dependencies For `simulation.launch.py`

The simulation launch file also assumes these external tools are already installed and available:

- `MicroXRCEAgent`
- PX4 SITL build tree
- `mavros`
- Python `pymavlink`

## Runtime Notes

- The planner does nothing until a valid pose has been received on `mavros/local_position/pose`.
- Waypoints are planned in map coordinates, but the mission sequencer consumes odom coordinates.
- Yaw is stored internally in radians and converted to degrees before publishing mission waypoints.
- The `mission_planner/current_decision_info` publisher exists in code but is not currently used.
- The config validator currently checks `search.step`, `search.max_waypoints`, and battery limits. It does not fully validate all search geometry combinations.

## Package Contents

- [`src/mission_planner.cpp`](./src/mission_planner.cpp): node implementation and state machine
- [`include/mission_planner/mission_planner.hpp`](./include/mission_planner/mission_planner.hpp): node declaration
- [`src/mission_planner_config.cpp`](./src/mission_planner_config.cpp): parameter declaration and loading
- [`include/mission_planner/landing_pad_estimator.hpp`](./include/mission_planner/landing_pad_estimator.hpp): robust map/odom alignment estimation
- [`include/mission_planner/waypoint_transformer.hpp`](./include/mission_planner/waypoint_transformer.hpp): waypoint frame conversion helpers
