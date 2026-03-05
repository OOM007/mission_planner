#pragma once
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <stdexcept>

struct MissionPlannerConfig {
  bool debug_waypoint_publishing;
  bool battery_check_enabled;

  bool do_mission_on_start;
  bool do_mission_once;

  int mission_repetitions;
  
  double battery_warn_threshold;
  double battery_critical_threshold;

  double approach_distance;
  double waypoint_reached_tolerance;

  double takeoff_altitude;
  double transit_altitude;
  double search_altitude;
  double land_altitude;

  double target_hover_time;
  
  double search_center_x;
  double search_center_y;
  int search_startPos_x;
  int search_startPos_y;

  double zero_offset_angle;
  int half_extent;
  int X_extent;
  int Y_extent;
  int step;
  int max_waypoints;
  double max_search_time;

  double home_x;
  double home_y;
  double home_z;

  double transit_x;
  double transit_y;
  double transit_z;

  static void declare(rclcpp::Node & node);
  static MissionPlannerConfig load(rclcpp::Node & node);

  void validate() const;
};