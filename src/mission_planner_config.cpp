#include "mission_planner/mission_planner_config.hpp"

void MissionPlannerConfig::declare(rclcpp::Node & node)
{
  node.declare_parameter<bool>("debug_waypoint_publishing", false);
  node.declare_parameter<bool>("battery_check_enabled", true);

  node.declare_parameter<bool>("do_mission_on_start", false);
  node.declare_parameter<bool>("do_mission_once", true);

   node.declare_parameter<int>("mission_repetitions", 1);
  
  node.declare_parameter<double>("battery_warn_threshold", 15.0);
  node.declare_parameter<double>("battery_critical_threshold", 5.0);

  node.declare_parameter<double>("approach_distance", 2.0);
  node.declare_parameter<double>("waypoint_reached_tolerance", 0.5);

  node.declare_parameter<double>("takeoff_altitude", 4.0);
  node.declare_parameter<double>("search_altitude", 2.5);
  node.declare_parameter<double>("transit_altitude", 4.0);
  node.declare_parameter<double>("land_altitude", 3.0);

  node.declare_parameter<double>("target_hover_time", 20.0);

  node.declare_parameter<double>("search.center.x", 0.0);
  node.declare_parameter<double>("search.center.y", 0.0);

  node.declare_parameter<int>("search.start_pos.x", 0);
  node.declare_parameter<int>("search.start_pos.y", -2);

  node.declare_parameter<double>("search.zero_offset_angle", 0.0);
  node.declare_parameter<int>("search.half_extent", 2);
  node.declare_parameter<int>("search.X_extent", 1);
  node.declare_parameter<int>("search.Y_extent", 1);
  node.declare_parameter<int>("search.step", 1);
  node.declare_parameter<int>("search.max_waypoints", 20);
  node.declare_parameter<double>("search.max_search_time", 60.0);

  node.declare_parameter<double>("home.position.x", 0.0);
  node.declare_parameter<double>("home.position.y", 0.0);
  node.declare_parameter<double>("home.position.z", 0.0);

  node.declare_parameter<double>("transit.position.x", 2.0);
  node.declare_parameter<double>("transit.position.y", 0.0);
  node.declare_parameter<double>("transit.position.z", 0.0);
}


MissionPlannerConfig MissionPlannerConfig::load(rclcpp::Node & node)
{
    MissionPlannerConfig cfg;

    node.get_parameter("debug_waypoint_publishing", cfg.debug_waypoint_publishing);
    node.get_parameter("battery_check_enabled", cfg.battery_check_enabled);
    node.get_parameter("do_mission_on_start", cfg.do_mission_on_start);
    node.get_parameter("do_mission_once", cfg.do_mission_once);
    node.get_parameter("mission_repetitions", cfg.mission_repetitions);
    node.get_parameter("battery_warn_threshold", cfg.battery_warn_threshold);
    node.get_parameter("battery_critical_threshold", cfg.battery_critical_threshold);
    node.get_parameter("approach_distance", cfg.approach_distance);
    node.get_parameter("waypoint_reached_tolerance", cfg.waypoint_reached_tolerance);
    node.get_parameter("takeoff_altitude", cfg.takeoff_altitude);
    node.get_parameter("search_altitude", cfg.search_altitude);
    node.get_parameter("transit_altitude", cfg.transit_altitude);
    node.get_parameter("land_altitude", cfg.land_altitude);
    node.get_parameter("target_hover_time", cfg.target_hover_time);
    node.get_parameter("search.center.x", cfg.search_center_x);
    node.get_parameter("search.center.y", cfg.search_center_y);
    node.get_parameter("search.start_pos.x", cfg.search_startPos_x);
    node.get_parameter("search.start_pos.y", cfg.search_startPos_y);
    node.get_parameter("search.zero_offset_angle", cfg.zero_offset_angle);
    node.get_parameter("search.half_extent", cfg.half_extent);
    node.get_parameter("search.X_extent", cfg.X_extent);
    node.get_parameter("search.Y_extent", cfg.Y_extent);
    node.get_parameter("search.step", cfg.step);
    node.get_parameter("search.max_waypoints", cfg.max_waypoints);
    node.get_parameter("search.max_search_time", cfg.max_search_time);
    node.get_parameter("home.position.x", cfg.home_x);
    node.get_parameter("home.position.y", cfg.home_y);
    node.get_parameter("home.position.z", cfg.home_z);
    node.get_parameter("transit.position.x", cfg.transit_x);
    node.get_parameter("transit.position.y", cfg.transit_y);
    node.get_parameter("transit.position.z", cfg.transit_z);

    return cfg;
}

void MissionPlannerConfig::validate() const
{
    auto require_positive = [](double v, const char* name) {
        if (v <= 0.0) throw std::invalid_argument(std::string(name) + " must be > 0");
    };

    require_positive(step, "search.step");
    require_positive(half_extent, "search.half_extent");
    
    if (max_waypoints <= 0) throw std::invalid_argument("search.max_waypoints must be > 0");

    if (battery_warn_threshold < 0 || battery_warn_threshold > 100)
        throw std::invalid_argument("battery_warn_threshold must be between [0..100]");
}