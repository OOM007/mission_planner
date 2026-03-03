#pragma once

#include <rclcpp/rclcpp.hpp>

#include "mission_planner/landing_pad_estimator.hpp"
#include <tf2_ros/transform_broadcaster.h>

#include "mission_planner/waypoint_transformer.hpp"

#include "mission_planner/mission_planner_config.hpp"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include <mission_sequencer/msg/mission_waypoint_array.hpp>
#include <mission_sequencer/msg/mission_waypoint.hpp>
#include <mission_sequencer/msg/mission_request.hpp>

#include "mavros_msgs/msg/state.hpp"

#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"

#include <Eigen/Dense>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <vector>
#include <cstddef>
#include <string>
#include <thread>

class MissionPlanner : public rclcpp::Node
{
public:
  explicit MissionPlanner(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  static void declareParametersFromConfig(rclcpp::Node &node);

private:
  enum class MissionPhase
  {
    IDLE,
    TAKEOFF,
    TRANSIT,
    DESCEND,
    SEARCH,
    TARGET_DETECTED,
    APPROACH,
    HOVER,
    RETURN,
    EXIT,
    LAND,
    EMERGENCY
  };

  enum class TargetType
  {
    NONE,
    CHAIR
  };

  struct TargetInfo
  {
    Eigen::Vector3f position{0.0f, 0.0f, 0.0f};
    rclcpp::Time detection_time{};
    float confidence{0.0f};
    TargetType type{TargetType::NONE};
    float distance_to_target{0.0f};
  };

  struct MissionContext
  {
    float battery_percentage{100.0f};
    float distance_to_home{0.0f};
  };

  struct GenericWaypoint
  {
    Eigen::Vector3f position{0.0f, 0.0f, 0.0f};
    float yaw{0.0f};
    float hold_time{0.0f};
  };

  // subscriptions and publishers
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_target_detection_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_current_position_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr sub_battery_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_request_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pad_vehicle_pose_; 

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_debug_waypoint;
  rclcpp::Publisher<mission_sequencer::msg::MissionRequest>::SharedPtr pub_mission_request_;
  rclcpp::Publisher<mission_sequencer::msg::MissionWaypointArray>::SharedPtr pub_waypoint_list_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_mission_status_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_descision_info_;

  LandingPadEstimator landing_pad_estimator_{200};

  rclcpp::TimerBase::SharedPtr decision_timer_;

  MissionContext context_{};
  TargetInfo current_target_{};

  MissionPlannerConfig cfg_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;


  geometry_msgs::msg::PoseStamped current_pose_{};
  geometry_msgs::msg::PoseStamped home_position_{};
  geometry_msgs::msg::PoseStamped transit_position_{};

  bool have_pose_{false};

  MissionPhase current_phase_{MissionPhase::IDLE};
  MissionPhase previous_phase_{MissionPhase::IDLE};
  Eigen::Vector3f previous_waypoint_{0.0f, 0.0f, 0.0f};
  float d2d{0.0f}; //2d distance
  int phase_iteration_count_{0};
  rclcpp::Time phase_start_time_{};

  bool current_vehicle_armed_{false};
  bool ready_to_takeoff_{false};
  bool mission_executed_{false};
  
  //target detection and validation
  bool search_in_progress_{false};
  bool target_found_{false};
  bool target_spotted_{false};

  bool target_candidate_initialized_{false};

  Eigen::Vector3f target_candidate_position_{0.0f, 0.0f, 0.0f};
  rclcpp::Time last_processed_detection_time_{};
  rclcpp::Time target_candidate_first_seen_{};

  int target_candidate_hits_{0};
  int target_candidate_consequtive_hits_{0};
  int target_candidate_misses_{0};

  bool map_odom_initialized_{false};
  rclcpp::Time last_map_odom_update_time_{0,0,RCL_ROS_TIME};
  rclcpp::Time last_pad_pose_time_{0,0,RCL_ROS_TIME};
  rclcpp::Duration pad_valid_time_{0, 500000000}; // (time on which the pose will be valid) could be changable in config
  double max_pair_dt{0.1};
  geometry_msgs::msg::PoseStamped pad_vehicle_pose_{};
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  Eigen::Matrix4d T_map_odom_{Eigen::Matrix4d::Identity()};

  std::vector<GenericWaypoint> search_pattern_{};
  std::size_t search_index_{0};

  //needed for the executeLand()
  bool land_request_sent_{false};
  int land_aligned_cycles_{0};

  double land_desired_yaw_rad_{0.0};
  double land_yaw_threshold_deg_{10.0};
  int land_stable_cycles_required_{2};

  // callback methods
  void cbTargetDetection(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void cbCurrentPosition(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void stateCallback(const mavros_msgs::msg::State::SharedPtr msg);
  void cbBattery(const sensor_msgs::msg::BatteryState::SharedPtr msg);
  void cbMissionRequest(const std_msgs::msg::String::SharedPtr msg);

  // calibration
  void updateMapOdomAlignment();

  // FSM main loop
  void mainDecisionLoop();
  void updateMissionPhase();
  void executePhase();

  // FSM transition tree
  bool canTransitionTo(MissionPhase new_phase) const;
  void transitionToPhase(MissionPhase new_phase);

  // states
  void executeIDLE();
  void executeTAKEOFF();
  void executeTRANSIT();
  void executeDESCEND();
  void executeSEARCH();
  void executeTARGET_DETECTED();
  void executeAPPROACH();
  void executeHOVER();
  void executeRETURN();
  void executeEXIT();
  void executeLAND();
  void executeEMERGENCY();

  // publish helpers
  void publishWaypoint(const Eigen::Vector3f &position, float yaw, float holdtime);
  void publishWaypointList(const std::vector<Eigen::Vector3f> &waypoints);

  // helpers
  float getTimeSincePhaseStart() const;
  static float distance(const Eigen::Vector3f &a, const Eigen::Vector3f &b);
  static float distance2D(const Eigen::Vector3f &point1, const Eigen::Vector3f &point2);
  Eigen::Vector3f transitionedVector(const Eigen::Vector3f& vector);

  std::vector<GenericWaypoint> generateSearchPattern() const;

  static Eigen::Vector3f vectorFromPosition(const geometry_msgs::msg::PoseStamped &pose)
  {
    return Eigen::Vector3f(
        static_cast<float>(pose.pose.position.x),
        static_cast<float>(pose.pose.position.y),
        static_cast<float>(pose.pose.position.z));
  }
};