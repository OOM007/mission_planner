#include "mission_planner/mission_planner.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>
#include <stdexcept>

using namespace std::chrono_literals;


MissionPlanner::MissionPlanner(const rclcpp::NodeOptions & options) : 
    rclcpp::Node("mission_planner", options), 
    tf_buffer_(this->get_clock()), 
    tf_listener_(tf_buffer_, this, false)
{
    sub_target_detection_ = create_subscription<geometry_msgs::msg::PointStamped>(
        "yolo/point_of_interest", rclcpp::QoS(1), std::bind(&MissionPlanner::cbTargetDetection, this, std::placeholders::_1)
    );
    sub_current_position_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "mavros/local_position/pose", rclcpp::QoS(1).best_effort(), std::bind(&MissionPlanner::cbCurrentPosition, this, std::placeholders::_1)
    );
    sub_battery_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
    "/mavros/battery", rclcpp::SensorDataQoS(),std::bind(&MissionPlanner::cbBattery, this, std::placeholders::_1)
    );
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
        "/mavros/state", rclcpp::QoS(1), std::bind(&MissionPlanner::stateCallback, this, std::placeholders::_1));
    mission_request_ = create_subscription<std_msgs::msg::String>(
        "mission/request", rclcpp::QoS(1), std::bind(&MissionPlanner::cbMissionRequest, this, std::placeholders::_1)
    );
    sub_pad_vehicle_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/aruco_pose_local", rclcpp::QoS(10), [this](geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {pad_vehicle_pose_ = *msg; last_pad_pose_time_ = msg->header.stamp;}
    );


    pub_waypoint_list_ = create_publisher<mission_sequencer::msg::MissionWaypointArray>(
        "/mission_sequencer/waypoint_list", rclcpp::QoS(10).reliable()
    );
    pub_mission_request_ = create_publisher<mission_sequencer::msg::MissionRequest>(
        "/autonomy/request", rclcpp::QoS(1).reliable()
    );
    pub_debug_waypoint = create_publisher<geometry_msgs::msg::PoseStamped>(
        "mission_planner/debug_waypoint", rclcpp::QoS(1)
    );
    pub_mission_status_ = create_publisher<std_msgs::msg::String>(
        "mission_planner/current_status", rclcpp::QoS(1)
    );
    pub_descision_info_ = create_publisher<std_msgs::msg::String>(
        "mission_planner/current_decision_info", rclcpp::QoS(1)
    );


    tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    MissionPlannerConfig::declare(*this);
    cfg_ = MissionPlannerConfig::load(*this);
    cfg_.validate();

    home_position_.pose.position.x = cfg_.home_x;
    home_position_.pose.position.y = cfg_.home_y;
    home_position_.pose.position.z = cfg_.home_z;

    transit_position_.pose.position.x = 0;
    transit_position_.pose.position.y = 0; // declared dynamically later
    transit_position_.pose.position.z = 0;

    context_.battery_percentage = 100.0f;

    current_phase_ = MissionPhase::IDLE;
    previous_phase_ = MissionPhase::IDLE;
    previous_waypoint_ = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    d2d = 0.0f;
    phase_iteration_count_ = 0;
    phase_start_time_ = now();

    map_odom_initialized_ = false;
    T_map_odom_ = Eigen::Matrix4d::Identity();

    current_vehicle_armed_ = false;
    ready_to_takeoff_ = false;
    mission_executed_ = false;

    search_in_progress_ = false;
    target_found_ = false;

    decision_timer_ = create_wall_timer(1s, std::bind(&MissionPlanner::mainDecisionLoop, this));

    RCLCPP_INFO(get_logger(), "Mission Planner node initialized");
}


// publish helpers
void MissionPlanner::publishWaypoint(const Eigen::Vector3f & position, float yaw, float holdtime)
{
    mission_sequencer::msg::MissionWaypoint wp;

    waypointTransform::WaypointXYZYaw w;
    w.x = position.x();
    w.y = position.y();
    w.z = position.z();
    w.yaw = yaw;

    if (this->map_odom_initialized_) {
        w = waypointTransform::mapToOdom(w, this->T_map_odom_);
    }

    wp.x = w.x;
    wp.y = w.y;
    wp.z = w.z;
    wp.yaw = w.yaw;
    wp.holdtime = holdtime;

    previous_waypoint_ = Eigen::Vector3f(wp.x, wp.y, wp.z);

    mission_sequencer::msg::MissionWaypointArray msg;
    msg.waypoints.clear();
    msg.waypoints.push_back(wp);

    msg.action = mission_sequencer::msg::MissionWaypointArray::CLEAR;
    msg.idx = 0;
    msg.reference = mission_sequencer::msg::MissionWaypointArray::GLOBAL;

    pub_waypoint_list_->publish(msg);

    if (cfg_.debug_waypoint_publishing) {
        geometry_msgs::msg::PoseStamped debug_msg;
        debug_msg.header.stamp = now();
        debug_msg.header.frame_id = map_odom_initialized_ ? "odom" : "map";
        debug_msg.pose.position.x = wp.x;
        debug_msg.pose.position.y = wp.y;
        debug_msg.pose.position.z = wp.z;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, wp.yaw);
        debug_msg.pose.orientation = tf2::toMsg(q);

        pub_debug_waypoint->publish(debug_msg);
    }
}


void MissionPlanner::publishWaypointList(const std::vector<Eigen::Vector3f> & waypoints)
{
    for (const auto & waypoint : waypoints) {
        publishWaypoint(waypoint, 0.0f, 0.0f);
    }
}


// callbacks
void MissionPlanner::cbTargetDetection(const geometry_msgs::msg::PointStamped::SharedPtr msg) 
{
    const std::string target_frame = "map";

    geometry_msgs::msg::PointStamped p_in = *msg;
    geometry_msgs::msg::PointStamped p_out;

    if (p_in.header.frame_id.empty()) {
        RCLCPP_WARN(get_logger(), "Target detection has empty frame_id; ignoring");
        return;
    }

    try {
        auto use_time = (p_in.header.stamp.sec == 0 && p_in.header.stamp.nanosec == 0)
            ? tf2::TimePointZero
            : tf2::TimePoint(std::chrono::seconds(p_in.header.stamp.sec) +
                             std::chrono::nanoseconds(p_in.header.stamp.nanosec));

        auto T = tf_buffer_.lookupTransform(
            target_frame,
            p_in.header.frame_id,
            use_time,
            tf2::durationFromSec(0.2)
        );

        tf2::doTransform(p_in, p_out, T);

        current_target_.position << (float)p_out.point.x, (float)p_out.point.y, (float)p_out.point.z;

        current_target_.detection_time = now();
        target_found_ = true;

        current_target_.distance_to_target = distance(
            Eigen::Vector3f(current_pose_.pose.position.x, current_pose_.pose.position.y, current_pose_.pose.position.z),
            current_target_.position
        );

        RCLCPP_INFO(get_logger(), "Target in %s: (%.2f, %.2f, %.2f)",
                    target_frame.c_str(),
                    current_target_.position.x(), current_target_.position.y(), current_target_.position.z());

    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(get_logger(), "TF transform failed (%s -> %s): %s",
                    p_in.header.frame_id.c_str(), target_frame.c_str(), ex.what());
        return;
    }
}


void MissionPlanner::cbCurrentPosition(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_pose_ = *msg;
  have_pose_ = true;

  Eigen::Vector3f pos(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  Eigen::Vector3f home(home_position_.pose.position.x,
                       home_position_.pose.position.y,
                       home_position_.pose.position.z);
  context_.distance_to_home = distance(pos, home);

  if (target_found_) {
    current_target_.distance_to_target = distance(pos, current_target_.position);
  }
}

void MissionPlanner::stateCallback(const mavros_msgs::msg::State::SharedPtr msg) {
    current_vehicle_armed_ = msg->armed;
}

void MissionPlanner::cbBattery(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
    if (!cfg_.battery_check_enabled) {
        return;
    }

    context_.battery_percentage = msg->percentage * 100.0f;
}

void MissionPlanner::cbMissionRequest(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data == "START_MISSION") {
    if (current_phase_ == MissionPhase::IDLE && have_pose_) {
      transitionToPhase(MissionPhase::TAKEOFF);
      RCLCPP_INFO(get_logger(), "Mission start requested - transitioning to TAKEOFF");
    }
  } else if (msg->data == "ABORT_MISSION") {
    transitionToPhase(MissionPhase::EMERGENCY);
    RCLCPP_WARN(get_logger(), "Mission abort requested - returning home");
  }
}


// heleprs
static Eigen::Vector3f vectorFromPosition(const geometry_msgs::msg::PoseStamped & pose)
{
    return Eigen::Vector3f(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
}

Eigen::Vector3f MissionPlanner::transitionedVector(const Eigen::Vector3f& vector) {
    waypointTransform::WaypointXYZYaw wp;
    wp.x = (double)vector.x(); wp.y = (double)vector.y(); wp.z = (double)vector.z();

    if (map_odom_initialized_) {
        wp = waypointTransform::mapToOdom(wp, T_map_odom_);
    }

    return Eigen::Vector3f(wp.x, wp.y, wp.z);
}

float MissionPlanner::getTimeSincePhaseStart() const
{
    return (now() - phase_start_time_).seconds();
}

float MissionPlanner::distance(const Eigen::Vector3f & point1, const Eigen::Vector3f & point2) {
    return (point1 - point2).norm();
    
}

float MissionPlanner::distance2D(const Eigen::Vector3f & point1, const Eigen::Vector3f & point2) {
    Eigen::Vector2f p1_2d(point1.x(), point1.y());
    Eigen::Vector2f p2_2d(point2.x(), point2.y());
    return (p1_2d - p2_2d).norm();
}

std::vector<MissionPlanner::GenericWaypoint> MissionPlanner::generateSearchPattern() const
{
    std::vector<GenericWaypoint> pattern;

    float height = (float)cfg_.search_altitude - cfg_.land_altitude;
    float wait_time = 2.0f;

    double half_extent = cfg_.half_extent;

    int minX = -half_extent, maxX = half_extent, minY = -half_extent, maxY = half_extent;
    int step = cfg_.step;
    int startX = cfg_.search_startPos_x, startY = cfg_.search_startPos_y;
    double zero_offset_angle = 0.0;

    int translation_x = (cfg_.half_extent * (int)(std::abs(transit_position_.pose.position.x) > 0));
    int translation_y = (cfg_.half_extent * (int)(std::abs(transit_position_.pose.position.y) > 0));

    if (step <= 0) throw std::invalid_argument("step must be > 0");
    if ((maxX - minX) % step != 0 || (maxY - minY) % step != 0)
        throw std::invalid_argument("Range must be divisible by step.");

    std::vector<std::pair<int,int>> pts;
    pts.reserve(2 * ((maxX - minX) / step + (maxY - minY) / step) + 4);

    for(int x = minX; x <= maxX; x+=step) pts.push_back({x, minY});
    for(int y = minY + step; y <= maxY; y+=step) pts.push_back({maxX, y});
    for(int x = maxX - step; x >= minX; x-=step) pts.push_back({x, maxY});
    for(int y = maxY - step; y >= minY + step; y-=step) pts.push_back({minX, y});

    int start_id = -1;
    for (int i = 0; i<(int)pts.size(); i++) {
        if (pts[i].first == startX && pts[i].second == startY){start_id = i; break;}
    }

    if (start_id == -1) throw std::invalid_argument("Start point not in pattern");


    std::vector<std::pair<int,int>> seq;
    seq.reserve(pts.size());
    for (int i = 0; i < (int)pts.size(); ++i) seq.push_back(pts[(start_id + i) % pts.size()]);

    auto normAngle = [](double deg) {
        deg = std::fmod(deg, 360.0);
        if (deg < 0) deg += 360.0;
        return deg;
    };



    double cx = (minX + maxX) / 2.0;
    double cy = (minY + maxY) / 2.0;
    int limit = (int)seq.size();
    pattern.reserve(limit);

    for (int i = 0; i < (int)seq.size(); i++) {
        int x = seq[i].first, y = seq[i].second;

        double dx = cx - x;
        double dy = cy - y;

        double yaw_deg = std::atan2(dy, dx) * 180.0 / M_PI + zero_offset_angle;
        yaw_deg = normAngle(yaw_deg);

        pattern.push_back({Eigen::Vector3f(
            (float)(transit_position_.pose.position.x + translation_x + x),
            (float)(transit_position_.pose.position.y + translation_y + y),
            height
        ), yaw_deg, wait_time});
    }

    return pattern;
}


// execution logic
void MissionPlanner::executeIDLE()
{
    if (!current_vehicle_armed_ && (int)(now() - phase_start_time_).seconds() % 5 == 0) {
        RCLCPP_INFO(get_logger(), "Vehicle is disarmed - waiting for arming to start mission");
                
        mission_sequencer::msg::MissionRequest req;
        req.request = mission_sequencer::msg::MissionRequest::ARM;
        pub_mission_request_->publish(req);

        return;
    }

    if (current_vehicle_armed_ &&
        (!cfg_.do_mission_once || !mission_executed_)) {

        ready_to_takeoff_ = true;

        if (cfg_.do_mission_once) {
            mission_executed_ = true;
        }
    }

    publishWaypoint(Eigen::Vector3f(
        current_pose_.pose.position.x, 
        current_pose_.pose.position.y, 
        current_pose_.pose.position.z), 0.0f, 0.0f);
}

void MissionPlanner::executeTAKEOFF()
{
    if (phase_iteration_count_ >= 1) {
        return;
    }

    if (phase_iteration_count_ == 0) {
        float takeoff_z = (float)(cfg_.takeoff_altitude - cfg_.land_altitude);
        transit_position_.pose.position.x = home_position_.pose.position.x + (float)cfg_.transit_x;
        transit_position_.pose.position.y = home_position_.pose.position.y + (float)cfg_.transit_y;
        transit_position_.pose.position.z = home_position_.pose.position.z + takeoff_z + (float)cfg_.transit_z;

    }

    publishWaypoint(Eigen::Vector3f(
        current_pose_.pose.position.x, 
        current_pose_.pose.position.y, 
        current_pose_.pose.position.z + (float)(cfg_.takeoff_altitude - cfg_.land_altitude)), 0.0f, 0.0f);
    
    mission_sequencer::msg::MissionRequest req;
    req.request = mission_sequencer::msg::MissionRequest::TAKEOFF;
    pub_mission_request_->publish(req);

    phase_iteration_count_++;
}

void MissionPlanner::executeTRANSIT()
{
    if (phase_iteration_count_ >= 1) {
        return;
    }

    publishWaypoint(Eigen::Vector3f(
        transit_position_.pose.position.x, 
        transit_position_.pose.position.y, 
        transit_position_.pose.position.z), 0.0f, 0.0f);

    phase_iteration_count_++;
}

void MissionPlanner::executeDESCEND()
{
    if (phase_iteration_count_ >= 1) {
        return;
    }

    publishWaypoint(Eigen::Vector3f(
        transit_position_.pose.position.x, 
        transit_position_.pose.position.y, 
        (float)(cfg_.search_altitude - cfg_.land_altitude)), 0.0f, 0.0f);

    phase_iteration_count_++;  
}


void MissionPlanner::executeSEARCH()
{
    if (!search_in_progress_) {
        search_pattern_ = generateSearchPattern();
        search_index_ = 0;
        search_in_progress_ = true;

        if (!search_pattern_.empty()) {
            publishWaypoint(search_pattern_[0].position,
                            search_pattern_[0].yaw,
                            search_pattern_[0].hold_time);
        }
        return;
    }

    if (target_found_) return;
    if (search_index_ >= search_pattern_.size()) {
        search_in_progress_ = false;
        return;
    }

    float d2d = distance2D(vectorFromPosition(current_pose_), previous_waypoint_);
    if (d2d > cfg_.waypoint_reached_tolerance) {
        return;
    }

    search_index_++;
    if (search_index_ < search_pattern_.size()) {
        const auto & next = search_pattern_[search_index_];
        publishWaypoint(next.position, next.yaw, next.hold_time);
    } else {
        search_in_progress_ = false;
    }
}


void MissionPlanner::executeTARGET_DETECTED()
{
    if (phase_iteration_count_ >= 1) {
        return;
    }

    //TODO: make a confidence check

    publishWaypoint(Eigen::Vector3f(
        current_pose_.pose.position.x, 
        current_pose_.pose.position.y, 
        current_pose_.pose.position.z), 0.0f, 0.0f);

    if (phase_iteration_count_++ % 5 == 0) {
        RCLCPP_INFO(get_logger(), "Analyzing target: confidence=%.2f, distance=%.1f",
            current_target_.confidence, current_target_.distance_to_target);
    }
}

void MissionPlanner::executeAPPROACH()
{
    if (phase_iteration_count_ >= 1) {
        return;
    }

    publishWaypoint(Eigen::Vector3f(
        current_target_.position.x(), 
        current_target_.position.y(), 
        (float)(cfg_.approach_distance)), 0.0f, 0.0f);

    phase_iteration_count_++;
}

void MissionPlanner::executeRETURN()
{
    if (phase_iteration_count_ >= 1) {
        return;
    }

    publishWaypoint(Eigen::Vector3f(
        transit_position_.pose.position.x, 
        transit_position_.pose.position.y, 
        transit_position_.pose.position.z), 180.0f, 0.0f);

    phase_iteration_count_++;
}

void MissionPlanner::executeEXIT()
{
    if (phase_iteration_count_ >= 1) {
        return;
    }
    // home position needs to be adjustable if drift is present
    publishWaypoint(Eigen::Vector3f(
        home_position_.pose.position.x, 
        home_position_.pose.position.y, 
        home_position_.pose.position.z + (float)(cfg_.takeoff_altitude - cfg_.land_altitude)), 180.0f, 0.0f);

    phase_iteration_count_++;
}

void MissionPlanner::executeLAND()
{
    if (!have_pose_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "LAND: no pose yet");
        return;
    }

    publishWaypoint(
        Eigen::Vector3f(
            current_pose_.pose.position.x,
            current_pose_.pose.position.y,
            current_pose_.pose.position.z),
        static_cast<float>(land_desired_yaw_rad_),
        0.0f
    );

    tf2::Quaternion q(
        current_pose_.pose.orientation.x,
        current_pose_.pose.orientation.y,
        current_pose_.pose.orientation.z,
        current_pose_.pose.orientation.w
    );

    tf2::Matrix3x3 m(q);
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    m.getRPY(roll, pitch, yaw);

    double yaw_err = land_desired_yaw_rad_ - yaw;
    if (yaw_err > M_PI)  yaw_err -= 2.0 * M_PI;
    if (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;

    const double yaw_err_deg = std::abs(yaw_err) * (180.0 / M_PI);

    if (yaw_err_deg <= land_yaw_threshold_deg_) {
        land_aligned_cycles_++;
    } else {
        land_aligned_cycles_ = 0;
    }

    if (!land_request_sent_ && land_aligned_cycles_ >= land_stable_cycles_required_) {
        mission_sequencer::msg::MissionRequest req_land;
        req_land.request = mission_sequencer::msg::MissionRequest::LAND;
        pub_mission_request_->publish(req_land);

        land_request_sent_ = true;
        RCLCPP_INFO(get_logger(), "LAND request sent (yaw error %.2f deg)", yaw_err_deg);
    }

    target_found_ = false;
}


void MissionPlanner::executeEMERGENCY()
{
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "EMERGENCY: Battery critically low (%.1f%%) - executing emergency landing", context_.battery_percentage);

    //TODO: Implement publishing of landing waypoint
}


// map-odom alignment using aruco marker observations
void MissionPlanner::updateMapOdomAlignment()
{
    const bool in_calib_window =
        (current_phase_ == MissionPhase::EXIT) ||
        (current_phase_ == MissionPhase::LAND);

    if (!in_calib_window) {
        return;
    }

    const auto t_now = now();

    const bool pad_fresh =
        (last_pad_pose_time_.nanoseconds() != 0) &&
        ((t_now - this->last_pad_pose_time_) <= this->pad_valid_time_);

    if (!(have_pose_ && pad_fresh)) {
        return;
    }

    if (std::abs((t_now - this->last_pad_pose_time_).seconds()) > this->max_pair_dt) {
        return;
    }

    landing_pad_estimator_.addSample(pad_vehicle_pose_.pose, current_pose_.pose);

    // limiting the estimate updates (2 Hz)
    if (last_map_odom_update_time_.nanoseconds() != 0) {
        const double dt = (t_now - this->last_map_odom_update_time_).seconds();
        if (dt < 0.5) {
        return;
        }
    }
    last_map_odom_update_time_ = t_now;

    auto Topt = landing_pad_estimator_.estimate(30, 0.10, 8.0);
    if (!Topt.has_value()) {
        return;
    }

    const Eigen::Matrix4d T_meas = *Topt;

    if (!this->map_odom_initialized_) {
        T_map_odom_ = T_meas;
        map_odom_initialized_ = true;
        return;
    }

    const double alpha = 0.15;

    // translation
    Eigen::Vector3d t_f(T_map_odom_(0,3), T_map_odom_(1,3), T_map_odom_(2,3));
    Eigen::Vector3d t_m(T_meas(0,3),     T_meas(1,3),     T_meas(2,3));
    Eigen::Vector3d t_new = (1.0 - alpha) * t_f + alpha * t_m;

    auto yawFromT = [](const Eigen::Matrix4d& T){
        return std::atan2(T(1,0), T(0,0));
    };
    const double yaw_f = yawFromT(T_map_odom_);
    const double yaw_m = yawFromT(T_meas);
    const double dy = std::atan2(std::sin(yaw_m - yaw_f), std::cos(yaw_m - yaw_f));
    const double yaw_new = yaw_f + alpha * dy;

    // transform matrix
    Eigen::Matrix4d T_new = Eigen::Matrix4d::Identity();
    T_new(0,0) = std::cos(yaw_new); T_new(0,1) = -std::sin(yaw_new);
    T_new(1,0) = std::sin(yaw_new); T_new(1,1) =  std::cos(yaw_new);
    T_new(0,3) = t_new.x();
    T_new(1,3) = t_new.y();
    T_new(2,3) = t_new.z();
    T_map_odom_ = T_new;
}


//FSM definitions and main loop
void MissionPlanner::mainDecisionLoop() 
{
    if (!have_pose_) {
        RCLCPP_WARN(get_logger(), "No pose information yet - cannot make decisions");
        return;
    }

    updateMissionPhase();
    executePhase();

    updateMapOdomAlignment();

    std_msgs::msg::String status_msg;
    switch (current_phase_) {
        case MissionPhase::IDLE: status_msg.data = "IDLE"; break;
        case MissionPhase::TAKEOFF: status_msg.data = "TAKEOFF"; break;
        case MissionPhase::TRANSIT: status_msg.data = "TRANSIT"; break;
        case MissionPhase::DESCEND: status_msg.data = "DESCEND"; break;
        case MissionPhase::SEARCH: status_msg.data = "SEARCH"; break;
        case MissionPhase::TARGET_DETECTED: status_msg.data = "TARGET_DETECTED"; break;
        case MissionPhase::APPROACH: status_msg.data = "APPROACH"; break;
        case MissionPhase::HOVER: status_msg.data = "HOVER"; break;
        case MissionPhase::RETURN: status_msg.data = "RETURN"; break;
        case MissionPhase::EXIT: status_msg.data = "EXIT"; break;
        case MissionPhase::LAND: status_msg.data = "LAND"; break;
        case MissionPhase::EMERGENCY: status_msg.data = "EMERGENCY"; break;
        default: status_msg.data = "UNKNOWN"; break;
    }

    pub_mission_status_->publish(status_msg);
}

bool MissionPlanner::canTransitionTo(MissionPhase new_phase) const
{
    switch (current_phase_) {
        case MissionPhase::IDLE:
            return new_phase == MissionPhase::SEARCH ||
                    new_phase == MissionPhase::TAKEOFF ||
                    new_phase == MissionPhase::EMERGENCY;

        case MissionPhase::TAKEOFF:
            return new_phase == MissionPhase::TRANSIT ||
                    new_phase == MissionPhase::EMERGENCY;
        
        case MissionPhase::TRANSIT:
            return new_phase == MissionPhase::DESCEND ||
                    new_phase == MissionPhase::EMERGENCY;
        
        case MissionPhase::SEARCH:
            return new_phase == MissionPhase::APPROACH ||
                    new_phase == MissionPhase::RETURN || 
                    new_phase == MissionPhase::TARGET_DETECTED ||
                    new_phase == MissionPhase::EMERGENCY;
        
        case MissionPhase::TARGET_DETECTED:
            return new_phase == MissionPhase::APPROACH ||
                    new_phase == MissionPhase::SEARCH ||
                    new_phase == MissionPhase::EMERGENCY;

        case MissionPhase::DESCEND:
            return new_phase == MissionPhase::SEARCH ||
                    new_phase == MissionPhase::EMERGENCY; 
        
        case MissionPhase::APPROACH:
            return new_phase == MissionPhase::HOVER ||
                    new_phase == MissionPhase::SEARCH ||
                    new_phase == MissionPhase::EMERGENCY;
        
        case MissionPhase::HOVER:
            return new_phase == MissionPhase::RETURN ||
                    new_phase == MissionPhase::EMERGENCY;
        
        case MissionPhase::RETURN:
            return new_phase == MissionPhase::EXIT ||
                    new_phase == MissionPhase::EMERGENCY;
        
        case MissionPhase::EXIT:
            return new_phase == MissionPhase::LAND ||
                    new_phase == MissionPhase::EMERGENCY;

        case MissionPhase::LAND:
            return new_phase == MissionPhase::IDLE ||
                    new_phase == MissionPhase::EMERGENCY;

        default:
            return false;
    }
}

void MissionPlanner::transitionToPhase(MissionPhase new_phase) 
{
    if (new_phase == MissionPhase::TAKEOFF || new_phase == MissionPhase::IDLE) {
        landing_pad_estimator_.clear();
        ready_to_takeoff_ = false;
    }

    // reseting params for landing
    if (new_phase == MissionPhase::LAND) {
        land_request_sent_ = false;
        land_aligned_cycles_ = 0;
    }

    if (canTransitionTo(new_phase)) {
        previous_phase_ = current_phase_;
        current_phase_ = new_phase;
        phase_iteration_count_ = 0;
        phase_start_time_ = now();

        RCLCPP_INFO(get_logger(), "Transitioned from %d to %d", static_cast<int>(previous_phase_), static_cast<int>(current_phase_));
    } else {
        RCLCPP_WARN(get_logger(), "Invalid transition from %d to %d", static_cast<int>(current_phase_), static_cast<int>(new_phase));
    }
}

void MissionPlanner::updateMissionPhase() 
{
    if (context_.battery_percentage < (float)cfg_.battery_critical_threshold) {
        if (current_phase_ != MissionPhase::EMERGENCY) {
            RCLCPP_ERROR(get_logger(), "Battery is critically low (%.1f%%) - transitioning to EMERGENCY", context_.battery_percentage);
            transitionToPhase(MissionPhase::EMERGENCY);
        }
        return;
    }

    if (cfg_.do_mission_on_start && current_phase_ == MissionPhase::IDLE && current_vehicle_armed_ && ready_to_takeoff_) {
        transitionToPhase(MissionPhase::TAKEOFF);
        return;
    }

    switch (current_phase_) {
        case MissionPhase::IDLE:
            break;

        case MissionPhase::TAKEOFF:
            if (current_pose_.pose.position.z > (cfg_.takeoff_altitude - cfg_.land_altitude) - cfg_.waypoint_reached_tolerance) {
                 transitionToPhase(MissionPhase::TRANSIT);
                 return;
            }
            break;

        case MissionPhase::TRANSIT:
            d2d = MissionPlanner::distance2D(vectorFromPosition(current_pose_), previous_waypoint_);
            if (d2d < cfg_.waypoint_reached_tolerance) {
                transitionToPhase(MissionPhase::DESCEND);
                return;
            }
            break;

        case MissionPhase::DESCEND:
        {
            float descend_altitude = (float)(cfg_.search_altitude - cfg_.land_altitude);
            if (std::abs(current_pose_.pose.position.z - descend_altitude) < cfg_.waypoint_reached_tolerance) {
                 transitionToPhase(MissionPhase::SEARCH);
                 return;
            }
            break;
        }
        case MissionPhase::SEARCH:
            if (target_found_) {
                transitionToPhase(MissionPhase::TARGET_DETECTED);
            } else if (getTimeSincePhaseStart() > cfg_.max_search_time) {
                transitionToPhase(MissionPhase::RETURN);
            }
            break;

        case MissionPhase::TARGET_DETECTED:
            if (context_.battery_percentage > (float)cfg_.battery_warn_threshold){
                transitionToPhase(MissionPhase::APPROACH);
            } else {
                transitionToPhase(MissionPhase::RETURN);
            }
            break;

        case MissionPhase::APPROACH:
            d2d = MissionPlanner::distance2D(vectorFromPosition(current_pose_), previous_waypoint_);
            if (d2d < cfg_.waypoint_reached_tolerance) {
                transitionToPhase(MissionPhase::HOVER);
            } else if (context_.battery_percentage < (float)cfg_.battery_warn_threshold) {
                transitionToPhase(MissionPhase::RETURN);
            }
            break;
        
        case MissionPhase::HOVER:
            if (getTimeSincePhaseStart() > (float)cfg_.target_hover_time) {
                transitionToPhase(MissionPhase::RETURN);
            } else if (context_.battery_percentage < (float)cfg_.battery_warn_threshold) {
                transitionToPhase(MissionPhase::RETURN);
            }
            break;

        case MissionPhase::RETURN:
            d2d = MissionPlanner::distance2D(vectorFromPosition(current_pose_), previous_waypoint_);
            if (d2d < cfg_.waypoint_reached_tolerance) {
                transitionToPhase(MissionPhase::EXIT);
            } else if (context_.battery_percentage < (float)cfg_.battery_critical_threshold) {
                transitionToPhase(MissionPhase::EMERGENCY);
            }
            break;

        case MissionPhase::EXIT:
            d2d = MissionPlanner::distance2D(vectorFromPosition(current_pose_), previous_waypoint_);
            if (d2d < cfg_.waypoint_reached_tolerance) {
                transitionToPhase(MissionPhase::LAND);
            } else if (context_.battery_percentage < (float)cfg_.battery_critical_threshold) {
                transitionToPhase(MissionPhase::EMERGENCY);
            }
            break;

        case MissionPhase::LAND:
            if (getTimeSincePhaseStart() > 10.0f) {
                transitionToPhase(MissionPhase::IDLE);
                RCLCPP_INFO(get_logger(), "Mission complete, back to IDLE");
            }
            break;
        
        default:
            break;
        
    }
}

void MissionPlanner::executePhase()
{
    switch(current_phase_) {
        case MissionPhase::IDLE: executeIDLE(); break;
        case MissionPhase::TAKEOFF: executeTAKEOFF(); break;
        case MissionPhase::TRANSIT: executeTRANSIT(); break;
        case MissionPhase::DESCEND: executeDESCEND(); break;
        case MissionPhase::SEARCH: executeSEARCH(); break;
        case MissionPhase::TARGET_DETECTED: executeTARGET_DETECTED(); break;
        case MissionPhase::APPROACH: executeAPPROACH(); break;
        case MissionPhase::HOVER: executeIDLE(); break; // hover and idle has the same implementation for now
        case MissionPhase::RETURN: executeRETURN(); break;
        case MissionPhase::EXIT: executeEXIT(); break;
        case MissionPhase::EMERGENCY: executeEMERGENCY(); break;
        case MissionPhase::LAND: executeLAND(); break;
        default: break;
    }
}





int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionPlanner>());
  rclcpp::shutdown();
  return 0;
}