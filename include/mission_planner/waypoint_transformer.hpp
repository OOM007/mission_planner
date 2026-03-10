#pragma once
#include <Eigen/Dense>
#include <cmath>

namespace waypointTransform
{
    struct WaypointXYZYaw {
        double x{0.0}; double y{0.0}; double z{0.0}; double yaw{0.0};
    };

    inline double wrapPI(double a) {
        return std::atan2(std::sin(a), std::cos(a));
    };

    // Build a homogeneous transform from a planar pose. Roll and pitch are
    // intentionally omitted because the planner only works with yaw waypoints.
    inline Eigen::Matrix4d T_from_xyz_yaw(double x, double y, double z, double yaw) {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        const double c = std::cos(yaw);
        const double s = std::sin(yaw);

        T(0,0)=c ; T(0,1)=-s ;
        T(1,0)=s ; T(1,1)=c ;
        T(2,2)=1.0;

        T(0,3)=x;
        T(1,3)=y;
        T(2,3)=z;

        return T;
    }

    // Recover the planner waypoint representation from a homogeneous transform.
    inline WaypointXYZYaw xyz_yaw_from_T(const Eigen::Matrix4d& T) {
        WaypointXYZYaw w;
        w.x = T(0,3);
        w.y = T(1,3);
        w.z = T(2,3);

        w.yaw = std::atan2(T(1,0), T(0,0));
        w.yaw = wrapPI(w.yaw);
        
        return w;
    }

    // T_map_odom is the transform from odom to map.
    inline WaypointXYZYaw mapToOdom(const WaypointXYZYaw& wp_map, const Eigen::Matrix4d& T_map_odom) {
        const Eigen::Matrix4d T_map_wp = T_from_xyz_yaw(wp_map.x, wp_map.y, wp_map.z, wp_map.yaw);
        const Eigen::Matrix4d T_odom_wp = T_map_odom.inverse() * T_map_wp;

        return xyz_yaw_from_T(T_odom_wp);
    }
    
    inline WaypointXYZYaw odomToMap(const WaypointXYZYaw& wp_odom, const Eigen::Matrix4d& T_map_odom) {
        const Eigen::Matrix4d T_odom_wp = T_from_xyz_yaw(wp_odom.x, wp_odom.y, wp_odom.z, wp_odom.yaw);
        const Eigen::Matrix4d T_map_wp = T_map_odom * T_odom_wp;

        return xyz_yaw_from_T(T_map_wp);
    }
}
