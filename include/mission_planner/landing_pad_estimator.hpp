#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <optional>
#include <vector>

// One buffered observation of the map->odom correction.
// The estimator keeps only translation and yaw because the planner assumes
// a planar vehicle motion model for the final home-landing alignment.
struct Sample {
    Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
    double yaw{0.0};
};

static inline double angleDiffRad(double a, double b) {
    return std::remainder(a - b, 2.0 * M_PI);
}

static inline double median(std::vector<double>& v) {
    const std::size_t n = v.size();
    std::nth_element(v.begin(), v.begin() + n / 2, v.end());
    double m = v[n / 2];

    if (n % 2 == 0) {
        auto max_it = std::max_element(v.begin(), v.begin() + n / 2);
        m = (*max_it + m) / 2.0;
    }
    return m;
}

static inline double rot_to_yaw(const Eigen::Matrix3d& rot) {
    return std::atan2(rot.col(0).y(), rot.col(0).x());
}

static inline Eigen::Matrix3d yaw_to_rot(double yaw) {
    Eigen::Matrix3d rot;
    rot << std::cos(yaw), -std::sin(yaw), 0.0,
           std::sin(yaw),  std::cos(yaw), 0.0,
           0.0,            0.0,           1.0;
    return rot;
}

template <typename poseT>
static inline Eigen::Isometry3d poseToIsometry(const poseT& pose) {
    Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();

    iso.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);

    Eigen::Quaterniond q(pose.orientation.w,
                         pose.orientation.x,
                         pose.orientation.y,
                         pose.orientation.z);
    q.normalize();
    iso.linear() = q.toRotationMatrix();

    return iso;
}

// Estimates a stable map->odom transform from repeated landing-pad observations.
//
// For each paired observation the estimator computes:
//   T_map_odom = T_map_base * inverse(T_odom_base)
class LandingPadEstimator {
public:
    explicit LandingPadEstimator(std::size_t buffer_size = 200)
        : buffer_size_(buffer_size) {}

    template <typename poseT>
    void addSample(const poseT& pose_map_base, const poseT& pose_odom_base) {
        const Eigen::Isometry3d T_map_base  = poseToIsometry(pose_map_base);
        const Eigen::Isometry3d T_odom_base = poseToIsometry(pose_odom_base);

        const Eigen::Isometry3d T_map_odom = T_map_base * T_odom_base.inverse();

        Sample sample;
        sample.translation = T_map_odom.translation();
        sample.yaw = rot_to_yaw(T_map_odom.linear());

        push_sample(sample);
    }

    std::optional<Eigen::Matrix4d> estimate(std::size_t min_samples = 10,
                                           double max_deviation_xy = 0.1,
                                           double max_deviation_yaw = 0.1) const {
        if (samples_.size() < min_samples) {
            return std::nullopt;
        }

        std::vector<double> xs, ys, zs, yaws;
        xs.reserve(samples_.size());
        ys.reserve(samples_.size());
        zs.reserve(samples_.size());
        yaws.reserve(samples_.size());

        for (const auto& sample : samples_) {
            xs.push_back(sample.translation.x());
            ys.push_back(sample.translation.y());
            zs.push_back(sample.translation.z());
            yaws.push_back(sample.yaw);
        }

        // median is used as a center reference before enough avarages were calculated.
        const double mx = median(xs);
        const double my = median(ys);
        const double mz = median(zs);

        double sy = 0.0;
        double cy = 0.0;
        for (double angle : yaws) {
            sy += std::sin(angle);
            cy += std::cos(angle);
        }
        const double myaw = std::atan2(sy, cy);

        std::vector<Sample> kept_samples;
        kept_samples.reserve(samples_.size());

        for (const auto& sample : samples_) {
            const double dx = sample.translation.x() - mx;
            const double dy = sample.translation.y() - my;
            const double dz = sample.translation.z() - mz;

            // shift rejecting gate, based on XY plane
            if (std::hypot(dx, dy) > max_deviation_xy) {
                continue;
            }

            if (std::abs(angleDiffRad(sample.yaw, myaw)) > max_deviation_yaw) {
                continue;
            }

            kept_samples.push_back(sample);
        }

        if (kept_samples.size() < min_samples) {
            return std::nullopt;
        }

        Eigen::Vector3d t_sum = Eigen::Vector3d::Zero();
        for (const auto& sample : kept_samples) {
            t_sum += sample.translation;
        }
        const Eigen::Vector3d t_average = t_sum / static_cast<double>(kept_samples.size());

        sy = 0.0;
        cy = 0.0;
        for (const auto& sample : kept_samples) {
            sy += std::sin(sample.yaw);
            cy += std::cos(sample.yaw);
        }
        const double yaw_average = std::atan2(sy, cy);

        Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
        out.block<3, 3>(0, 0) = yaw_to_rot(yaw_average);
        out.block<3, 1>(0, 3) = t_average;

        return out;
    }

    void clear() { samples_.clear(); }

private:
    void push_sample(const Sample& sample) {
        if (samples_.size() >= buffer_size_) {
            samples_.erase(samples_.begin());
        }
        samples_.push_back(sample);
    }

    std::size_t buffer_size_;
    std::vector<Sample> samples_;
};
