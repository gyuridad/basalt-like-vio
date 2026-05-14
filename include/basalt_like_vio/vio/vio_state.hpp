
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "basalt_like_vio/common/time_utils.hpp"
#include "basalt_like_vio/imu/imu_sample.hpp"

namespace basalt_like_vio {
namespace vio {

struct VioState {
    common::TimeNs timestamp_ns = 0;

    Eigen::Vector3d position_w_i = Eigen::Vector3d::Zero();

    Eigen::Quaterniond rotation_w_i = Eigen::Quaterniond::Identity();

    Eigen::Vector3d velocity_w_i = Eigen::Vector3d::Zero();

    imu::ImuBias bias;

    VioState() = default;

    explicit VioState(common::TimeNs timestamp_ns_)
        : timestamp_ns(timestamp_ns_) {}

    Eigen::Matrix3d rotationMatrix() const {
        return rotation_w_i.toRotationMatrix();
    }

    void normalizeRotation() {
        rotation_w_i.normalize();
    }

};


}
}



