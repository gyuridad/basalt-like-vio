
#pragma once
#include <Eigen/Core>

#include "basalt_like_vio/common/time_utils.hpp"

namespace basalt_like_vio {
namespace imu {


struct ImuSample {
    common::TimeNs timestamp_ns = 0;

    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel = Eigen::Vector3d::Zero();

    ImuSample() = default;

    ImuSample(
        common::TimeNs timestamp_ns_,
        const Eigen::Vector3d& gyro_,
        const Eigen::Vector3d& accel_)
        : timestamp_ns(timestamp_ns_),
          gyro(gyro_),
          accel(accel_) {}
};

struct ImuBias {
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();

    ImuBias() = default;

    ImuBias(
        const Eigen::Vector3d& gyro_bias_,
        const Eigen::Vector3d& accel_bias_)
        : gyro_bias(gyro_bias_),
          accel_bias(accel_bias_) {}
};

inline Eigen::Vector3d correctedGyro(
    const ImuSample& sample,
    const ImuBias& bias) {
    return sample.gyro - bias.gyro_bias;
}

inline Eigen::Vector3d correctedAccel(
    const ImuSample& sample,
    const ImuBias& bias) {
  return sample.accel - bias.accel_bias;
}

}  // namespace imu
}  // namespace basalt_like_vio


