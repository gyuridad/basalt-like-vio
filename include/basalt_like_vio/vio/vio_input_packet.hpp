
#pragma once

#include <memory>    
#include <vector>

#include <Eigen/Core>

#include "basalt_like_vio/common/time_utils.hpp"
#include "basalt_like_vio/imu/imu_sample.hpp"


namespace basalt_like_vio {
namespace vio {


struct ImageFrame {
    common::TimeNs timestamp_ns = 0;

    int width = 0;
    int height = 0;

    std::vector<uint8_t> gray_data;

    ImageFrame() = default;

    ImageFrame(
        common::TimeNs timestamp_ns_,
        int width_,
        int height_,
        const std::vector<uint8_t>& gray_data_)
        : timestamp_ns(timestamp_ns_),
          width(width_),
          height(height_),
          gray_data(gray_data_) {}
};

/**
 * @brief VIO 한 스텝에 들어가는 입력 패킷
 *
 * Basalt 계열 VIO의 핵심 흐름은 보통 이렇습니다.
 *
 *   이미지 1장
 *   + 그 이미지 시각까지 누적된 IMU measurements
 *   → Optical Flow
 *   → IMU Preintegration
 *   → VIO Estimator Update
 *
 * 그래서 하나의 이미지 프레임과,
 * 해당 이미지 사이 구간의 IMU 샘플들을 하나의 packet으로 묶습니다.
 */
struct VioInputPacket {
    common::TimeNs timestamp_ns = 0;

    std::shared_ptr<ImageFrame> image;

    std::vector<imu::ImuSample> imu_samples;

    VioInputPacket() = default;

    VioInputPacket(
        common::TimeNs timestamp_ns_,
        const std::shared_ptr<ImageFrame>& image_,
        const std::vector<imu::ImuSample>& imu_samples_)
        : timestamp_ns(timestamp_ns_),
          image(image_),
          imu_samples(imu_samples_) {}

    // 이 함수는 VioInputPacket 안에 이미지가 있는지 없는지 확인합니다.
    bool hasImage() const {
        return image != nullptr;  // 이미지가 실제로 들어있다는 의미임 
    }

    // 이 함수는 VioInputPacket 안에 IMU 샘플이 하나라도 있는지 확인합니다.
    bool hasImuSamples() const {
        return !imu_samples.empty();  // imu_samples가 비어 있지 않으면 true
    }
    

};




