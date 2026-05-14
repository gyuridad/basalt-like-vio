#include <cmath>
#include <deque>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "basalt_like_vio/imu/imu_preintegrator.hpp"

using basalt_like_vio::common::TimeNs;
using basalt_like_vio::imu::ImuBias;
using basalt_like_vio::imu::ImuPreintegrator;
using basalt_like_vio::imu::ImuPreintegratorOptions;
using basalt_like_vio::imu::ImuSample;
using basalt_like_vio::imu::PreintegrationResult;


class IsaacImuPreintegrationNode : public rclcpp::Node {

public:
    IsaacImuPreintegrationNode() : Node("isaac_imu_preintegration_node") {

        imu_topic_ = this->declare_parameter<std::string>("imu_topic", "/chassis/imu");
        window_sec_ = this->declare_parameter<double>("window_sec", 1.0);  // IMU 적분 시간 범위를 정하는 값
            // 이 값은 최근 몇 초 동안의 IMU 데이터만 모아서 preintegration할지 정해.
        print_every_sec_ = this->declare_parameter<double>("print_every_sec", 1.0);
            // 이 값은 preintegration 결과를 몇 초마다 한 번씩 출력할지 정해.

        ImuPreintegratorOptions options;
        options.gravity_w = Eigen::Vector3d(0.0, 0.0, -9.81);

        options.noise.gyro_noise_std = 1e-3;
        options.noise.accel_noise_std = 1e-2;
        options.noise.gyro_bias_rw_std = 1e-5;
        options.noise.accel_bias_rw_std = 1e-4;

        preintegrator_ = ImuPreintegrator(options);

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_,
            rclcpp::SensorDataQoS(),  // 센서 데이터용 QoS
                // 센서 데이터는 최신 값이 중요하니까, IMU처럼 빠르게 들어오는 데이터에 맞는 통신 설정을 사용하겠다 의미임 
            std::bind(&IsaacImuPreintegrationNode::onImu, this, std::placeholders::_1)
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Subscribing IMU topic: %s",
            imu_topic_.c_str()
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Integration window: %.3f sec",
            window_sec_
        );
    }

private:
    static TimeNs stampToNs(const builtin_interfaces::msg::Time& stamp) {
        return static_cast<TimeNs>(stamp.sec) * 1000000000LL +
               static_cast<TimeNs>(stamp.nanosec);
    }

    ImuSample convertMsgToSample(const sensor_msgs::msg::Imu& msg) {
        ImuSample sample;

        sample.timestamp_ns = stampToNs(msg.header.stamp);

        sample.gyro = Eigen::Vector3d(
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        );

        sample.accel = Eigen::Vector3d(
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        );
        return sample;
    }

    void onImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
        ImuSample sample = convertMsgToSample(*msg);

        if (sample.timestamp_ns <= 0) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "IMU timestamp is zero or invalid."
            );
            return;
        }

        imu_buffer_.push_back(sample);    // push_back은 컨테이너의 맨 뒤에 값을 추가하는 함수

        const TimeNs latest_time_ns = imu_buffer_.back().timestamp_ns;
        const TimeNs window_ns = static_cast<TimeNs>(window_sec_ * 1000000000.0);

        while (!imu_buffer_.empty() &&
                latest_time_ns - imu_buffer_.front().timestamp_ns > window_ns) {
            imu_buffer_.pop_front();
        }

        const double now_sec = this->now().seconds();

        if (now_sec - last_print_time_sec_ < print_every_sec_) {
            return;
        }

        last_print_time_sec_ = now_sec;

        runPreintegration();
    }

    void runPreintegration() {
        if (imu_buffer_.size() < 2) {
            return;
        }

        std::vector<ImuSample> samples;
        samples.reserve(imu_buffer_.size());

        for (const auto& sample : imu_buffer_) {
            samples.push_back(sample);
        }

        ImuBias bias;
        bias.gyro_bias = Eigen::Vector3d::Zero();
        bias.accel_bias = Eigen::Vector3d::Zero();

        const PreintegrationResult result = preintegrator_.integrate(samples, bias);

        if (!result.valid) {
            RCLCPP_WARN(
                this->get_logger(),
                "Preintegration result is invalid."
            );
            return;
        }

        // ---- IMU preintegration 결과를 터미널에 보기 좋게 출력하는 코드 ----
        const Eigen::AngleAxisd aa(result.delta_rotation);
            // Eigen::AngleAxisd는 쿼터니언 회전값을 “회전축 + 회전각” 형태로 변환해주는 Eigen 클래스
            // 변환 예시
            //     axis  = [0, 0, 1]
            //     angle = 1.5708 rad

        const double rot_angle_deg = aa.angle() * 180.0 / M_PI;
            // M_PI는 보통 이 <cmath> 또는 C의 <math.h> 계열에서 제공되는 경우가 많아
            // M_PI는 C++ 표준에서 반드시 보장되는 이름은 아니야.
            // 더 안전하게 쓰려면 이렇게 직접 정의할 수 있어:
            //     constexpr double M_PI = 3.14159265358979323846;

        std::cout << "\n========== Isaac Nova Carter IMU Preintegration ==========\n";

        std::cout << "num samples        : " << samples.size() << "\n";
        std::cout << "delta_time_sec     : " << result.delta_time_sec << "\n";
        std::cout << "delta_position    : ["
                  << result.delta_position.x() << ", "
                  << result.delta_position.y() << ", "
                  << result.delta_position.z() << "]\n";

        std::cout << "delta_velocity    : ["
                  << result.delta_velocity.x() << ", "
                  << result.delta_velocity.y() << ", "
                  << result.delta_velocity.z() << "]\n";

        std::cout << "delta_rotation q  : [w="
                  << result.delta_rotation.w()
                  << ", x=" << result.delta_rotation.x()
                  << ", y=" << result.delta_rotation.y()
                  << ", z=" << result.delta_rotation.z()
                  << "]\n";

        std::cout << "rotation angle deg: "
                  << rot_angle_deg << "\n";

        std::cout << "covariance trace  : "
                  << result.covariance.trace() << "\n";

        std::cout << "=========================================================\n";

    }

private:
    std::string imu_topic_;
    double window_sec_ = 1.0;
    double print_every_sec_ = 1.0;
    double last_print_time_sec_ = 0.0;

    std::deque<ImuSample> imu_buffer_;

    ImuPreintegrator preintegrator_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<IsaacImuPreintegrationNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}


// 실행 예시
/*
ros2 run basalt_like_vio isaac_imu_preintegration_node \
  --ros-args \
  -p use_sim_time:=true
  
ros2 run basalt_like_vio isaac_imu_preintegration_node \
  --ros-args \
  -p use_sim_time:=true \
  -p imu_topic:=/chassis/imu \
  -p window_sec:=1.0
*/

