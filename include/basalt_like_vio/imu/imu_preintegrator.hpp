
#pragma once

#include <cmath>
#include <cstddef>   

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>    

#include "basalt_like_vio/common/time_utils.hpp"
#include "basalt_like_vio/imu/imu_sample.hpp"
#include "basalt_like_vio/vio/vio_state.hpp"

namespace basalt_like_vio {
namespace imu {

/**
 * @brief SO(3) hat 연산자
 *
 * 벡터 w = [wx, wy, wz]를 skew-symmetric matrix로 바꿉니다.
 *
 * w_hat =
 * [  0  -wz   wy
 *   wz    0  -wx
 *  -wy   wx    0 ]
 *
 * 외적과 같은 역할:
 *
 *   hat(w) * v = w x v
 */
inline Eigen::Matrix3d hat(const Eigen::Vector3d& w) {
    Eigen::Matrix3d W;
    W << 0.0, -w.z(), w.y(),
         w.z(), 0.0, -w.x(),
         -w.y(), w.x(), 0.0;
    return W;
}

inline Eigen::Quaterniond expSO3(const Eigen::Vector3d& phi) {
    const double theta = phi.norm();   // 벡터의 길이 즉, 회전각 크기

    // 아주 작은 회전에서는 theta가 거의 0이라서 나누기 계산이 불안정하여 아래와 같이 근사식을 씁니다.
    if (theta < 1e-12) {
        Eigen::Quaterniond q(
            1.0,
            0.5 * phi.x(),
            0.5 * phi.y(),
            0.5 * phi.z()
        );
        q.normalize();
        return q;
    }

    const Eigen::Vector3d axis = phi / theta;   // 단위벡터(방향벡터)
    Eigen::Quaterniond q(Eigen::AngleAxisd(theta, axis));  // AngleAxisd로 quaternion 만들기
        // 회전각 theta와 회전축 axis를 이용해서 quaternion q를 만든다
    q.normalize();
    return q;
}

// quaternion 회전을 작은 회전 벡터로 변환합니다.
inline Eigen::Vector3d logSO3(const Eigen::Quaterniond& q_in) {
    Eigen::Quaterniond q = q_in;
    q.normalize();

    Eigen::AngleAxisd aa(q);
        // aa 안에는 두 가지 정보가 들어갑니다.
        //     1. axis  : 회전축 방향 벡터
        //     2. angle : 회전각 크기

    return aa.axis() * aa.angle();
}


inline Eigen::Matrix3d rightJacobianSO3(const Eigen::Vector3d& phi) {
    const double theta = phi.norm();
    const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d Phi = hat(phi);

    if (theta < 1e-8) {
        return I - 0.5 * Phi + (1.0 / 6.0) * Phi * Phi;
            // 아주 작은 회전에서는 정확한 공식 대신 근사식을 씁니다.
            //         Jr(phi) ≈ I - 1/2 * hat(phi) + 1/6 * hat(phi)^2
            // 회전이 거의 0이면 복잡한 삼각함수 계산 대신
            // 안정적인 근사식으로 계산한다
    }

    const double theta2 = theta * theta;
    const double theta3 = theta2 * theta;

    // 아래 공식은 SO(3) 회전의 exponential map을 미분했을 때 나오는 공식
    // 회전벡터 phi에 작은 오차가 붙었을때,
    // 그 오차가 실제 회전 exp(phi) 리군에서 어떻게 보이는가?
    return I
        - ((1.0 - std::cos(theta)) / theta2) * Phi
        + ((theta - std::sin(theta)) / theta3) * Phi * Phi;
}

struct ImuNoiseOptions {
    double gyro_noise_std = 1e-3;
    double accel_noise_std = 1e-2;

    double gyro_bias_rw_std = 1e-5;
    double accel_bias_rw_std = 1e-4;
};

struct ImuPreintegratorOptions {
    Eigen::Vector3d gravity_w = Eigen::Vector3d(0.0, 0.0, -9.81);

    double max_dt_sec = 0.05;

    ImuNoiseOptions noise;
};


struct PreintegrationResult {
    Eigen::Quaterniond delta_rotation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d delta_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d delta_position = Eigen::Vector3d::Zero();

    double delta_time_sec = 0.0;

    common::TimeNs start_time_ns = 0;
    common::TimeNs end_time_ns = 0;

    ImuBias linearization_bias;

    /**
    * 상태 오차 순서:
    *
    *   0:3    position error
    *   3:6    rotation error
    *   6:9    velocity error
    *   9:12   accel bias error
    *   12:15  gyro bias error
    */
    Eigen::Matrix<double, 15, 15> covariance = 
        Eigen::Matrix<double, 15, 15>::Zero();

    Eigen::Matrix<double, 15, 15> jacobian = 
        Eigen::Matrix<double, 15, 15>::Identity();

    Eigen::Matrix3d dP_dBa = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d dP_dBg = Eigen::Matrix3d::Zero();

    Eigen::Matrix3d dR_dBg = Eigen::Matrix3d::Zero();

    Eigen::Matrix3d dV_dBa = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d dV_dBg = Eigen::Matrix3d::Zero();

    bool valid = false;

    // bias가 바뀌었을 때 delta_rotation 보정
    Eigen::Quaterniond correctedDeltaRotation(const ImuBias& new_bias) const {
        const Eigen::Vector3d dbg = 
            new_bias.gyro_bias - linearization_bias.gyro_bias;

        Eigen::Quaterniond corrected =
            delta_rotation * expSO3(dR_dBg * dbg);

        corrected.normalize();
        return corrected;
    }

    // bias가 바뀌었을 때 delta_velocity 보정
    // 새로운 bias가 들어왔을 때,
    // 기존 delta_velocity를 새 bias 기준으로 살짝 보정한다.
    Eigen::Vector3d correctedDeltaVelocity(const ImuBias& new_bias) const {
        const Eigen::Vector3d dba = 
            new_bias.accel_bias - linearization_bias.accel_bias;
        
        const Eigen::Vector3d dbg =
            new_bias.gyro_bias - linearization_bias.gyro_bias;

        return delta_velocity + dV_dBa * dba + dV_dBg * dbg;
    }

    // bias가 바뀌었을 때 delta_position 보정
    Eigen::Vector3d correctedDeltaPosition(const ImuBias& new_bias) const {
        const Eigen::Vector3d dba = 
            new_bias.accel_bias - linearization_bias.accel_bias;
        
        const Eigen::Vector3d dbg =
            new_bias.gyro_bias - linearization_bias.gyro_bias;

        return delta_position + dP_dBa * dba + dP_dBg * dbg;
    }

};

struct ImuResidual {
    Eigen::Matrix<double, 15, 1> residual = 
        Eigen::Matrix<double, 15, 1>::Zero();

    Eigen::Matrix<double, 15, 15> covariance = 
        Eigen::Matrix<double, 15, 15>::Zero();

    Eigen::Matrix<double, 15, 15> information =
        Eigen::Matrix<double, 15, 15>::Zero();

    bool valid = false;
};


class ImuPreintegrator {

public:
    ImuPreintegrator() = default;

    explicit ImuPreintegrator(const ImuPreintegratorOptions& options)
        : options_(options) {}

    PreintegrationResult integrate(
        const std::vector<ImuSample>& imu_samples,
        const ImuBias& bias
    ) const {
        PreintegrationResult result;

        if (imu_samples.size() < 2) {
            return result;
        }

        result.start_time_ns = imu_samples.front().timestamp_ns;
            // std::vector의 기본 함수인 .front()는 imu_samples[0]와 같은 의미
            // 첫 번째 IMU 샘플 시간
        result.end_time_ns = imu_samples.back().timestamp_ns;
        result.linearization_bias = bias;

        Eigen::Quaterniond delta_R = Eigen::Quaterniond::Identity();
        Eigen::Vector3d delta_v = Eigen::Vector3d::Zero();
        Eigen::Vector3d delta_p = Eigen::Vector3d::Zero();

        // preintegration 결과의 불확실성, covariance
        Eigen::Matrix<double, 15, 15> P = 
            Eigen::Matrix<double, 15, 15>::Zero();
        // bias가 바뀌었을 때 delta 값들이 어떻게 변하는지 나타내는 Jacobian
        Eigen::Matrix<double, 15, 15> J = 
            Eigen::Matrix<double, 15, 15>::Identity();

        double total_dt = 0.0;
        

        for (std::size_t i = 0; i + 1 < imu_samples.size(); ++i) {
            const ImuSample& s0 = imu_samples[i];
            const ImuSample& s1 = imu_samples[i + 1];

            const double dt = common::deltaTimeSeconds(
            s1.timestamp_ns,
            s0.timestamp_ns
            );

            if (dt <= 0.0) {
            continue;
            }

            /**
            * Midpoint 방식:
            *
            * gyro, accel을 s0/s1 평균으로 사용합니다.
            * 단순 Euler보다 안정적입니다.
            */
            const Eigen::Vector3d gyro0 = correctedGyro(s0, bias);
            // correctedGyro함수 반환값은 s0.gyro - bias.gyro_bias 로서 자이로값 보정
            const Eigen::Vector3d gyro1 = correctedGyro(s1, bias);
            const Eigen::Vector3d acc0 = correctedAccel(s0, bias);
            const Eigen::Vector3d acc1 = correctedAccel(s1, bias);

            const Eigen::Vector3d omega = 0.5 * (gyro0 + gyro1);
            const Eigen::Vector3d accel = 0.5 * (acc0 + acc1);

            const Eigen::Vector3d phi = omega * dt;
            const Eigen::Quaterniond dR = expSO3(phi);  // 리대수를 리군으로 올림

            const Eigen::Matrix3d R = delta_R.toRotationMatrix(); // 회전벡터를 회전행렬로 변환
            const Eigen::Vector3d acc_delta = R * accel;

            // 가속도와 속도를 이용해서 위치 변화량 delta_p를 업데이트하는 부분
            delta_p = delta_p + delta_v * dt + 0.5 * acc_delta * dt * dt;

            delta_v = delta_v + acc_delta * dt;
            delta_R = delta_R * dR;
            delta_R.normalize();

            Eigen::Matrix<double, 15, 15> F = 
                Eigen::Matrix<double, 15, 15>::Identity();

            Eigen::Matrix<double, 15, 12> G =
                Eigen::Matrix<double, 15, 12>::Zero();

            const Eigen::Matrix3d acc_hat = hat(accel);
            const Eigen::Matrix3d R_acc_hat = R * acc_hat;

            // position error propagation
            // 속도 오차 dv가 위치 오차 dp에 얼마나 영향을 주는가?
            F.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * dt;
            
            // position error dp가 rotation error dtheta의 영향을 받는 부분
            F.block<3, 3>(0, 3) = -0.5 * R_acc_hat * dt * dt;
            
            // 가속도 bias 오차 dba가 위치 오차 dp에 영향을 주는 관계를 F 행렬에 넣는 코드
            F.block<3, 3>(0, 9) = -0.5 * R * dt * dt;

            // rotation error propagation
            // 회전 오차 dtheta가 다음 시간으로 어떻게 전파되는지를 F 행렬에 넣는 코드
            F.block<3, 3>(3, 3) = dR.inverse().toRotationMatrix();

            // gyro bias 오차 dbg가 회전 오차 dtheta에 어떤 영향을 주는지를 F 행렬에 넣는 코드
            F.block<3, 3>(3, 12) = -rightJacobianSO3(phi) * dt;

            // velocity error propagation
            // 회전 오차 dtheta가 속도 오차 dv에 영향을 주는 관계를 F 행렬에 넣는 코드
            F.block<3, 3>(6, 3) = -R_acc_hat * dt;

            // 가속도 bias 오차가 속도 오차에 얼마나 영향을 주는가?
            F.block<3, 3>(6, 9) = -R * dt;

            G.block<3, 3>(0, 0) = 0.5 * R * dt * dt;
            G.block<3, 3>(3, 3) = rightJacobianSO3(phi) * dt;
            G.block<3, 3>(6, 0) = R * dt;
            G.block<3, 3>(9, 6) = Eigen::Matrix3d::Identity() * dt;
            G.block<3, 3>(12, 9) = Eigen::Matrix3d::Identity() * dt;

            Eigen::Matrix<double, 12, 12> Q =
                Eigen::Matrix<double, 12, 12>::Zero();

            // 아래 코드는 IMU 노이즈 표준편차 설정값을 공분산 계산에 사용할 수 있는 분산값으로 바꾸는 부분
            const double ng2 = 
                options_.noise.gyro_noise_std * options_.noise.gyro_noise_std;
            const double na2 =
                options_.noise.accel_noise_std * options_.noise.accel_noise_std;
            const double nbg2 =
                options_.noise.gyro_bias_rw_std * options_.noise.gyro_bias_rw_std;
            const double nba2 =
                options_.noise.accel_bias_rw_std * options_.noise.accel_bias_rw_std;

            // 아래 코드는 12x12 노이즈 공분산 행렬 Q 안에 각각의 IMU 노이즈 크기를 넣는 부분
            Q.block<3, 3>(0, 0) = na2 * Eigen::Matrix3d::Identity();
            Q.block<3, 3>(3, 3) = ng2 * Eigen::Matrix3d::Identity();
            Q.block<3, 3>(6, 6) = nba2 * Eigen::Matrix3d::Identity();
            Q.block<3, 3>(9, 9) = nbg2 * Eigen::Matrix3d::Identity();

            P = F * P * F.transpose() + G * Q * G.transpose();
            J = F * J;

            total_dt += dt;
        }

        result.delta_rotation = delta_R;
        result.delta_velocity = delta_v;
        result.delta_position = delta_p;
        result.delta_time_sec = total_dt;

        result.covariance = P;
        result.jacobian = J;

        result.dP_dBa = J.block<3, 3>(0, 9);
        result.dP_dBg = J.block<3, 3>(0, 12);

        result.dR_dBg = J.block<3, 3>(3, 12);

        result.dV_dBa = J.block<3, 3>(6, 9);
        result.dV_dBg = J.block<3, 3>(6, 12);

        result.valid = total_dt > 0.0;

        return result;
    }   // PreintegrationResult integrate 함수 끝

    /**
    * @brief IMU preintegration으로 다음 상태 예측
    *
    * 이 함수는 frontend tracking 또는 estimator 초기 예측에 사용합니다.
    */
    vio::VioState predict(
        const vio::VioState& previous_state,
        const std::vector<ImuSample>& imu_samples
    ) const {
        vio::VioState predicted_state = previous_state;

        if (imu_samples.size() < 2) {
            return predicted_state;
        }

        const PreintegrationResult preint =
            integrate(imu_samples, previous_state.bias);

        if (!preint.valid) {
            return predicted_state;
        }

        return predict(previous_state, preint, previous_state.bias);
    }

    // 이미 계산된 preintegration 결과를 이용해서 다음 상태 예측하는 함수
    vio::VioState predict(
        const vio::VioState& previous_state,
        const PreintegrationResult& preint,
        const ImuBias& current_bias) const {
        vio::VioState predicted_state = previous_state;

        if (!preint.valid) {
            return predicted_state;
        }

        const double dt = preint.delta_time_sec;

        const Eigen::Quaterniond R_w_i = previous_state.rotation_w_i;
        const Eigen::Vector3d p_w_i = previous_state.position_w_i;
        const Eigen::Vector3d v_w_i = previous_state.velocity_w_i;

        // bias가 바뀔때 해당 자코비안으로 보정 
        const Eigen::Quaterniond corrected_dR =
            preint.correctedDeltaRotation(current_bias);

        const Eigen::Vector3d corrected_dv =
            preint.correctedDeltaVelocity(current_bias);

        const Eigen::Vector3d corrected_dp = 
            preint.correctedDeltaPosition(current_bias);

        predicted_state.position_w_i =
            p_w_i +
            v_w_i * dt +
            0.5 * options_.gravity_w * dt * dt +
            R_w_i * corrected_dp;
        
        predicted_state.velocity_w_i =
            v_w_i +
            options_.gravity_w * dt +
            R_w_i * corrected_dv;

        predicted_state.rotation_w_i = 
            R_w_i * corrected_dR;

        predicted_state.normalizeRotation();

        predicted_state.timestamp_ns = preint.end_time_ns;
        predicted_state.bias = current_bias;
            // “이번 예측 상태는 이 바이어스 값을 사용해서 만들어졌다.” 의미 

        return predicted_state;
    }

    ImuResidual computeResidual(
        const vio::VioState& state_i,
        const vio::VioState& state_j,
        const PreintegrationResult& preint
    ) const {
        ImuResidual out;

        if (!preint.valid) {
            return out;
        }

        const double dt = preint.delta_time_sec;

        const Eigen::Quaterniond R_w_i = state_i.rotation_w_i;
        const Eigen::Quaterniond R_i_w = R_w_i.inverse();

        const Eigen::Vector3d p_i = state_i.position_w_i;
        const Eigen::Vector3d p_j = state_j.position_w_i;

        const Eigen::Vector3d v_i = state_i.velocity_w_i;
        const Eigen::Vector3d v_j = state_j.velocity_w_i;

        const Eigen::Quaterniond R_w_j = state_j.rotation_w_i;

        // preint는 i → j 구간의 IMU 적분값이고, 이 적분은 보통 state_i 시점의 bias를 기준으로 보정한다
        const Eigen::Vector3d dp_corr = 
            preint.correctedDeltaPosition(state_i.bias);
        const Eigen::Vector3d dv_corr = 
            preint.correctedDeltaVelocity(state_i.bias);
        const Eigen::Quaterniond dR_corr =
            preint.correctedDeltaRotation(state_i.bias);

        const Eigen::Vector3d r_p =
            R_i_w *
                (p_j - p_i - v_i * dt - 0.5 * options_.gravity_w * dt * dt) 
            - dp_corr;
        
        Eigen::Quaterniond q_err =
            dR_corr.inverse() * R_i_w * R_w_j;

        q_err.normalize();

        const Eigen::Vector3d r_R = logSO3(q_err);

        const Eigen::Vector3d r_v =
            R_i_w * (v_j - v_i - options_.gravity_w * dt) - dv_corr;

        const Eigen::Vector3d r_ba =
            state_j.bias.accel_bias - state_i.bias.accel_bias;

        const Eigen::Vector3d r_bg =
            state_j.bias.gyro_bias - state_i.bias.gyro_bias;

        out.residual.segment<3>(0) = r_p;
        out.residual.segment<3>(3) = r_R;
        out.residual.segment<3>(6) = r_v;
        out.residual.segment<3>(9) = r_ba;
        out.residual.segment<3>(12) = r_bg;

        out.covariance = preint.covariance;

        Eigen::Matrix<double, 15, 15> cov_damped =
                preint.covariance +
                1e-12 * Eigen::Matrix<double, 15, 15>::Identity();

        out.information = cov_damped.inverse();
        out.valid = true;

        return out;
    }

    const ImuPreintegratorOptions& options() const {
        return options_;
    }

private:

    ImuPreintegratorOptions options_;

};

}  // namespace imu
}  // namespace basalt_like_vio




