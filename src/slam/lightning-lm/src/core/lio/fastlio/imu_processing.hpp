// fastlio 版 IMU 处理：移植自 fast_lio_robosenseAiry/src/IMU_Processing.hpp
// 适配 lightning 的点类型(PointXYZIT, .time ms)、IMU(lightning::IMU)、MeasureGroup
#pragma once
#ifndef FASTLIO_IMU_PROCESSING_H
#define FASTLIO_IMU_PROCESSING_H

#include <algorithm>
#include <cmath>
#include <deque>
#include <vector>

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/log.h"
#include "common/measure_group.h"
#include "common/point_def.h"
#include "core/lightning_math.hpp"
#include "core/lio/pose6d.h"

#include "fastlio_common.h"
#include "use-ikfom.hpp"

namespace fastlio {

constexpr int MAX_INI_COUNT = 10;

class ImuProcess {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ImuProcess();
    ~ImuProcess();

    void Reset();
    void set_extrinsic(const V3D& transl, const M3D& rot);
    void set_gyr_cov(const V3D& scaler);
    void set_acc_cov(const V3D& scaler);
    void set_gyr_bias_cov(const V3D& b_g);
    void set_acc_bias_cov(const V3D& b_a);

    void Process(const lightning::MeasureGroup& meas,
                 esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state, lightning::CloudPtr pcl_out);

    bool IsIMUInited() const { return imu_need_init_ == false; }

    double first_lidar_time = 0;

    Eigen::Matrix<double, 12, 12> Q;

    V3D cov_acc;
    V3D cov_gyr;
    V3D cov_acc_scale;
    V3D cov_gyr_scale;
    V3D cov_bias_gyr;
    V3D cov_bias_acc;

   private:
    void IMU_init(const lightning::MeasureGroup& meas, esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state, int& N);
    void UndistortPcl(const lightning::MeasureGroup& meas, esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state,
                      lightning::CloudPtr pcl_out);

    lightning::CloudPtr cur_pcl_un_;
    lightning::IMUPtr last_imu_;
    std::deque<lightning::IMUPtr> v_imu_;
    std::vector<lightning::Pose6D> IMUpose;

    M3D Lidar_R_wrt_IMU = M3D::Identity();
    V3D Lidar_T_wrt_IMU = V3D::Zero();
    V3D mean_acc = V3D(0, 0, -1.0);
    V3D mean_gyr = V3D::Zero();
    V3D angvel_last = V3D::Zero();
    V3D acc_s_last = V3D::Zero();
    double start_timestamp_ = -1;
    double last_lidar_end_time_ = 0;
    int init_iter_num = 1;
    bool b_first_frame_ = true;
    bool imu_need_init_ = true;
};

inline ImuProcess::ImuProcess() : b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1) {
    init_iter_num = 1;
    Q = process_noise_cov();
    cov_acc = V3D(0.1, 0.1, 0.1);
    cov_gyr = V3D(0.1, 0.1, 0.1);
    cov_bias_gyr = V3D(0.0001, 0.0001, 0.0001);
    cov_bias_acc = V3D(0.0001, 0.0001, 0.0001);
    mean_acc = V3D(0, 0, -1.0);
    mean_gyr = V3D::Zero();
    angvel_last = V3D::Zero();
    Lidar_T_wrt_IMU = V3D::Zero();
    Lidar_R_wrt_IMU = M3D::Identity();
    last_imu_.reset(new lightning::IMU());
}

inline ImuProcess::~ImuProcess() {}

inline void ImuProcess::Reset() {
    mean_acc = V3D(0, 0, -1.0);
    mean_gyr = V3D::Zero();
    angvel_last = V3D::Zero();
    imu_need_init_ = true;
    start_timestamp_ = -1;
    init_iter_num = 1;
    v_imu_.clear();
    IMUpose.clear();
    last_imu_.reset(new lightning::IMU());
    cur_pcl_un_.reset(new lightning::PointCloudType());
}

inline void ImuProcess::set_extrinsic(const V3D& transl, const M3D& rot) {
    Lidar_T_wrt_IMU = transl;
    Lidar_R_wrt_IMU = rot;
}

inline void ImuProcess::set_gyr_cov(const V3D& scaler) { cov_gyr_scale = scaler; }

inline void ImuProcess::set_acc_cov(const V3D& scaler) { cov_acc_scale = scaler; }

inline void ImuProcess::set_gyr_bias_cov(const V3D& b_g) { cov_bias_gyr = b_g; }

inline void ImuProcess::set_acc_bias_cov(const V3D& b_a) { cov_bias_acc = b_a; }

inline void ImuProcess::IMU_init(const lightning::MeasureGroup& meas,
                                 esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state, int& N) {
    V3D cur_acc, cur_gyr;

    if (b_first_frame_) {
        Reset();
        N = 1;
        b_first_frame_ = false;
        const auto& imu_acc = meas.imu_.front()->linear_acceleration;
        const auto& gyr_acc = meas.imu_.front()->angular_velocity;
        mean_acc = imu_acc;
        mean_gyr = gyr_acc;
        first_lidar_time = meas.lidar_begin_time_;
    }

    for (const auto& imu : meas.imu_) {
        const auto& imu_acc = imu->linear_acceleration;
        const auto& gyr_acc = imu->angular_velocity;
        cur_acc = imu_acc;
        cur_gyr = gyr_acc;

        mean_acc += (cur_acc - mean_acc) / N;
        mean_gyr += (cur_gyr - mean_gyr) / N;

        cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
        cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);

        N++;
    }

    state_ikfom init_state = kf_state.get_x();

    /// 重力对齐：世界系 +z 轴对齐到真实竖直（由加速度计测得的"上" mean_acc 决定）
    /// 说明：acc 静止时测量的是比力（指向远离地球方向），即传感器系里的"上"。
    /// 将 body 的"上"旋转到世界 +z，则重力在世界系 = (0,0,-G)，地图即为水平。
    V3D up_body = mean_acc.normalized();
    Eigen::Matrix3d R_align =
        Eigen::Quaterniond::FromTwoVectors(up_body, V3D(0.0, 0.0, 1.0)).toRotationMatrix();
    init_state.rot = SO3(R_align);  // body -> world

    init_state.grav = S2(V3D(0.0, 0.0, -G_m_s2));  // 对齐后重力竖直向下
    init_state.bg = mean_gyr;
    init_state.offset_T_L_I = Lidar_T_wrt_IMU;
    init_state.offset_R_L_I = Lidar_R_wrt_IMU;
    kf_state.change_x(init_state);

    esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
    init_P.setIdentity();
    init_P(6, 6) = init_P(7, 7) = init_P(8, 8) = 0.00001;
    init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.00001;
    init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.0001;
    init_P(18, 18) = init_P(19, 19) = init_P(20, 20) = 0.001;
    init_P(21, 21) = init_P(22, 22) = 0.00001;
    kf_state.change_P(init_P);

    last_imu_ = meas.imu_.back();
}

inline void ImuProcess::UndistortPcl(const lightning::MeasureGroup& meas,
                                     esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state,
                                     lightning::CloudPtr pcl_out) {
    /*** add the imu of the last frame-tail to the head of current frame ***/
    auto v_imu = meas.imu_;
    v_imu.push_front(last_imu_);
    const double& imu_beg_time = v_imu.front()->timestamp;
    const double& imu_end_time = v_imu.back()->timestamp;
    const double& pcl_beg_time = meas.lidar_begin_time_;
    const double& pcl_end_time = meas.lidar_end_time_;

    /*** sort point clouds by offset time ***/
    *pcl_out = *(meas.scan_);
    std::sort(pcl_out->points.begin(), pcl_out->points.end(),
              [](const lightning::PointType& a, const lightning::PointType& b) { return a.time < b.time; });

    /*** Initialize IMU pose ***/
    state_ikfom imu_state = kf_state.get_x();
    IMUpose.clear();
    IMUpose.push_back(lightning::Pose6D(0.0, acc_s_last, angvel_last, imu_state.vel, imu_state.pos,
                                        imu_state.rot.toRotationMatrix()));

    /*** forward propagation at each imu point ***/
    V3D angvel_avr, acc_avr;
    M3D R_imu;
    double dt = 0;
    input_ikfom in;

    for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++) {
        auto&& head = *(it_imu);
        auto&& tail = *(it_imu + 1);

        double tail_stamp = tail->timestamp;
        double head_stamp = head->timestamp;

        if (tail_stamp < last_lidar_end_time_) {
            continue;
        }

        angvel_avr = 0.5 * (head->angular_velocity + tail->angular_velocity);
        acc_avr = 0.5 * (head->linear_acceleration + tail->linear_acceleration);
        acc_avr = acc_avr * G_m_s2 / mean_acc.norm();

        if (head_stamp < last_lidar_end_time_) {
            dt = tail_stamp - last_lidar_end_time_;
        } else {
            dt = tail_stamp - head_stamp;
        }

        in.acc = acc_avr;
        in.gyro = angvel_avr;
        Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
        Q.block<3, 3>(3, 3).diagonal() = cov_acc;
        Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
        Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;
        kf_state.predict(dt, Q, in);

        /* save the poses at each IMU measurements */
        imu_state = kf_state.get_x();
        angvel_last = angvel_avr - imu_state.bg;
        acc_s_last = imu_state.rot * (acc_avr - imu_state.ba);
        for (int i = 0; i < 3; i++) {
            acc_s_last[i] += imu_state.grav[i];
        }
        double offs_t = tail_stamp - pcl_beg_time;
        IMUpose.push_back(lightning::Pose6D(offs_t, acc_s_last, angvel_last, imu_state.vel, imu_state.pos,
                                            imu_state.rot.toRotationMatrix()));
    }

    /*** calculated the pos and attitude prediction at the frame-end ***/
    double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
    dt = note * (pcl_end_time - imu_end_time);
    kf_state.predict(dt, Q, in);

    imu_state = kf_state.get_x();
    last_imu_ = meas.imu_.back();
    last_lidar_end_time_ = pcl_end_time;

    /*** undistort each lidar point (backward propagation) ***/
    if (pcl_out->points.empty()) {
        return;
    }
    auto it_pcl = pcl_out->points.end() - 1;
    for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--) {
        auto head = it_kp - 1;
        auto tail = it_kp;
        R_imu = head->rot;
        V3D vel_imu = head->vel;
        V3D pos_imu = head->pos;
        V3D acc_imu = tail->acc;
        angvel_avr = tail->gyr;

        for (; it_pcl->time / double(1000) > head->offset_time; it_pcl--) {
            dt = it_pcl->time / double(1000) - head->offset_time;

            /* Transform to the 'end' frame, using only the rotation */
            M3D R_i(R_imu * lightning::math::Exp(angvel_avr, dt));

            V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
            V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
            V3D P_compensate = imu_state.offset_R_L_I.conjugate() *
                               (imu_state.rot.conjugate() *
                                    (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) -
                                imu_state.offset_T_L_I);

            it_pcl->x = P_compensate(0);
            it_pcl->y = P_compensate(1);
            it_pcl->z = P_compensate(2);

            if (it_pcl == pcl_out->points.begin()) {
                break;
            }
        }
    }
}

inline void ImuProcess::Process(const lightning::MeasureGroup& meas,
                                esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state, lightning::CloudPtr pcl_out) {
    if (meas.imu_.empty()) {
        return;
    }

    if (imu_need_init_) {
        /// The very first lidar frame
        IMU_init(meas, kf_state, init_iter_num);

        imu_need_init_ = true;

        last_imu_ = meas.imu_.back();

        if (init_iter_num > MAX_INI_COUNT) {
            cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
            imu_need_init_ = false;

            cov_acc = cov_acc_scale;
            cov_gyr = cov_gyr_scale;
            LLOG_INFO(lightning::logging::kLio, "fastlio IMU Initial Done, grav: {} {} {}", kf_state.get_x().grav[0],
                      kf_state.get_x().grav[1], kf_state.get_x().grav[2]);
        } else {
            LLOG_DEBUG(lightning::logging::kLio, "fastlio waiting for imu init ... {}", init_iter_num);
        }

        return;
    }

    UndistortPcl(meas, kf_state, pcl_out);
}

}  // namespace fastlio

#endif  // FASTLIO_IMU_PROCESSING_H
