//
// Created by xiang on 25-3-12.
//

#ifndef LIGHTNING_KEYFRAME_H
#define LIGHTNING_KEYFRAME_H

#include "common/eigen_types.h"
#include "common/nav_state.h"
#include "common/point_def.h"
#include "common/std_types.h"

#include <pcl/filters/voxel_grid.h>

#include <cmath>
#include <utility>
#include <vector>

namespace lightning {

/// 关键帧描述
/// NOTE: 在添加后端后，需要加锁
class Keyframe {
   public:
    using Ptr = std::shared_ptr<Keyframe>;

    Keyframe() {}

    /// @param cloud_res 本关键帧点云已经过的体素分辨率；0 = 未降采样（存的是原始去畸变扫描）。
    ///                  用途见 GetCloudDownsampled()：如果存的时侯已经滤过，就没必要再滤一遍。
    Keyframe(unsigned long id, CloudPtr cloud, NavState state, float cloud_res = 0)
        : id_(id), cloud_(cloud), cloud_res_(cloud_res), pose_lio_(state.GetPose()), state_(state) {
        timestamp_ = state_.timestamp_;
        pose_opt_ = pose_lio_;
    }

    unsigned long GetID() const { return id_; }
    CloudPtr GetCloud() const { return cloud_; }

    /// 本关键帧点云已经过的体素分辨率（0 = 未降采样）
    float GetCloudRes() const { return cloud_res_; }

    /// 按 res 体素降采样后的点云（懒计算 + 按 res 缓存）。
    ///
    /// 为什么可以缓存：
    ///   cloud_ 是 LIDAR 系的去畸变点云，构造之后就不再变化（Keyframe 没有 SetCloud）；
    ///   体素降采样只依赖「点云 + res」，与位姿无关。所以同一 res 的结果永远不需要重算。
    /// 为什么值得缓存：
    ///   GetGlobalMap() 每调用一次都会对全部关键帧重新降采样一遍。实测 1663 kf 时
    ///   单次 GetGlobalMap 中「逐帧降采样」占 310ms / 497ms = 62%，而这次重算的结果
    ///   和上次一模一样。缓存后该开销只在「关键帧首次被用到」时付一次。
    /// 注意：
    ///   用独立的 cache_mutex_，不要并到 data_mutex_ 里 —— 降采样本身要 0.3~0.4ms，
    ///   持有 data_mutex_ 那么久会挡住 GetLIOPose()/SetOptPose()（worker 线程在跑回环）。
    CloudPtr GetCloudDownsampled(float res) {
        std::lock_guard<std::mutex> lock(cache_mutex_);

        /// 存的时侯就是按 cloud_res_ 降过采样的：若请求的分辨率与它一致，
        /// 再滤一遍不会有任何变化，直接返回。
        /// 这正是 GetGlobalMap(0.1) 的情形 —— 实测逐帧滤波占了存图总耗时的 66%。
        if (cloud_res_ > 0 && std::fabs(res - cloud_res_) < 1e-6f) {
            return cloud_;
        }

        for (const auto& e : cloud_downsampled_) {
            if (e.first == res) {
                return e.second;
            }
        }

        auto out = std::make_shared<PointCloudType>();
        pcl::VoxelGrid<PointType> voxel;
        voxel.setLeafSize(res, res, res);
        voxel.setInputCloud(cloud_);
        voxel.filter(*out);

        /// 正常情况下全工程只会用 1~2 个 res（发布用粗的、存图用细的）。
        /// 万一被传了五花八门的 res，这里兜一个上限，避免缓存无界增长（每份几十 KB）。
        if (cloud_downsampled_.size() >= 4) {
            cloud_downsampled_.clear();
        }
        cloud_downsampled_.emplace_back(res, out);
        return out;
    }

    SE3 GetLIOPose() {
        UL lock(data_mutex_);
        return pose_lio_;
    }

    void SetLIOPose(const SE3& pose) {
        UL lock(data_mutex_);
        pose_lio_ = pose;

        // also set opt
        pose_opt_ = pose_lio_;
    }

    SE3 GetOptPose() {
        UL lock(data_mutex_);
        return pose_opt_;
    }

    void SetOptPose(const SE3& pose) {
        UL lock(data_mutex_);
        pose_opt_ = pose;
    }

    void SetState(NavState s) {
        UL lock(data_mutex_);
        state_ = s;
    }

    NavState GetState() {
        UL lock(data_mutex_);
        return state_;
    }

   protected:
    unsigned long id_ = 0;

    double timestamp_ = 0;

    /// 关键帧点云，LIDAR 系。
    /// 内容取决于 LaserMapping::kf_cloud_res_：
    ///   0（默认）  原始去畸变扫描（只经过预处理 point_filter_num 抽点，无体素降采样）
    ///   >0        MakeKF() 时已按该分辨率体素降采样过（见 cloud_res_）
    /// 之前这里写成「降采样之后的点云」（其实只对 >0 的情况成立），会让人误以为
    /// GetGlobalMap 里的逐帧体素滤波是多余的 —— 它实际上是必须的，只是应该避免重算。
    CloudPtr cloud_ = nullptr;

    /// cloud_ 已经过的体素分辨率（0 = 未降采样）。见 GetCloudDownsampled()。
    float cloud_res_ = 0;

    /// GetCloudDownsampled() 的缓存：res -> 降采样结果。见该函数的说明。
    std::mutex cache_mutex_;
    std::vector<std::pair<float, CloudPtr>> cloud_downsampled_;

    std::mutex data_mutex_;
    SE3 pose_lio_;  // 前端的pose
    SE3 pose_opt_;  // 后端优化后的pose

    NavState state_;  // 卡尔曼滤波器状态
};

}  // namespace lightning

#endif  // LIGHTNING_KEYFRAME_H
