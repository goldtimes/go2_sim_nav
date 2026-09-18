#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/registration_helper.hpp>

#include "common/eigen_types.h"
#include "common/point_def.h"

namespace lightning::loc {

/// 基于 small_gicp 的地图匹配器（GICP）
/// 用法：分块地图逐个 AddTargetCloud() 后调用 BuildTarget() 一次性预处理建目标；
/// 之后 Align() 做配准。目标热替换由外部 match_mutex_ 同步，类内无锁。
class SmallGicpMatcher {
   public:
    struct Options {
        double target_res_ = 0.3;    /// 目标地图降采样分辨率 (m)
        double source_res_ = 0.3;    /// 源点云降采样分辨率 (m)
        double max_corr_dist_ = 1.0; /// 最大对应点距离 (m)
        int max_iterations_ = 50;
        int num_threads_ = 4;
        int num_neighbors_ = 10; /// 协方差估计近邻数
    };

    SmallGicpMatcher() = default;
    explicit SmallGicpMatcher(const Options& options) : options_(options) {}

    Options options_;

    /// 累积一路目标点云（分块地图的静态/动态层），BuildTarget() 时统一处理
    void AddTargetCloud(const CloudPtr& cloud);

    /// 对已累积的目标点云做降采样、协方差估计并建立 kdtree
    void BuildTarget();

    /// 目标是否已就绪
    bool HasTarget() const { return target_ != nullptr && !target_->empty(); }

    /// 配准：source 为当前帧（PCL），init_pose 为 map 系初始猜测
    /// @param confidence 内点比例（num_inliers / 降采样后源点数），[0,1] 越大越好；配准失败为 0
    /// @return 目标就绪且配准执行成功
    bool Align(const CloudPtr& source, const SE3& init_pose, SE3& out_pose, double& confidence) const;

    /// 源点云预处理（降采样+协方差估计），结果可复用于多次 AlignPre（同一帧多候选搜索时避免重复处理）
    std::shared_ptr<small_gicp::PointCloud> PreprocessSource(const CloudPtr& source) const;

    /// 对已预处理源点云做配准（目标须已 BuildTarget）
    bool AlignPre(const std::shared_ptr<small_gicp::PointCloud>& source_pp, const SE3& init_pose, SE3& out_pose,
                  double& confidence) const;

   private:
    std::vector<Eigen::Vector4d> raw_target_points_;        /// 未处理的分块目标点累积
    std::shared_ptr<small_gicp::PointCloud> target_;        /// 降采样+协方差后的目标
    std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>> target_tree_;
};

}  // namespace lightning::loc
