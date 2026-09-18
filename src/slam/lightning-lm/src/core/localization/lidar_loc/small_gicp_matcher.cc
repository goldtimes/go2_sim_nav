#include "core/localization/lidar_loc/small_gicp_matcher.h"

#include <pcl/common/point_tests.h>
#include <pcl/common/transforms.h>

#include "common/log.h"

namespace lightning::loc {

void SmallGicpMatcher::AddTargetCloud(const CloudPtr& cloud) {
    if (cloud == nullptr) {
        return;
    }

    raw_target_points_.reserve(raw_target_points_.size() + cloud->size());
    for (const auto& pt : cloud->points) {
        if (!pcl::isFinite(pt)) {
            continue;
        }
        raw_target_points_.emplace_back(pt.x, pt.y, pt.z, 1.0);
    }
}

void SmallGicpMatcher::BuildTarget() {
    target_ = nullptr;
    target_tree_ = nullptr;

    if (raw_target_points_.empty()) {
        LLOG_WARN(logging::kLidarLoc, "SmallGicpMatcher: no target points, skip build");
        return;
    }

    small_gicp::PointCloud raw;
    raw.points = std::move(raw_target_points_);
    raw_target_points_.clear();
    raw_target_points_.shrink_to_fit();

    auto result = small_gicp::preprocess_points(raw, options_.target_res_, options_.num_neighbors_, options_.num_threads_);
    target_ = result.first;
    target_tree_ = result.second;

    LLOG_INFO(logging::kLidarLoc, "SmallGicpMatcher target built, points: {} -> {}, res: {}", raw.points.size(),
              target_->size(), options_.target_res_);
}

bool SmallGicpMatcher::Align(const CloudPtr& source, const SE3& init_pose, SE3& out_pose, double& confidence) const {
    confidence = 0.0;
    if (!HasTarget() || source == nullptr || source->empty()) {
        return false;
    }

    return AlignPre(PreprocessSource(source), init_pose, out_pose, confidence);
}

std::shared_ptr<small_gicp::PointCloud> SmallGicpMatcher::PreprocessSource(const CloudPtr& source) const {
    if (source == nullptr || source->empty()) {
        return nullptr;
    }

    small_gicp::PointCloud raw_source;
    raw_source.points.reserve(source->size());
    for (const auto& pt : source->points) {
        if (!pcl::isFinite(pt)) {
            continue;
        }
        raw_source.points.emplace_back(pt.x, pt.y, pt.z, 1.0);
    }

    return small_gicp::preprocess_points(raw_source, options_.source_res_, options_.num_neighbors_, options_.num_threads_)
        .first;
}

bool SmallGicpMatcher::AlignPre(const std::shared_ptr<small_gicp::PointCloud>& source_pp, const SE3& init_pose,
                                SE3& out_pose, double& confidence) const {
    confidence = 0.0;
    if (!HasTarget() || source_pp == nullptr || source_pp->empty()) {
        return false;
    }
    const auto& source_cloud = source_pp;

    small_gicp::RegistrationSetting setting;
    setting.type = small_gicp::RegistrationSetting::GICP;
    setting.max_correspondence_distance = options_.max_corr_dist_;
    setting.max_iterations = options_.max_iterations_;
    setting.num_threads = options_.num_threads_;

    const Eigen::Isometry3d init_T(init_pose.matrix());
    const auto& result =
        small_gicp::align(*target_, *source_cloud, *target_tree_, init_T, setting);

    const Eigen::Matrix4d m = result.T_target_source.matrix();
    Quatd q(m.block<3, 3>(0, 0));
    q.normalize();
    out_pose = SE3(q, m.block<3, 1>(0, 3));

    /// 内点比例：落在 max_corr_dist 内的源点占比，[0,1] 越大越好
    confidence = static_cast<double>(result.num_inliers) / static_cast<double>(source_cloud->size());

    return true;
}

}  // namespace lightning::loc
