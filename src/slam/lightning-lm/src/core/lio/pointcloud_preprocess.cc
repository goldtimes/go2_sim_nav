#include "pointcloud_preprocess.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <execution>
#include <limits>

#include "common/log.h"

namespace lightning {

namespace {

/// PointCloud2 里某个字段的位置与类型（找不到时 offset = -1）
struct FieldInfo {
    int offset = -1;
    uint8_t datatype = 0;
};

FieldInfo FindField(const sensor_msgs::msg::PointCloud2 &msg, const std::string &name) {
    for (const auto &f : msg.fields) {
        if (f.name == name) {
            return FieldInfo{static_cast<int>(f.offset), f.datatype};
        }
    }
    return FieldInfo{};
}

/// 按字段声明的类型从原始 buffer 里读一个标量（float32/float64 之外返回 NaN）
float ReadScalar(const uint8_t *base, const FieldInfo &fi) {
    switch (fi.datatype) {
        case sensor_msgs::msg::PointField::FLOAT32: {
            float v;
            std::memcpy(&v, base + fi.offset, sizeof(float));
            return v;
        }
        case sensor_msgs::msg::PointField::FLOAT64: {
            double v;
            std::memcpy(&v, base + fi.offset, sizeof(double));
            return static_cast<float>(v);
        }
        default:
            return std::numeric_limits<float>::quiet_NaN();
    }
}

}  // namespace

void PointCloudPreprocess::Set(LidarType lid_type, double bld, int pfilt_num) {
    lidar_type_ = lid_type;
    blind_ = bld;
    point_filter_num_ = pfilt_num;
}

void PointCloudPreprocess::Process(const sensor_msgs::msg::PointCloud2 ::SharedPtr &msg, PointCloudType::Ptr &pcl_out) {
    switch (lidar_type_) {
        case LidarType::OUST64:
            Oust64Handler(msg);
            break;

        case LidarType::VELO32:
            VelodyneHandler(msg);
            break;

        case LidarType::ROBOSENSE:
            RoboSenseHandler(msg);
            break;

        case LidarType::SIMULATED:
            SimulatedHandler(msg);
            break;

        default:
            LLOG_ERROR(logging::kLio, "Error LiDAR Type");
            break;
    }
    *pcl_out = cloud_out_;
}

void PointCloudPreprocess::Process(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg,
                                   PointCloudType::Ptr &pcl_out) {
    cloud_out_.clear();
    cloud_full_.clear();

    int plsize = msg->point_num;

    cloud_out_.reserve(plsize);
    cloud_full_.resize(plsize);

    std::vector<char> is_valid_pt(plsize, 0);
    std::vector<uint> index(plsize - 1);
    for (uint i = 0; i < plsize - 1; ++i) {
        index[i] = i + 1;  // 从1开始
    }

    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const uint &i) {
        // if ((msg->points[i].line < num_scans_) &&
        // ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00)) {
        if (i % point_filter_num_ != 0) {
            return;
        }

        cloud_full_[i].x = msg->points[i].x;
        cloud_full_[i].y = msg->points[i].y;
        cloud_full_[i].z = msg->points[i].z;
        cloud_full_[i].intensity = msg->points[i].reflectivity;

        // use curvature as time of each laser points, curvature unit: ms
        cloud_full_[i].time = msg->points[i].offset_time / double(1000000);

        // 过滤 NaN/Inf 无效点
        if (!std::isfinite(cloud_full_[i].x) || !std::isfinite(cloud_full_[i].y) || !std::isfinite(cloud_full_[i].z)) {
            return;
        }

        if (cloud_full_[i].z < height_min_ || cloud_full_[i].z > height_max_) {
            return;
        }

        if ((abs(cloud_full_[i].x - cloud_full_[i - 1].x) > 1e-7) ||
            (abs(cloud_full_[i].y - cloud_full_[i - 1].y) > 1e-7) ||
            (abs(cloud_full_[i].z - cloud_full_[i - 1].z) > 1e-7) &&
                (cloud_full_[i].x * cloud_full_[i].x + cloud_full_[i].y * cloud_full_[i].y +
                     cloud_full_[i].z * cloud_full_[i].z >
                 (blind_ * blind_))) {
            is_valid_pt[i] = 1;
        }

        // }
    });

    for (uint i = 1; i < plsize; i++) {
        if (is_valid_pt[i]) {
            cloud_out_.points.push_back(cloud_full_[i]);
        }
    }

    cloud_out_.width = cloud_out_.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = false;
    *pcl_out = cloud_out_;
}

void PointCloudPreprocess::Oust64Handler(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
    cloud_out_.clear();
    cloud_full_.clear();

    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.size();
    cloud_out_.reserve(plsize);

    for (int i = 0; i < pl_orig.points.size(); i++) {
        if (i % point_filter_num_ != 0) {
            continue;
        }

        // 过滤 NaN/Inf 无效点
        if (!std::isfinite(pl_orig.points[i].x) || !std::isfinite(pl_orig.points[i].y) ||
            !std::isfinite(pl_orig.points[i].z)) {
            continue;
        }

        double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                       pl_orig.points[i].z * pl_orig.points[i].z;

        if (range < (blind_ * blind_)) {
            continue;
        }

        if (pl_orig.points[i].z < height_min_ || pl_orig.points[i].z > height_max_) {
            continue;
        }

        PointType added_pt;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;

        added_pt.time = pl_orig.points[i].t / 1e6;
        cloud_out_.points.push_back(added_pt);
    }

    cloud_out_.width = cloud_out_.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = false;
}

void PointCloudPreprocess::RoboSenseHandler(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
    cloud_out_.clear();
    cloud_full_.clear();

    pcl::PointCloud<PointRobotSense> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);

    int plsize = pl_orig.size();
    cloud_out_.reserve(plsize);

    double head_time = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;

    /// RoboSense的时间戳是double, 均为linux时间且单位为秒，这里减去header time并乘以1000得到毫秒为单位的时间戳

    for (int i = 0; i < pl_orig.points.size(); i++) {
        if (i % point_filter_num_ != 0) {
            continue;
        }

        // 过滤 NaN/Inf 无效点（RoboSense Airy 等会产生大量 NaN 回波，约 24%）
        if (!std::isfinite(pl_orig.points[i].x) || !std::isfinite(pl_orig.points[i].y) ||
            !std::isfinite(pl_orig.points[i].z)) {
            continue;
        }

        double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                       pl_orig.points[i].z * pl_orig.points[i].z;

        if (range < (blind_ * blind_)) {
            continue;
        }

        if (pl_orig.points[i].z < height_min_ || pl_orig.points[i].z > height_max_) {
            continue;
        }

        PointType added_pt;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;

        added_pt.time = (pl_orig.points[i].timestamp - head_time) * 1e3;  //  / 1e6;  // curvature unit: ms

        cloud_out_.points.push_back(added_pt);
    }

    cloud_out_.width = cloud_out_.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = false;
}

void PointCloudPreprocess::VelodyneHandler(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
    cloud_out_.clear();
    cloud_full_.clear();

    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    cloud_out_.reserve(plsize);

    /*** These variables only works when no point timestamps given ***/
    double omega_l = 3.61;  // scan angular velocity
    std::vector<bool> is_first(num_scans_, true);
    std::vector<double> yaw_fp(num_scans_, 0.0);    // yaw of first scan point
    std::vector<float> yaw_last(num_scans_, 0.0);   // yaw of last scan point
    std::vector<float> time_last(num_scans_, 0.0);  // last offset time
    /*****************************************************************/

    if (pl_orig.points[plsize - 1].time > 0) {
        given_offset_time_ = true;
    } else {
        given_offset_time_ = false;
        double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
        double yaw_end = yaw_first;
        int layer_first = pl_orig.points[0].ring;
        for (uint i = plsize - 1; i > 0; i--) {
            if (pl_orig.points[i].ring == layer_first) {
                yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
                break;
            }
        }
    }

    for (int i = 0; i < plsize; i++) {
        PointType added_pt;

        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.time = pl_orig.points[i].time * time_scale_;  // curvature unit: ms

        // 过滤 NaN/Inf 无效点
        if (!std::isfinite(added_pt.x) || !std::isfinite(added_pt.y) || !std::isfinite(added_pt.z)) {
            continue;
        }

        if (!given_offset_time_) {
            int layer = pl_orig.points[i].ring;
            double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

            if (is_first[layer]) {
                yaw_fp[layer] = yaw_angle;
                is_first[layer] = false;
                added_pt.time = 0.0;
                yaw_last[layer] = yaw_angle;
                time_last[layer] = added_pt.time;
                continue;
            }

            // compute offset time
            if (yaw_angle <= yaw_fp[layer]) {
                added_pt.time = (yaw_fp[layer] - yaw_angle) / omega_l;
            } else {
                added_pt.time = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
            }

            if (added_pt.time < time_last[layer]) {
                added_pt.time += 360.0 / omega_l;
            }

            yaw_last[layer] = yaw_angle;
            time_last[layer] = added_pt.time;
        }

        if (i % point_filter_num_ == 0) {
            if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > (blind_ * blind_)) {
                cloud_out_.points.push_back(added_pt);
            }
        }
    }

    cloud_out_.width = cloud_out_.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = false;
}

void PointCloudPreprocess::SimulatedHandler(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
    cloud_out_.clear();
    cloud_full_.clear();

    /// 仿真雷达（gz-sim 的 lidar / gpu_lidar / ray 等）与真实旋转/固态雷达有两点本质差异：
    ///   1) 整帧是**同一个仿真时刻**一次性投完所有射线的瞬时快照，物理上不存在扫描周期；
    ///   2) 输出的 PointCloud2 只有 x/y/z/intensity(/ring)，**没有逐点 `time`**。
    ///
    /// 若这类点云走 VelodyneHandler，`given_offset_time_ = (last.time > 0)` 会因字段缺失被判为 false，
    /// 于是退化成“按 ring + yaw 反推每点时间”的合成路径，给每点编出 0~100ms 的假采集时刻；
    /// 随后 ImuProcess::UndistortPcl 会拿这些假时间插值 IMU 做运动补偿
    /// —— 等于给本来零畸变的点云**人为注入**一个扫描周期的畸变，且畸变幅度随当前运动状态变化，
    /// 会持续污染几何、拉低建图/定位精度。
    ///
    /// 这里显式把每点时间置 0（= 扫描窗口起点）：UndistortPcl 的补偿内层循环条件是
    /// `point.time/1000 > head->offset_time`，而 IMUpose 的 offset_time 从 0 起，
    /// 所以 time = 0 的点不会被施加任何变换，瞬时几何被如实保留。
    /// 扫描结束时刻由 LaserMapping::SyncPackages 用 lidar_mean_scantime_
    /// （初值 lo::lidar_time_interval = 0.1s，与 10Hz 仿真的帧长一致）兜底，无需额外配置。
    static constexpr double kSimulatedPointTime = 0.0;

    const FieldInfo fi_x = FindField(*msg, "x");
    const FieldInfo fi_y = FindField(*msg, "y");
    const FieldInfo fi_z = FindField(*msg, "z");
    const FieldInfo fi_i = FindField(*msg, "intensity");

    if (fi_x.offset < 0 || fi_y.offset < 0 || fi_z.offset < 0) {
        LLOG_ERROR(logging::kLio, "simulated point cloud has no x/y/z field");
        return;
    }

    const int pfilter = std::max(1, point_filter_num_);
    const size_t num_pts = static_cast<size_t>(msg->width) * msg->height;
    cloud_out_.reserve(num_pts / static_cast<size_t>(pfilter));

    for (size_t i = 0; i < num_pts; ++i) {
        if (i % static_cast<size_t>(pfilter) != 0) {
            continue;
        }

        const uint8_t *base = msg->data.data() + i * msg->point_step;

        PointType added_pt;
        added_pt.x = ReadScalar(base, fi_x);
        added_pt.y = ReadScalar(base, fi_y);
        added_pt.z = ReadScalar(base, fi_z);
        // gz-sim 的 lidar 不做反射率建模，intensity 恒为 0；字段缺失时也补 0
        added_pt.intensity = (fi_i.offset >= 0) ? ReadScalar(base, fi_i) : 0.0f;
        added_pt.time = kSimulatedPointTime;

        /// 未命中 / 超量程的射线会被填成 NaN 或 ±inf，必须滤掉
        if (!std::isfinite(added_pt.x) || !std::isfinite(added_pt.y) || !std::isfinite(added_pt.z)) {
            continue;
        }

        if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z < (blind_ * blind_)) {
            continue;
        }

        if (added_pt.z < height_min_ || added_pt.z > height_max_) {
            continue;
        }

        cloud_out_.points.push_back(added_pt);
    }

    cloud_out_.width = cloud_out_.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = false;
}

}  // namespace lightning
