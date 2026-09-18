//
// Created by xiang on 25-4-22.
//

#ifndef LIGHTNING_POINTCLOUD_UTILS_H
#define LIGHTNING_POINTCLOUD_UTILS_H

#include "common/point_def.h"

#include <cmath>
#include <cstdint>

namespace lightning {

/// 体素滤波
CloudPtr VoxelGrid(CloudPtr cloud, float voxel_size = 0.05);

/// 移除地面
void RemoveGround(CloudPtr cloud, float z_min = 0.5);

/// 点 -> 体素索引的 64bit key。每轴 21bit 有符号：
/// res=0.1 时覆盖 ±104km，res=0.01 时覆盖 ±10.4km，建图足够。
///
/// 用途：跨多次调用做「同一个体素只保留一个点」的去重。
/// 增量地图发布靠它把「同一面墙被 N 个关键帧各发一遍」压成「只发一遍」，
/// 否则 rviz 累积起来的点数会随运行时间无界增长。
/// 负数直接取低 21bit（补码），效果等价于给它加了个偏移，相邻体素仍然相邻。
inline uint64_t VoxelKey(const PointType& p, float res) {
    constexpr uint64_t kMask = 0x1FFFFFULL;  // 21 bit
    const int64_t ix = static_cast<int64_t>(std::floor(p.x / res));
    const int64_t iy = static_cast<int64_t>(std::floor(p.y / res));
    const int64_t iz = static_cast<int64_t>(std::floor(p.z / res));
    return ((static_cast<uint64_t>(ix) & kMask) << 42) | ((static_cast<uint64_t>(iy) & kMask) << 21) |
           (static_cast<uint64_t>(iz) & kMask);
}

}  // namespace lightning

#endif  // LIGHTNING_POINTCLOUD_UTILS_H
