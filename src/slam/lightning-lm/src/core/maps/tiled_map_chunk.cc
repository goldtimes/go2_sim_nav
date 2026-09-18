//
// Created by xiang on 23-2-7.
//

#include "tiled_map_chunk.h"

#include "common/log.h"

#include <pcl/io/pcd_io.h>

namespace lightning {

void MapChunk::AddPoint(const PointType& pt) {
    if (cloud_ == nullptr) {
        cloud_.reset(new PointCloudType);
        cloud_->reserve(50000);
    }
    cloud_->points.emplace_back(pt);
}

void MapChunk::LoadCloud() {
    if (cloud_ == nullptr) {
        cloud_.reset(new PointCloudType);
    }
    cloud_->clear();

    /// 注意：loadPCDFile 失败（文件缺失/损坏）时只返回负值，不会抛异常。
    /// 原先忽略返回值并无条件 loaded_ = true，会让"地图加载成功"变成假象：
    /// 区块对象存在但里面一个点都没有，后续 GICP 匹配会莫名失败。
    const int ret = pcl::io::loadPCDFile(filename_, *cloud_);
    if (ret < 0) {
        LLOG_ERROR(logging::kMap, "failed to load map chunk ({}): {}", ret, filename_);
        cloud_->clear();
        loaded_ = false;
        return;
    }

    loaded_ = true;
}

void MapChunk::Unload() {
    cloud_ = nullptr;
    loaded_ = false;
}

}  // namespace lightning