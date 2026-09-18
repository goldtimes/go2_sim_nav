//
// Created by gx on 23-11-23.
//

#include "utils/timer.h"

#include <algorithm>
#include <fstream>
#include <numeric>

namespace lightning {

std::map<std::string, Timer::TimerRecord> Timer::records_;
std::mutex Timer::records_mutex_;

void Timer::PrintAll() {
    /// 加锁：PrintAll 会对每个计时项的 deque 做 sort，而 Evaluate 可能仍在写
    LLOG_INFO(logging::kUtil, ">>> ===== Printing run time =====");
    std::lock_guard<std::mutex> lk(records_mutex_);
    for (auto& r : records_) {
        auto& rec = r.second.time_usage_in_ms_;
        std::sort(rec.begin(), rec.end());

        LLOG_INFO(logging::kUtil, "> [{}] average time usage: {:.3f} ms, med: {:.3f} 95%: {:.3f}, called times : {}",
                  r.first, std::accumulate(rec.begin(), rec.end(), 0.0) / double(rec.size()), rec[rec.size() * 0.5],
                  rec[rec.size() * 0.95], r.second.time_usage_in_ms_.size());
    }
    LLOG_INFO(logging::kUtil, ">>> ===== Printing run time end =====");
}

void Timer::DumpIntoFile(const std::string& file_name) {
    std::ofstream ofs(file_name, std::ios::out);
    if (!ofs.is_open()) {
        LLOG_ERROR(logging::kUtil, "Failed to open file: {}", file_name);
        return;
    } else {
        LLOG_INFO(logging::kUtil, "Dump Time Records into file: {}", file_name);
    }

    size_t max_length = 0;
    for (const auto& iter : records_) {
        ofs << iter.first << ", ";
        if (iter.second.time_usage_in_ms_.size() > max_length) {
            max_length = iter.second.time_usage_in_ms_.size();
        }
    }
    ofs << std::endl;

    for (size_t i = 0; i < max_length; ++i) {
        for (const auto& iter : records_) {
            if (i < iter.second.time_usage_in_ms_.size()) {
                ofs << iter.second.time_usage_in_ms_[i] << ",";
            } else {
                ofs << ",";
            }
        }
        ofs << std::endl;
    }
    ofs.close();
}

double Timer::GetMeanTime(const std::string& func_name) {
    if (records_.find(func_name) == records_.end()) {
        return 0.0;
    }

    auto r = records_[func_name];
    return std::accumulate(r.time_usage_in_ms_.begin(), r.time_usage_in_ms_.end(), 0.0) /
           double(r.time_usage_in_ms_.size());
}
}  // namespace lightning
