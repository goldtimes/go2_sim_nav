//
// Created by gx on 23-11-23.
//
#pragma once

#include <chrono>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "common/log.h"

namespace lightning {

/// 统计时间工具
class Timer {
   public:
    struct TimerRecord {
        TimerRecord() = default;
        TimerRecord(const std::string& name, double time_usage) {
            func_name_ = name;
            time_usage_in_ms_.emplace_back(time_usage);
        }
        std::string func_name_;
        std::deque<double> time_usage_in_ms_;
    };

    /**
     * 评价并记录函数用时
     * @tparam F
     * @param func
     * @param func_name
     */
    template <class F>
    static void Evaluate(F&& func, const std::string& func_name, bool print = false) {
        auto t1 = std::chrono::steady_clock::now();
        std::forward<F>(func)();
        auto t2 = std::chrono::steady_clock::now();
        auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;

        /// records_ 是静态的，而 Evaluate 会被多个线程并发调用：
        ///   executor 线程 -> Proc Lidar / Preprocess
        ///   worker  线程 -> LIO Run / PublishMappingTf / PublishGlobalMap / 各子阶段
        /// 不加锁就是对 std::map 的并发写，会直接破坏红黑树（可能崩）。
        /// 只锁“记账”这一小段，**不包含被测函数**，所以不影响计时精度。
        {
            std::lock_guard<std::mutex> lk(records_mutex_);
            auto& rec = records_[func_name];
            rec.time_usage_in_ms_.emplace_back(time_used);
            while (rec.time_usage_in_ms_.size() > kMaxSamples) {
                rec.time_usage_in_ms_.pop_front();
            }
        }

        if (print) {
            LLOG_DEBUG(logging::kUtil, "func <{}> timer: {:.3f} ms", func_name, time_used);
        }
    }

    /// 打印记录的所有耗时
    static void PrintAll();

    /// 写入文件，方便作图分析
    static void DumpIntoFile(const std::string& file_name);

    /// 获取某个函数的平均执行时间
    static double GetMeanTime(const std::string& func_name);

    /// 清理记录
    static void Clear() { records_.clear(); }

   private:
    /// 每个计时项最多保留的样本数（超出后丢最旧的），避免长时间运行内存无界增长。
    /// 注意：PrintAll 输出的“called times”是这个**样本数**，不是真实总调用次数。
    static constexpr size_t kMaxSamples = 2000;

    static std::map<std::string, TimerRecord> records_;
    /// 保护 records_（Evaluate 会被多线程调用）
    static std::mutex records_mutex_;
};
}  // namespace lightning
