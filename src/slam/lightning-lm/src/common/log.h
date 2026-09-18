//
// Created by lihang on 26-9-10.
//
// 统一日志门面（spdlog 后端）。
//
// 用法：
//   1. main 中调用 lightning::logging::InitFromYaml(config_path)（或 Init()）
//   2. 各处使用 LLOG_INFO/WARN/ERROR/...，第一个参数为模块名常量，其余为 fmt 格式：
//        LLOG_INFO(logging::kLio, "In num: {}, down: {}", in_num, down_num);
//        LLOG_WARN_THROTTLE(logging::kPgo, 2000, "time regress: {:.3f}", dt);   // 2 秒节流
//        LLOG_DEBUG(logging::kLidarLoc, "loc from: {}", pose.translation());    // 每帧明细放 DEBUG
//
// 级别约定：
//   TRACE/DEBUG  每帧、每关键帧明细（默认 info 级别下不输出）
//   INFO         生命周期与状态跃迁（初始化成功、地图加载、模式切换等）
//   WARN         可恢复异常（时间回退、断流、队列溢出、低置信度），高频场景需节流
//   ERROR/CRITICAL 影响结果的失败
//
// 配置（yaml，缺省时用默认值）：
//   common:
//     log:
//       level: info            # 全局默认级别
//       module_levels:         # 按模块覆盖级别（可选）
//         lidar_loc: debug
//       log_dir: "./log"       # 置空则不落盘
//       max_size_mb: 50        # 单文件大小上限，超过滚动
//       max_files: 5           # 滚动文件数上限
//

#pragma once

#ifndef LIGHTNING_LOG_H
#define LIGHTNING_LOG_H

#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <type_traits>

// 关闭 spdlog 的编译期级别门控（默认 INFO 会把 DEBUG/TRACE 宏编译为空），
// 运行期过滤统一交给 logger 级别（yaml 配置）
#ifndef SPDLOG_ACTIVE_LEVEL
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#endif

#include <spdlog/fmt/fmt.h>
#include <spdlog/spdlog.h>

// Eigen/Sophus 的 fmt formatter 依赖下面的类型定义，必须先包含
#include "common/eigen_types.h"

namespace lightning {
namespace logging {

/// 模块名（同时作为 spdlog logger 名，出现在日志行的 [模块] 字段中）
inline constexpr const char* kApp = "app";
inline constexpr const char* kCommon = "common";
inline constexpr const char* kIo = "io";
inline constexpr const char* kLio = "lio";
inline constexpr const char* kLidarLoc = "lidar_loc";
inline constexpr const char* kLoc = "loc";
inline constexpr const char* kPgo = "pgo";
inline constexpr const char* kLoop = "loop";
inline constexpr const char* kMap = "map";
inline constexpr const char* kSlam = "slam";
inline constexpr const char* kLocSys = "loc_sys";
inline constexpr const char* kUtil = "util";
inline constexpr const char* kUi = "ui";
inline constexpr const char* kMiao = "miao";
inline constexpr const char* kSelfTest = "selftest";

/// 初始化：stderr 带色 + rotating 文件双 sink；log_dir 为空则仅 stderr
void Init(const std::string& log_dir = "./log", const std::string& level = "info",
          const std::map<std::string, std::string>& module_levels = {}, size_t max_size_mb = 50,
          size_t max_files = 5);

/// 从 yaml 读取 common.log 节并初始化；文件/节点缺失时用默认值
void InitFromYaml(const std::string& yaml_path);

/// flush 并释放全部 logger，main 返回前调用
void Shutdown();

/// 获取/创建模块 logger（线程安全；Init 之前调用则退化为仅 stderr 输出）
std::shared_ptr<spdlog::logger> Get(const std::string& mod);

namespace fmt_detail {

template <typename T>
using decay_t = typename std::decay<T>::type;

template <typename T>
struct is_eigen_matrix_like : std::is_base_of<Eigen::MatrixBase<decay_t<T>>, decay_t<T>> {};

template <typename T>
struct is_eigen_quat : std::is_base_of<Eigen::QuaternionBase<decay_t<T>>, decay_t<T>> {};

template <typename T>
struct is_eigen_transform : std::false_type {};
template <typename S, int D, int M, int O>
struct is_eigen_transform<Eigen::Transform<S, D, M, O>> : std::true_type {};

template <typename T>
struct is_sophus_type
    : std::disjunction<std::is_base_of<Sophus::SE2Base<decay_t<T>>, decay_t<T>>,
                       std::is_base_of<Sophus::SE3Base<decay_t<T>>, decay_t<T>>,
                       std::is_base_of<Sophus::SO2Base<decay_t<T>>, decay_t<T>>,
                       std::is_base_of<Sophus::SO3Base<decay_t<T>>, decay_t<T>>> {};

/// 允许直接以 {} 打印的类型（沿用其 operator<< 输出格式）
template <typename T>
struct is_log_streamable
    : std::disjunction<is_eigen_matrix_like<T>, is_eigen_quat<T>, is_eigen_transform<T>, is_sophus_type<T>> {};

}  // namespace fmt_detail
}  // namespace logging
}  // namespace lightning

namespace fmt {

/// Eigen/Sophus 类型直接 {} 打印（Vec3d、SE3、Quaternion、矩阵表达式等）
template <typename T, typename Char>
struct formatter<T, Char,
                 typename std::enable_if<lightning::logging::fmt_detail::is_log_streamable<T>::value>::type> {
    constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const T& v, FormatContext& ctx) const -> decltype(ctx.out()) {
        std::stringstream ss;
        ss << v;
        return format_to(ctx.out(), "{}", ss.str());
    }
};

}  // namespace fmt

// ------------------------------ 日志宏 ------------------------------

#define LLOG_TRACE(mod, ...) SPDLOG_LOGGER_TRACE(lightning::logging::Get(mod), __VA_ARGS__)
#define LLOG_DEBUG(mod, ...) SPDLOG_LOGGER_DEBUG(lightning::logging::Get(mod), __VA_ARGS__)
#define LLOG_INFO(mod, ...) SPDLOG_LOGGER_INFO(lightning::logging::Get(mod), __VA_ARGS__)
#define LLOG_WARN(mod, ...) SPDLOG_LOGGER_WARN(lightning::logging::Get(mod), __VA_ARGS__)
#define LLOG_ERROR(mod, ...) SPDLOG_LOGGER_ERROR(lightning::logging::Get(mod), __VA_ARGS__)
#define LLOG_CRITICAL(mod, ...) SPDLOG_LOGGER_CRITICAL(lightning::logging::Get(mod), __VA_ARGS__)

/// 按时间节流：period_ms 内同一条日志至多输出一次
#define LLOG_TRACE_THROTTLE(mod, period_ms, ...)                                                     \
    do {                                                                                             \
        static std::atomic<int64_t> llog_last_ns{0};                                                 \
        const int64_t llog_now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(            \
                                        std::chrono::steady_clock::now().time_since_epoch())         \
                                        .count();                                                    \
        int64_t llog_prev_ns = llog_last_ns.load(std::memory_order_relaxed);                         \
        if (llog_now_ns - llog_prev_ns >= static_cast<int64_t>(period_ms) * 1000000 &&               \
            llog_last_ns.compare_exchange_strong(llog_prev_ns, llog_now_ns, std::memory_order_relaxed)) { \
            SPDLOG_LOGGER_TRACE(lightning::logging::Get(mod), __VA_ARGS__);                              \
        }                                                                                            \
    } while (0)

#define LLOG_DEBUG_THROTTLE(mod, period_ms, ...)                                                     \
    do {                                                                                             \
        static std::atomic<int64_t> llog_last_ns{0};                                                 \
        const int64_t llog_now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(            \
                                        std::chrono::steady_clock::now().time_since_epoch())         \
                                        .count();                                                    \
        int64_t llog_prev_ns = llog_last_ns.load(std::memory_order_relaxed);                         \
        if (llog_now_ns - llog_prev_ns >= static_cast<int64_t>(period_ms) * 1000000 &&               \
            llog_last_ns.compare_exchange_strong(llog_prev_ns, llog_now_ns, std::memory_order_relaxed)) { \
            SPDLOG_LOGGER_DEBUG(lightning::logging::Get(mod), __VA_ARGS__);                              \
        }                                                                                            \
    } while (0)

#define LLOG_INFO_THROTTLE(mod, period_ms, ...)                                                      \
    do {                                                                                             \
        static std::atomic<int64_t> llog_last_ns{0};                                                 \
        const int64_t llog_now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(            \
                                        std::chrono::steady_clock::now().time_since_epoch())         \
                                        .count();                                                    \
        int64_t llog_prev_ns = llog_last_ns.load(std::memory_order_relaxed);                         \
        if (llog_now_ns - llog_prev_ns >= static_cast<int64_t>(period_ms) * 1000000 &&               \
            llog_last_ns.compare_exchange_strong(llog_prev_ns, llog_now_ns, std::memory_order_relaxed)) { \
            SPDLOG_LOGGER_INFO(lightning::logging::Get(mod), __VA_ARGS__);                               \
        }                                                                                            \
    } while (0)

#define LLOG_WARN_THROTTLE(mod, period_ms, ...)                                                      \
    do {                                                                                             \
        static std::atomic<int64_t> llog_last_ns{0};                                                 \
        const int64_t llog_now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(            \
                                        std::chrono::steady_clock::now().time_since_epoch())         \
                                        .count();                                                    \
        int64_t llog_prev_ns = llog_last_ns.load(std::memory_order_relaxed);                         \
        if (llog_now_ns - llog_prev_ns >= static_cast<int64_t>(period_ms) * 1000000 &&               \
            llog_last_ns.compare_exchange_strong(llog_prev_ns, llog_now_ns, std::memory_order_relaxed)) { \
            SPDLOG_LOGGER_WARN(lightning::logging::Get(mod), __VA_ARGS__);                            \
        }                                                                                            \
    } while (0)

#define LLOG_ERROR_THROTTLE(mod, period_ms, ...)                                                     \
    do {                                                                                             \
        static std::atomic<int64_t> llog_last_ns{0};                                                 \
        const int64_t llog_now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(            \
                                        std::chrono::steady_clock::now().time_since_epoch())         \
                                        .count();                                                    \
        int64_t llog_prev_ns = llog_last_ns.load(std::memory_order_relaxed);                         \
        if (llog_now_ns - llog_prev_ns >= static_cast<int64_t>(period_ms) * 1000000 &&               \
            llog_last_ns.compare_exchange_strong(llog_prev_ns, llog_now_ns, std::memory_order_relaxed)) { \
            SPDLOG_LOGGER_ERROR(lightning::logging::Get(mod), __VA_ARGS__);                              \
        }                                                                                            \
    } while (0)

#define LLOG_CRITICAL_THROTTLE(mod, period_ms, ...)                                                  \
    do {                                                                                             \
        static std::atomic<int64_t> llog_last_ns{0};                                                 \
        const int64_t llog_now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(            \
                                        std::chrono::steady_clock::now().time_since_epoch())         \
                                        .count();                                                    \
        int64_t llog_prev_ns = llog_last_ns.load(std::memory_order_relaxed);                         \
        if (llog_now_ns - llog_prev_ns >= static_cast<int64_t>(period_ms) * 1000000 &&               \
            llog_last_ns.compare_exchange_strong(llog_prev_ns, llog_now_ns, std::memory_order_relaxed)) { \
            SPDLOG_LOGGER_CRITICAL(lightning::logging::Get(mod), __VA_ARGS__);                           \
        }                                                                                            \
    } while (0)

/// 每 n 次调用输出一次（用于每帧日志按帧数降频）
#define LLOG_TRACE_EVERY_N(mod, n, ...)                                                               \
    do {                                                                                              \
        static std::atomic<uint64_t> llog_cnt{0};                                                     \
        if (llog_cnt.fetch_add(1, std::memory_order_relaxed) % (n) == 0) {                            \
            SPDLOG_LOGGER_TRACE(lightning::logging::Get(mod), __VA_ARGS__);                               \
        }                                                                                             \
    } while (0)

#define LLOG_DEBUG_EVERY_N(mod, n, ...)                                                               \
    do {                                                                                              \
        static std::atomic<uint64_t> llog_cnt{0};                                                     \
        if (llog_cnt.fetch_add(1, std::memory_order_relaxed) % (n) == 0) {                            \
            SPDLOG_LOGGER_DEBUG(lightning::logging::Get(mod), __VA_ARGS__);                               \
        }                                                                                             \
    } while (0)

#define LLOG_INFO_EVERY_N(mod, n, ...)                                                                \
    do {                                                                                              \
        static std::atomic<uint64_t> llog_cnt{0};                                                     \
        if (llog_cnt.fetch_add(1, std::memory_order_relaxed) % (n) == 0) {                            \
            SPDLOG_LOGGER_INFO(lightning::logging::Get(mod), __VA_ARGS__);                                \
        }                                                                                             \
    } while (0)

#define LLOG_WARN_EVERY_N(mod, n, ...)                                                                \
    do {                                                                                              \
        static std::atomic<uint64_t> llog_cnt{0};                                                     \
        if (llog_cnt.fetch_add(1, std::memory_order_relaxed) % (n) == 0) {                            \
            SPDLOG_LOGGER_WARN(lightning::logging::Get(mod), __VA_ARGS__);                             \
        }                                                                                             \
    } while (0)

#define LLOG_ERROR_EVERY_N(mod, n, ...)                                                               \
    do {                                                                                              \
        static std::atomic<uint64_t> llog_cnt{0};                                                     \
        if (llog_cnt.fetch_add(1, std::memory_order_relaxed) % (n) == 0) {                            \
            SPDLOG_LOGGER_ERROR(lightning::logging::Get(mod), __VA_ARGS__);                               \
        }                                                                                             \
    } while (0)

#define LLOG_CRITICAL_EVERY_N(mod, n, ...)                                                            \
    do {                                                                                              \
        static std::atomic<uint64_t> llog_cnt{0};                                                     \
        if (llog_cnt.fetch_add(1, std::memory_order_relaxed) % (n) == 0) {                            \
            SPDLOG_LOGGER_CRITICAL(lightning::logging::Get(mod), __VA_ARGS__);                            \
        }                                                                                             \
    } while (0)

#endif  // LIGHTNING_LOG_H
