//
// miao 自包含日志：直连 spdlog，不依赖 lightning 的 common/log.h 门面。
// 独立使用（examples）时自动创建仅 stderr 的 "miao" logger；
// 在 lightning 进程内运行时，common/log.cc 初始化阶段会预注册共享双 sink 的 "miao" logger，此处直接拾取。
//
#pragma once

#ifndef MIAO_LOG_H
#define MIAO_LOG_H

#ifndef SPDLOG_ACTIVE_LEVEL
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#endif

#include <memory>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

namespace miao {

inline std::shared_ptr<spdlog::logger>& logger() {
    static std::shared_ptr<spdlog::logger> lg = []() -> std::shared_ptr<spdlog::logger> {
        if (auto existing = spdlog::get("miao")) {
            return existing;
        }
        return spdlog::stderr_color_mt("miao");
    }();
    return lg;
}

}  // namespace miao

// 注意用 ::miao 根限定：miao 代码位于 lightning::miao 内，裸 miao:: 会被解析到外层命名空间
#define MLOG_TRACE(...) SPDLOG_LOGGER_TRACE(::miao::logger(), __VA_ARGS__)
#define MLOG_DEBUG(...) SPDLOG_LOGGER_DEBUG(::miao::logger(), __VA_ARGS__)
#define MLOG_INFO(...) SPDLOG_LOGGER_INFO(::miao::logger(), __VA_ARGS__)
#define MLOG_WARN(...) SPDLOG_LOGGER_WARN(::miao::logger(), __VA_ARGS__)
#define MLOG_ERROR(...) SPDLOG_LOGGER_ERROR(::miao::logger(), __VA_ARGS__)
#define MLOG_CRITICAL(...) SPDLOG_LOGGER_CRITICAL(::miao::logger(), __VA_ARGS__)

#endif  // MIAO_LOG_H
