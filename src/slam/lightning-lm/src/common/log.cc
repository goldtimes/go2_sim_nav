//
// Created by lihang on 26-9-10.
//

#include "common/log.h"

#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <yaml-cpp/yaml.h>

#include <cstdio>
#include <filesystem>
#include <mutex>
#include <unordered_map>

#include <unistd.h>  // getpid

namespace lightning {
namespace logging {

namespace {

std::mutex g_mutex;
std::vector<spdlog::sink_ptr> g_sinks;
std::unordered_map<std::string, spdlog::level::level_enum> g_module_levels;
spdlog::level::level_enum g_default_level = spdlog::level::info;
bool g_inited = false;

// stderr：精简格式；文件：附源码位置，便于事后排查
constexpr const char* kStderrPattern = "[%H:%M:%S.%e][%^%l%$][%n] %v";
constexpr const char* kFilePattern = "[%Y-%m-%d %H:%M:%S.%e][%^%l%$][%n][%s:%#] %v";

spdlog::level::level_enum ParseLevel(const std::string& s, spdlog::level::level_enum fallback) {
    const auto lvl = spdlog::level::from_str(s);
    // from_str 对未知字符串返回 off，这里区分用户真的配了 off
    return (lvl == spdlog::level::off && s != "off") ? fallback : lvl;
}

void CreateLoggerLocked(const std::string& mod) {
    auto lg = std::make_shared<spdlog::logger>(mod, g_sinks.begin(), g_sinks.end());
    const auto it = g_module_levels.find(mod);
    lg->set_level(it != g_module_levels.end() ? it->second : g_default_level);
    lg->flush_on(spdlog::level::warn);
    spdlog::register_logger(lg);
}

}  // namespace

std::shared_ptr<spdlog::logger> Get(const std::string& mod) {
    std::lock_guard<std::mutex> lock(g_mutex);
    if (auto lg = spdlog::get(mod)) {
        return lg;
    }
    if (g_sinks.empty()) {
        // Init 之前的兜底：仅 stderr，保证早期日志不丢
        auto sink = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
        sink->set_pattern(kStderrPattern);
        g_sinks.push_back(sink);
    }
    CreateLoggerLocked(mod);
    return spdlog::get(mod);
}

void Init(const std::string& log_dir, const std::string& level,
          const std::map<std::string, std::string>& module_levels, size_t max_size_mb, size_t max_files) {
    std::lock_guard<std::mutex> lock(g_mutex);
    if (g_inited) {
        return;
    }

    g_default_level = ParseLevel(level, spdlog::level::info);
    g_module_levels.clear();
    for (const auto& kv : module_levels) {
        g_module_levels[kv.first] = ParseLevel(kv.second, g_default_level);
    }

    auto stderr_sink = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
    stderr_sink->set_pattern(kStderrPattern);
    g_sinks.push_back(stderr_sink);

    if (!log_dir.empty()) {
        try {
            std::filesystem::create_directories(log_dir);
            // 文件名带 pid：同机同时跑建图/定位两个进程时不互相覆盖
            const std::string filename =
                log_dir + "/lightning-" + std::to_string(static_cast<int>(getpid())) + ".log";
            auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
                filename, max_size_mb * 1024 * 1024, max_files);
            file_sink->set_pattern(kFilePattern);
            g_sinks.push_back(file_sink);
        } catch (const spdlog::spdlog_ex& e) {
            fprintf(stderr, "[log] create file sink failed: %s, fallback to stderr only\n", e.what());
        }
    }

    // 预注册 miao logger：miao 库不依赖本门面，运行时拾取同名 logger 以共享文件 sink
    CreateLoggerLocked(kMiao);

    g_inited = true;
}

void InitFromYaml(const std::string& yaml_path) {
    std::string log_dir = "./log";
    std::string level = "info";
    std::map<std::string, std::string> module_levels;
    size_t max_size_mb = 50;
    size_t max_files = 5;

    try {
        const auto yaml = YAML::LoadFile(yaml_path);
        const auto node = yaml["common"]["log"];
        if (node && node.IsMap()) {
            if (node["level"]) {
                level = node["level"].as<std::string>();
            }
            if (node["log_dir"]) {
                log_dir = node["log_dir"].as<std::string>();
            }
            if (node["max_size_mb"]) {
                max_size_mb = node["max_size_mb"].as<size_t>();
            }
            if (node["max_files"]) {
                max_files = node["max_files"].as<size_t>();
            }
            if (node["module_levels"] && node["module_levels"].IsMap()) {
                for (const auto& it : node["module_levels"]) {
                    module_levels[it.first.as<std::string>()] = it.second.as<std::string>();
                }
            }
        }
    } catch (const std::exception& e) {
        fprintf(stderr, "[log] read log config from %s failed: %s, use defaults\n", yaml_path.c_str(), e.what());
    }

    Init(log_dir, level, module_levels, max_size_mb, max_files);
}

void Shutdown() {
    std::lock_guard<std::mutex> lock(g_mutex);
    spdlog::shutdown();  // flush 全部 logger 并从 registry 移除
    g_sinks.clear();
    g_module_levels.clear();
    g_inited = false;
}

}  // namespace logging
}  // namespace lightning
