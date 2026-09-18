//
// Created by lightning on 25-9-17.
//

#pragma once

#include <cstdlib>
#include <string>

namespace lightning {
namespace path_utils {

/// 展开路径开头的 "~" / "~/" 为用户 HOME 目录。
///  - "~"          -> $HOME
///  - "~/maps"     -> $HOME/maps
///  - "~user/maps" -> 原样返回（不支持，避免猜错）
///  - $HOME 不可用 -> 原样返回
/// 地图路径在多台机器（用户名不同）之间共用配置时必须走这里，
/// 否则 yaml 里写死 /home/<user>/... 换机器就失效。
inline std::string ExpandUser(const std::string& path) {
    if (path.empty() || path[0] != '~') {
        return path;
    }

    const char* home_env = std::getenv("HOME");
    if (home_env == nullptr) {
        return path;
    }
    const std::string home = home_env;

    if (path.size() == 1) {
        return home;
    }
    if (path[1] == '/') {
        return home + path.substr(1);
    }
    return path;
}

/// 去掉尾部 '/'（根目录 "/" 保留）。
/// TiledMap 内部用 map_path + "/index.txt" 拼接，带尾斜杠会产生 "//"，
/// 虽然能跑但日志难看，且与上层传入的路径比对时容易判成不同目录。
inline std::string StripTrailingSlash(const std::string& path) {
    std::string out = path;
    while (out.size() > 1 && out.back() == '/') {
        out.pop_back();
    }
    return out;
}

/// 目录路径归一化：先展开 ~，再去尾部 '/'。path 为空时用 fallback。
inline std::string ResolveDir(const std::string& path, const std::string& fallback = "") {
    return StripTrailingSlash(ExpandUser(path.empty() ? fallback : path));
}

}  // namespace path_utils
}  // namespace lightning
