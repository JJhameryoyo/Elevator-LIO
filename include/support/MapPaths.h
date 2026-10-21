#pragma once

#include <filesystem>
#include <stdexcept>
#include <string>

namespace elevator_lio {
inline std::filesystem::path resolveMapSaveDirectory(
    const std::string& configured, const std::filesystem::path& package_root,
    const std::string& home) {
    if (configured.empty()) throw std::runtime_error("pcd_save.save_dir must not be empty");
    std::filesystem::path result;
    if (configured == "~" || configured.rfind("~/", 0) == 0) {
        if (home.empty()) throw std::runtime_error("HOME is not set; use an absolute pcd_save.save_dir");
        result = std::filesystem::path(home);
        if (configured.size() > 2) result /= configured.substr(2);
    } else {
        if (configured.front() == '~') {
            throw std::runtime_error("pcd_save.save_dir supports ~/ but not ~user paths");
        }
        result = configured;
        if (result.is_relative()) result = package_root / result;
    }
    return std::filesystem::absolute(result).lexically_normal();
}
}  // namespace elevator_lio
