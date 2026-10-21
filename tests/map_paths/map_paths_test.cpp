#include "support/MapPaths.h"
#include <filesystem>
#include <iostream>
#include <stdexcept>

int main() {
    namespace fs = std::filesystem;
    using elevator_lio::resolveMapSaveDirectory;
    const auto root = fs::temp_directory_path() / "elevator-map-path-test";
    const auto home = root / "home" / "robot";
    const auto package = root / "workspace" / "lio";
    auto expect = [](bool condition) {
        if (!condition) throw std::runtime_error("map output path assertion failed");
    };
    expect(resolveMapSaveDirectory("~/home/maps", package, home.string()) == home / "home" / "maps");
    expect(resolveMapSaveDirectory("~", package, home.string()) == home);
    expect(resolveMapSaveDirectory("~/", package, home.string()) == home);
    expect(resolveMapSaveDirectory("PCD", package, "") == package / "PCD");
    expect(resolveMapSaveDirectory((root / "maps").string(), package, "") == root / "maps");
    expect(resolveMapSaveDirectory("~/home/../maps", package, home.string()) == home / "maps");
    auto rejects = [&](const std::string& input, const std::string& home_dir) {
        try { resolveMapSaveDirectory(input, package, home_dir); }
        catch (const std::runtime_error&) { return true; }
        return false;
    };
    expect(rejects("", home.string()));
    expect(rejects("~/home/maps", ""));
    expect(rejects("~someone/maps", home.string()));
    std::cout << "PASS: 9 map output path checks\n";
}
