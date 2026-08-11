#pragma once

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <filesystem>
#include <stdexcept>
#include <string>

namespace lvi_sam
{

inline std::string resolve_package_asset(
    const std::string &configured_path, const std::string &description)
{
    if (configured_path.empty())
        throw std::runtime_error(description + " path must not be empty");

    const std::filesystem::path requested(configured_path);
    if (requested.is_absolute())
    {
        if (std::filesystem::is_regular_file(requested))
            return requested.string();

        // Only the upstream /config/... spelling is interpreted as a legacy
        // package-relative path. Other missing absolute paths remain errors;
        // silently rebasing them would hide deployment mistakes.
        if (requested.generic_string().rfind("/config/", 0) != 0)
            throw std::runtime_error(
                description + " file does not exist: " + requested.string());
    }

    // Older configurations use /config/... to mean package-relative. Preserve
    // that convention while allowing a real, existing absolute path above.
    std::filesystem::path relative = requested;
    if (requested.is_absolute())
        relative = requested.lexically_relative(requested.root_path());
    relative = relative.lexically_normal();
    for (const auto &component : relative)
    {
        if (component == "..")
            throw std::runtime_error(
                description + " package-relative path must not contain '..'");
    }

    const std::filesystem::path candidate =
        std::filesystem::path(
            ament_index_cpp::get_package_share_directory("lvi_sam")) /
        relative;
    if (!std::filesystem::is_regular_file(candidate))
        throw std::runtime_error(
            description + " file does not exist: " + candidate.string());
    return candidate.string();
}

}  // namespace lvi_sam
