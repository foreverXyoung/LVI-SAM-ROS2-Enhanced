#pragma once

#include <cstddef>

namespace lvi_sam::internal_odom_metadata {

// These indices describe compatibility metadata carried by internal
// nav_msgs/Odometry topics. They are not statistical covariance entries and
// must not be used on public navigation interfaces. See
// docs/INTERFACES_AND_STABILITY.md for the topic-specific contracts.
namespace mapping_correction {
inline constexpr std::size_t kResetId = 0;
inline constexpr std::size_t kDegenerate = 1;
}  // namespace mapping_correction

namespace visual_prior {
inline constexpr std::size_t kResetId = 0;
inline constexpr std::size_t kAccelBiasX = 1;
inline constexpr std::size_t kAccelBiasY = 2;
inline constexpr std::size_t kAccelBiasZ = 3;
inline constexpr std::size_t kGyroBiasX = 4;
inline constexpr std::size_t kGyroBiasY = 5;
inline constexpr std::size_t kGyroBiasZ = 6;
inline constexpr std::size_t kGravity = 7;
inline constexpr std::size_t kLastRequiredIndex = kGravity;
}  // namespace visual_prior

}  // namespace lvi_sam::internal_odom_metadata
