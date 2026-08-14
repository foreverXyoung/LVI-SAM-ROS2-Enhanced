#pragma once

#include <cstdint>

#include <lvi_sam_msgs/msg/localization_status.hpp>

namespace lvi_sam::localization_status {

// This enum mirrors the frozen LocInitSta values in mapOptimization.cpp. It
// intentionally lives at the adapter boundary; it is not a replacement for
// the algorithm's internal state machine.
enum class LegacyState : std::uint8_t {
  NonInitialized = 0,
  Initializing = 1,
  Initialized = 2,
  MayLost = 3,
};

struct ProjectionInput {
  bool mapping_mode = false;
  LegacyState legacy_state = LegacyState::NonInitialized;
  bool loss_event_pending = false;
};

inline std::uint8_t project_state(const ProjectionInput &input) {
  using Status = lvi_sam_msgs::msg::LocalizationStatus;
  if (input.mapping_mode) {
    return Status::MAPPING;
  }
  if (input.loss_event_pending || input.legacy_state == LegacyState::MayLost) {
    return Status::LOST;
  }
  switch (input.legacy_state) {
    case LegacyState::NonInitialized:
    case LegacyState::Initializing:
      return Status::RELOCALIZING;
    case LegacyState::Initialized:
      return Status::TRACKING;
    case LegacyState::MayLost:
      return Status::LOST;
    default:
      return Status::STATE_UNKNOWN;
  }
}

}  // namespace lvi_sam::localization_status
