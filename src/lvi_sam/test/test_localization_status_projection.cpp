#include <gtest/gtest.h>

#include <lvi_sam/localization_status_projection.hpp>

namespace {

using lvi_sam::localization_status::LegacyState;
using lvi_sam::localization_status::ProjectionInput;
using lvi_sam::localization_status::project_state;
using Status = lvi_sam_msgs::msg::LocalizationStatus;

TEST(LocalizationStatusProjection, PreservesMappingMode) {
  ProjectionInput input;
  input.mapping_mode = true;
  input.legacy_state = LegacyState::Initialized;
  EXPECT_EQ(project_state(input), Status::MAPPING);
}

TEST(LocalizationStatusProjection, MapsInitializationStatesToRelocalizing) {
  ProjectionInput input;
  input.legacy_state = LegacyState::NonInitialized;
  EXPECT_EQ(project_state(input), Status::RELOCALIZING);
  input.legacy_state = LegacyState::Initializing;
  EXPECT_EQ(project_state(input), Status::RELOCALIZING);
}

TEST(LocalizationStatusProjection, MapsInitializedStateToTracking) {
  ProjectionInput input;
  input.legacy_state = LegacyState::Initialized;
  EXPECT_EQ(project_state(input), Status::TRACKING);
}

TEST(LocalizationStatusProjection, PreservesTransientLossEvent) {
  ProjectionInput input;
  input.legacy_state = LegacyState::NonInitialized;
  input.loss_event_pending = true;
  EXPECT_EQ(project_state(input), Status::LOST);

  input.loss_event_pending = false;
  EXPECT_EQ(project_state(input), Status::RELOCALIZING);
}

}  // namespace
