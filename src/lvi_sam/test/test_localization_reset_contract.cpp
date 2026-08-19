#include <gtest/gtest.h>

#include <lvi_sam_msgs/msg/localization_reset.hpp>
#include <lvi_sam_msgs/msg/localization_status.hpp>

TEST(LocalizationResetContract, StableReasonValues) {
  using Reset = lvi_sam_msgs::msg::LocalizationReset;
  EXPECT_EQ(Reset::REASON_RELOCALIZATION, 0u);
  EXPECT_EQ(Reset::REASON_FORCE_RELOCALIZATION, 1u);
  EXPECT_EQ(Reset::REASON_MAP_CORRECTION, 2u);
  EXPECT_EQ(Reset::REASON_VINS_FAILURE, 3u);
  EXPECT_EQ(Reset::REASON_IMU_FAILURE, 4u);
  EXPECT_EQ(Reset::REASON_IMAGE_STREAM_RESET, 5u);
  EXPECT_EQ(Reset::REASON_MANUAL, 6u);
}

TEST(LocalizationResetContract, StatusCarriesResetGeneration) {
  using Status = lvi_sam_msgs::msg::LocalizationStatus;
  Status status;
  status.reset_id = 17u;
  EXPECT_EQ(status.reset_id, 17u);
}
