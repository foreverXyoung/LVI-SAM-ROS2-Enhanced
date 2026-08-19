#include <gtest/gtest.h>

#include <lvi_sam_msgs/msg/localization_status.hpp>

TEST(LocalizationStatusContract, StableStateValues) {
  using Status = lvi_sam_msgs::msg::LocalizationStatus;
  EXPECT_EQ(Status::WAITING_FOR_SENSORS, 0u);
  EXPECT_EQ(Status::RELOCALIZING, 1u);
  EXPECT_EQ(Status::VERIFYING, 2u);
  EXPECT_EQ(Status::TRACKING, 3u);
  EXPECT_EQ(Status::DEGRADED, 4u);
  EXPECT_EQ(Status::LOST, 5u);
  EXPECT_EQ(Status::ERROR, 6u);
  EXPECT_EQ(Status::MAPPING, 7u);
  EXPECT_EQ(Status::STATE_UNKNOWN, 255u);
}

TEST(LocalizationStatusContract, StableModeValues) {
  using Status = lvi_sam_msgs::msg::LocalizationStatus;
  EXPECT_EQ(Status::MODE_MAPPING, 0u);
  EXPECT_EQ(Status::MODE_LOCALIZATION, 1u);
}
