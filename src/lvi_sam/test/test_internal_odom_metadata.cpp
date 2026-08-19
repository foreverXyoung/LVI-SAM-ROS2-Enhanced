#include "lvi_sam/internal_odom_metadata.hpp"

#include <gtest/gtest.h>

TEST(InternalOdomMetadata, MappingCorrectionLayoutIsStable)
{
    using namespace lvi_sam::internal_odom_metadata::mapping_correction;
    EXPECT_EQ(kResetId, 0U);
    EXPECT_EQ(kDegenerate, 1U);
}

TEST(InternalOdomMetadata, VisualPriorLayoutIsStable)
{
    using namespace lvi_sam::internal_odom_metadata::visual_prior;
    EXPECT_EQ(kResetId, 0U);
    EXPECT_EQ(kAccelBiasX, 1U);
    EXPECT_EQ(kAccelBiasY, 2U);
    EXPECT_EQ(kAccelBiasZ, 3U);
    EXPECT_EQ(kGyroBiasX, 4U);
    EXPECT_EQ(kGyroBiasY, 5U);
    EXPECT_EQ(kGyroBiasZ, 6U);
    EXPECT_EQ(kGravity, 7U);
    EXPECT_EQ(kLastRequiredIndex, kGravity);
}
