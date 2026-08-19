#include "lvi_sam/topic_names.hpp"

#include <gtest/gtest.h>

TEST(TopicNames, ProducesAbsoluteProjectTopic)
{
    EXPECT_EQ(
        lvi_sam::topics::project_topic("lvi_sam", lvi_sam::topics::kFeature),
        "/lvi_sam/vins/feature/feature");
}

TEST(TopicNames, NormalizesProjectSlashes)
{
    EXPECT_EQ(
        lvi_sam::topics::project_topic("/robot_a/", lvi_sam::topics::kOdometry),
        "/robot_a/vins/odometry/odometry");
}

TEST(TopicNames, RejectsInvalidInputs)
{
    EXPECT_THROW(
        lvi_sam::topics::project_topic("///", lvi_sam::topics::kFeature),
        std::invalid_argument);
    EXPECT_THROW(
        lvi_sam::topics::project_topic("lvi_sam", "vins/feature/feature"),
        std::invalid_argument);
    EXPECT_THROW(
        lvi_sam::topics::project_topic("robot//camera", lvi_sam::topics::kFeature),
        std::invalid_argument);
    EXPECT_THROW(
        lvi_sam::topics::project_topic("robot-a", lvi_sam::topics::kFeature),
        std::invalid_argument);
    EXPECT_THROW(
        lvi_sam::topics::project_topic("robot/2camera", lvi_sam::topics::kFeature),
        std::invalid_argument);
}
