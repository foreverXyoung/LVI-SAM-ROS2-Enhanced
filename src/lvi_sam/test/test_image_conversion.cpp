#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "lvi_sam/image_conversion.hpp"

namespace image_conversion = lvi_sam::image_conversion;

TEST(ImageConversion, ConvertsRgb8ToOwningMono8)
{
  sensor_msgs::msg::Image input;
  input.width = 2;
  input.height = 1;
  input.encoding = sensor_msgs::image_encodings::RGB8;
  input.step = 6;
  input.data = {255, 0, 0, 0, 255, 0};

  auto result = image_conversion::toMono8(input);

  ASSERT_TRUE(result) << result.error;
  ASSERT_EQ(result.image.type(), CV_8UC1);
  EXPECT_EQ(result.image.at<std::uint8_t>(0, 0), 76);
  EXPECT_EQ(result.image.at<std::uint8_t>(0, 1), 150);

  input.data[0] = 0;
  EXPECT_EQ(result.image.at<std::uint8_t>(0, 0), 76);
}

TEST(ImageConversion, CopiesMono8WithPaddedRows)
{
  sensor_msgs::msg::Image input;
  input.width = 2;
  input.height = 2;
  input.encoding = sensor_msgs::image_encodings::MONO8;
  input.step = 4;
  input.data = {1, 2, 99, 99, 3, 4, 99, 99};

  auto result = image_conversion::toMono8(input);

  ASSERT_TRUE(result) << result.error;
  EXPECT_EQ(result.image.at<std::uint8_t>(0, 0), 1);
  EXPECT_EQ(result.image.at<std::uint8_t>(0, 1), 2);
  EXPECT_EQ(result.image.at<std::uint8_t>(1, 0), 3);
  EXPECT_EQ(result.image.at<std::uint8_t>(1, 1), 4);
  EXPECT_TRUE(result.image.isContinuous());
}

TEST(ImageConversion, RejectsInvalidInputBuffer)
{
  sensor_msgs::msg::Image input;
  input.width = 2;
  input.height = 2;
  input.encoding = sensor_msgs::image_encodings::RGB8;
  input.step = 6;
  input.data.resize(6);

  const auto result = image_conversion::toMono8(input);

  EXPECT_FALSE(result);
  EXPECT_NE(result.error.find("smaller"), std::string::npos);
}

TEST(ImageConversion, RejectsUnknownEncoding)
{
  sensor_msgs::msg::Image input;
  input.width = 1;
  input.height = 1;
  input.encoding = "yuv-custom";
  input.step = 1;
  input.data = {0};

  const auto result = image_conversion::toMono8(input);

  EXPECT_FALSE(result);
  EXPECT_NE(result.error.find("unsupported"), std::string::npos);
}

TEST(ImageConversion, PublishesOnlyActivePixelsFromPaddedMatrix)
{
  std::array<std::uint8_t, 16> storage{
    1, 2, 3, 4, 5, 6, 99, 99,
    7, 8, 9, 10, 11, 12, 99, 99};
  const cv::Mat image(2, 2, CV_8UC3, storage.data(), 8);

  const auto result = image_conversion::toRosImage(
    image, sensor_msgs::image_encodings::RGB8);

  ASSERT_TRUE(result) << result.error;
  EXPECT_EQ(result.message.width, 2U);
  EXPECT_EQ(result.message.height, 2U);
  EXPECT_EQ(result.message.step, 6U);
  const std::vector<std::uint8_t> expected{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
  EXPECT_EQ(result.message.data, expected);
}

TEST(ImageConversion, RejectsOutputTypeEncodingMismatch)
{
  const cv::Mat mono(1, 1, CV_8UC1, cv::Scalar(0));

  const auto result = image_conversion::toRosImage(
    mono, sensor_msgs::image_encodings::RGB8);

  EXPECT_FALSE(result);
  EXPECT_NE(result.error.find("does not match"), std::string::npos);
}
