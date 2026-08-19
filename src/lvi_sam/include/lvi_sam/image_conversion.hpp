#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

namespace lvi_sam::image_conversion
{

struct CvImageResult
{
  cv::Mat image;
  std::string error;

  explicit operator bool() const noexcept
  {
    return error.empty();
  }
};

struct RosImageResult
{
  sensor_msgs::msg::Image message;
  std::string error;

  explicit operator bool() const noexcept
  {
    return error.empty();
  }
};

namespace detail
{

inline bool validateImageBuffer(
  const sensor_msgs::msg::Image & message,
  const std::size_t bytes_per_pixel,
  std::string & error)
{
  if (message.width == 0U || message.height == 0U) {
    error = "image dimensions must be non-zero";
    return false;
  }
  if (message.width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
    message.height > static_cast<std::uint32_t>(std::numeric_limits<int>::max()))
  {
    error = "image dimensions exceed OpenCV limits";
    return false;
  }

  if (bytes_per_pixel == 0U ||
    static_cast<std::size_t>(message.width) >
    std::numeric_limits<std::size_t>::max() / bytes_per_pixel)
  {
    error = "image row size overflows size_t";
    return false;
  }
  const auto minimum_step = static_cast<std::size_t>(message.width) * bytes_per_pixel;
  if (static_cast<std::size_t>(message.step) < minimum_step) {
    error = "image step is smaller than width times bytes per pixel";
    return false;
  }
  if (message.step != 0U &&
    static_cast<std::size_t>(message.height) >
    std::numeric_limits<std::size_t>::max() / static_cast<std::size_t>(message.step))
  {
    error = "image buffer size overflows size_t";
    return false;
  }

  const auto required_size =
    static_cast<std::size_t>(message.height) * static_cast<std::size_t>(message.step);
  if (message.data.size() < required_size) {
    error = "image data is smaller than height times step";
    return false;
  }
  return true;
}

inline cv::Mat makeReadOnlyView(const sensor_msgs::msg::Image & message, const int cv_type)
{
  // OpenCV's external-buffer constructor takes a mutable pointer even for read-only operations.
  // The adapter never writes through this view; all persistent results own separate storage.
  return cv::Mat(
    static_cast<int>(message.height), static_cast<int>(message.width), cv_type,
    const_cast<std::uint8_t *>(message.data.data()), static_cast<std::size_t>(message.step));
}

inline bool outputTypeForEncoding(const std::string & encoding, int & cv_type)
{
  if (encoding == sensor_msgs::image_encodings::MONO8 ||
    encoding == sensor_msgs::image_encodings::TYPE_8UC1)
  {
    cv_type = CV_8UC1;
    return true;
  }
  if (encoding == sensor_msgs::image_encodings::RGB8 ||
    encoding == sensor_msgs::image_encodings::BGR8)
  {
    cv_type = CV_8UC3;
    return true;
  }
  if (encoding == sensor_msgs::image_encodings::RGBA8 ||
    encoding == sensor_msgs::image_encodings::BGRA8)
  {
    cv_type = CV_8UC4;
    return true;
  }
  return false;
}

}  // namespace detail

// Converts a ROS image to an owning mono8 matrix. RGB/BGR inputs require one color
// conversion; mono inputs require one clone because the feature tracker keeps frames
// beyond the subscription callback that owns the ROS message.
inline CvImageResult toMono8(const sensor_msgs::msg::Image & message)
{
  CvImageResult result;
  int source_type = -1;
  int conversion_code = -1;
  std::size_t bytes_per_pixel = 0U;
  bool clone_source = false;

  if (message.encoding == sensor_msgs::image_encodings::MONO8 ||
    message.encoding == sensor_msgs::image_encodings::TYPE_8UC1)
  {
    source_type = CV_8UC1;
    bytes_per_pixel = 1U;
    clone_source = true;
  } else if (message.encoding == sensor_msgs::image_encodings::RGB8) {
    source_type = CV_8UC3;
    bytes_per_pixel = 3U;
    conversion_code = cv::COLOR_RGB2GRAY;
  } else if (message.encoding == sensor_msgs::image_encodings::BGR8) {
    source_type = CV_8UC3;
    bytes_per_pixel = 3U;
    conversion_code = cv::COLOR_BGR2GRAY;
  } else if (message.encoding == sensor_msgs::image_encodings::RGBA8) {
    source_type = CV_8UC4;
    bytes_per_pixel = 4U;
    conversion_code = cv::COLOR_RGBA2GRAY;
  } else if (message.encoding == sensor_msgs::image_encodings::BGRA8) {
    source_type = CV_8UC4;
    bytes_per_pixel = 4U;
    conversion_code = cv::COLOR_BGRA2GRAY;
  } else {
    result.error = "unsupported image encoding: " + message.encoding;
    return result;
  }

  if (!detail::validateImageBuffer(message, bytes_per_pixel, result.error)) {
    return result;
  }

  try {
    const cv::Mat source = detail::makeReadOnlyView(message, source_type);
    if (clone_source) {
      result.image = source.clone();
    } else {
      cv::cvtColor(source, result.image, conversion_code);
    }
  } catch (const cv::Exception & exception) {
    result.error = "OpenCV image conversion failed: " + std::string(exception.what());
  }
  return result;
}

// Copies an OpenCV matrix into a ROS Image message. The caller must supply the
// encoding that describes the matrix; no implicit channel reordering is performed.
inline RosImageResult toRosImage(
  const cv::Mat & image,
  const std::string & encoding,
  const std_msgs::msg::Header & header = std_msgs::msg::Header())
{
  RosImageResult result;
  int expected_type = -1;
  if (!detail::outputTypeForEncoding(encoding, expected_type)) {
    result.error = "unsupported output image encoding: " + encoding;
    return result;
  }
  if (image.empty() || image.rows <= 0 || image.cols <= 0) {
    result.error = "output image must be non-empty";
    return result;
  }
  if (image.type() != expected_type) {
    result.error = "output matrix type does not match encoding " + encoding;
    return result;
  }

  const auto row_bytes = static_cast<std::size_t>(image.cols) * image.elemSize();
  if (row_bytes > std::numeric_limits<std::uint32_t>::max()) {
    result.error = "output image step exceeds ROS Image limits";
    return result;
  }
  if (static_cast<std::size_t>(image.rows) >
    std::numeric_limits<std::size_t>::max() / row_bytes)
  {
    result.error = "output image buffer size overflows size_t";
    return result;
  }

  result.message.header = header;
  result.message.height = static_cast<std::uint32_t>(image.rows);
  result.message.width = static_cast<std::uint32_t>(image.cols);
  result.message.encoding = encoding;
  result.message.is_bigendian = false;
  result.message.step = static_cast<std::uint32_t>(row_bytes);
  result.message.data.resize(static_cast<std::size_t>(image.rows) * row_bytes);

  for (int row = 0; row < image.rows; ++row) {
    std::memcpy(
      result.message.data.data() + static_cast<std::size_t>(row) * row_bytes,
      image.ptr(row), row_bytes);
  }
  return result;
}

}  // namespace lvi_sam::image_conversion
