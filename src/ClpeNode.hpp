/*
 * Copyright (C) 2022 Can-lab Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#pragma once

#include <unordered_map>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Transform.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2/LinearMath/Quaternion.h>

#include "errors.hpp"

namespace clpe
{
enum class CalibrationModel : uint32_t
{
  Jhang = 0,
  FishEye = 1,
};

// Supported types by cv_bridge
// https://github.com/ros-perception/vision_opencv/blob/c791220cefd0abf02c6719e2ce0fea465857a88e/cv_bridge/include/cv_bridge/cv_bridge.h#L202
// using array instead of enum to allow iteration.
static constexpr std::array<const char*, 6> kSupportedEncodings({
    // cv_bridge only supports converting to from color to mono of the same channels, so "mono8"
    // is not supported.
    // "mono8"
    "bgr8",
    "bgra8",
    "rgb8",
    "rgba8",
    "mono16",
    // camera encoding
    "yuv422",
});

static constexpr const char* kPassword = "password";
static constexpr const char* kEncoding = "encoding";
static constexpr const char* kCamEnable[] = { "cam_0_enable", "cam_1_enable", "cam_2_enable", "cam_3_enable" };
static constexpr const char* kCamPose[] = { "cam_0_pose", "cam_1_pose", "cam_2_pose", "cam_3_pose" };
static constexpr const char* kCamBaseFrame[] = { "cam_0_frame_id", "cam_1_frame_id", "cam_2_frame_id",
                                                 "cam_3_frame_id" };
static constexpr const char* kCamQueueSize[] = { "cam_0_image_queue_size", "cam_1_image_queue_size",
                                                 "cam_2_image_queue_size", "cam_3_image_queue_size" };
static constexpr const char* kCamLatch[] = { "cam_0_image_latch", "cam_1__image_latch", "cam_2_image_latch",
                                             "cam_3_image_latch" };

// TODO: docs say 95 bytes, but reference sheet is 107 bytes
template <typename ClpeClientApi>
class ClpeNode : public ros::NodeHandle, public std::enable_shared_from_this<ClpeNode<ClpeClientApi>>
{
private:
  using Me = ClpeNode<ClpeClientApi>;

public:
  static std::shared_ptr<Me> make_shared(ClpeClientApi&& clpe_api)
  {
    std::shared_ptr<Me> inst(new Me(std::move(clpe_api)));
    inst->transport_ = std::make_unique<image_transport::ImageTransport>(*inst->shared_from_this());
    if (Me::kNode_)
    {
      ROS_FATAL("only one instance allowed");
      exit(-1);
    }
    Me::kNode_ = inst->shared_from_this().get();
    return inst;
  }

  ClpeClientApi clpe_api;

  /**
   * Initialize ClpeClient
   */
  void Init()
  {
    // FIXME: This requires sudo password!!
    std::string password;
    if (!this->getParam(kPassword, password))
    {
      ROS_FATAL("Password is required");
      exit(-1);
    }
    const auto result = this->clpe_api.Clpe_Connection(password);
    if (result != 0)
    {
      ROS_FATAL("Failed to initiate the clpe network connection (%s)", ConnectionError::get().message(result).c_str());
      exit(result);
    }

    // publish tf
    for (int i = 0; i < 4; ++i)
    {
      const auto tf_pub = this->advertise<geometry_msgs::Transform>("cam_" + std::to_string(i) + "/tf", 1, true);
      geometry_msgs::Transform tf_msg;
      auto pose = this->GetPoseParam_(i);
      tf_pub.publish(Me::CreateTfMsg_(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]));
    }

    // reading eeprom is slow so the camera info is stored and reused.
    ROS_INFO("Discovering camera properties");
    for (int i = 0; i < 4; ++i)
    {
      if (!this->cam_enabled_[i])
      {
        ROS_INFO("Skipped cam_%i  because it is not enabled", i);
        continue;
      }
    }
    ROS_INFO("Successfully discovered camera properties");

    // create camera publishers
    kImagePubs.reserve(4);
    for (int i = 0; i < 4; ++i)
    {
      if (!this->cam_enabled_[i])
      {
        continue;
      }
      bool latch = this->param<bool>(kCamLatch[i], false);
      int queue_size = this->param<int>(kCamQueueSize[i], 10);
      kImagePubs[i] = this->transport_->advertise("cam_" + std::to_string(i) + "/image_raw", queue_size, latch);
    }

    // start publishing
    {
      ROS_INFO("Preparing camera for streaming");
      const auto result = this->clpe_api.Clpe_StartStream(
          [](unsigned int cam_id, unsigned char* buffer, unsigned int size, struct timeval* frame_us) -> int {
            ROS_DEBUG("got new image for cam_%i", cam_id);

            // skip all work if there is no subscribers
            if (kImagePubs[cam_id].getNumSubscribers() == 0)
            {
              ROS_DEBUG("skipped publishing for cam_%i because there are no subscribers", cam_id);
              return 0;
            }

            sensor_msgs::Image image;
            const auto frame_id = Me::kNode_->param<std::string>(kCamBaseFrame[cam_id], "base_link");
            const ros::Time stamp = ros::Time::now();
            Me::FillImageMsg_(buffer, size, stamp, frame_id, image, Me::kNode_->encoding_);
            kImagePubs[cam_id].publish(image);
            return 0;
          },
          static_cast<int>(this->cam_enabled_[0]), static_cast<int>(this->cam_enabled_[1]),
          static_cast<int>(this->cam_enabled_[2]), static_cast<int>(this->cam_enabled_[3]), 0);
      if (result != 0)
      {
        const std::error_code error(result, clpe::StartStreamError::get());
        ROS_FATAL("Failed to start streaming (%s)", error.message().c_str());
        exit(result);
      }
      ROS_INFO("Start streaming images");
    }
  }

private:
  // needed because clpe callback does not support user data :(.
  static Me* kNode_;
  static std::unordered_map<int, image_transport::Publisher> kImagePubs;

  std::unique_ptr<image_transport::ImageTransport> transport_;
  std::string encoding_;
  std::array<bool, 4> cam_enabled_;

  explicit ClpeNode(ClpeClientApi&& clpe_api) : ros::NodeHandle("~"), clpe_api(std::move(clpe_api))
  {
    this->encoding_ = this->GetEncoding_();
    for (int i = 0; i < 4; ++i)
    {
      this->cam_enabled_[i] = this->param(kCamEnable[i], true);
    }
  }

  std::vector<double> GetPoseParam_(int cam_id)
  {
    std::vector<double> defaultPose({ 0, 0, 0, 0, 0, 0 });
    std::vector<double> pose = this->param<std::vector<double>>(kCamPose[cam_id], defaultPose);
    if (pose.size() != 6)
    {
      ROS_FATAL("Failed to get pose parameter, wrong number of elements");
      exit(-1);
    }
    return pose;
  }

  static geometry_msgs::Transform CreateTfMsg_(double x, double y, double z, double roll, double pitch, double yaw)
  {
    geometry_msgs::Transform tf_msg;
    tf_msg.translation.x = x;
    tf_msg.translation.y = y;
    tf_msg.translation.z = z;
    tf2::Quaternion quat;
    quat.setRPY(roll, pitch, yaw);
    tf_msg.rotation.x = quat.x();
    tf_msg.rotation.y = quat.y();
    tf_msg.rotation.z = quat.z();
    tf_msg.rotation.w = quat.w();
    return tf_msg;
  }


  static void FillImageMsg_(unsigned char* buffer, unsigned int size, const ros::Time& stamp,
                            const std::string& frame_id, sensor_msgs::Image& image, const std::string& encoding)
  {
    image.header.frame_id = frame_id;
    // buffer is only valid for 16 frames, since ros2 publish has no real time guarantees, we must
    // copy the data out to avoid UB.
    image.data = std::vector<uint8_t>(buffer, buffer + size);
    image.encoding = sensor_msgs::image_encodings::YUV422;
    image.width = 1920;
    image.height = 1080;
    // assume that each row is same sized.
    image.step = size / 1080;
    image.is_bigendian = false;
    image.header.stamp = stamp;

    if (encoding != "yuv422")
    {
      auto cv_image = cv_bridge::toCvCopy(image, encoding);
      cv_image->toImageMsg(image);
    }
  }

  std::error_code GetCameraImage_(int cam_id, sensor_msgs::Image& image)
  {
    unsigned char* buffer;
    unsigned int size;
    timeval timestamp;
    const auto result = this->clpe_api.Clpe_GetFrameOneCam(cam_id, &buffer, &size, &timestamp);
    if (result != 0)
    {
      return std::error_code(result, GetFrameError::get());
    }
    this->FillImageMsg_(buffer, size, ros::Time::now(), this->param<std::string>(kCamBaseFrame[cam_id], "base_link"),
                        image, this->encoding_);
    return kNoError;
  }

  std::string GetEncoding_()
  {
    const auto enc = this->param<std::string>(kEncoding, "yuv422");
    if (std::find(kSupportedEncodings.begin(), kSupportedEncodings.end(), enc) == kSupportedEncodings.end())
    {
      ROS_FATAL("Unsupported encoding");
      exit(-1);
    }
    return enc;
  }

  friend class ClpeComponentNode;
};

template <typename ClpeClientApi>
ClpeNode<ClpeClientApi>* ClpeNode<ClpeClientApi>::kNode_;

template <typename ClpeClientApi>
std::unordered_map<int, image_transport::Publisher> ClpeNode<ClpeClientApi>::kImagePubs;

}  // namespace clpe
