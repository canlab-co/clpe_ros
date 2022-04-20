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
#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <clpe_ros_msgs/msg/clpe_camera_info.hpp>

#include "errors.hpp"

//==============================================================================
namespace clpe
{
//==============================================================================
enum class CalibrationModel : uint32_t
{
  Jhang = 0,
  FishEye = 1,
};

// Supported types by cv_bridge
// https://github.com/ros-perception/vision_opencv/blob/c791220cefd0abf02c6719e2ce0fea465857a88e/cv_bridge/include/cv_bridge/cv_bridge.h#L202
// using array instead of enum to allow iteration.
static constexpr std::array<const char *, 6> kSupportedEncodings({
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

static constexpr const char * kPassword = "password";
static constexpr const char * kEncoding = "encoding";
static constexpr const char * kCamEnable[] = {"cam_0_enable", "cam_1_enable", "cam_2_enable",
  "cam_3_enable"};
static constexpr const char * kCamPose[] = {"cam_0_pose", "cam_1_pose", "cam_2_pose", "cam_3_pose"};
static constexpr const char * kCamBaseFrame[] = {"cam_0_frame_id", "cam_1_frame_id",
  "cam_2_frame_id", "cam_3_frame_id"};
static constexpr const char * kCamQos[] = {"cam_0_image_qos", "cam_1_image_qos", "cam_2_image_qos",
  "cam_3_image_qos"};
static constexpr const char * kCamInfoQos[] = {"cam_0_info_qos", "cam_1_info_qos", "cam_2_info_qos",
  "cam_3_info_qos"};
static constexpr const char * kQosSystemDefault = "SYSTEM_DEFAULT";
static constexpr const char * kQosParameterEvents = "PARAMETER_EVENTS";
static constexpr const char * kQosServicesDefault = "SERVICES_DEFAULT";
static constexpr const char * kQosParameters = "PARAMETERS";
static constexpr const char * kQosDefault = "DEFAULT";
static constexpr const char * kQosSensorData = "SENSOR_DATA";
static constexpr const char * kQosHidDefault = "HID_DEFAULT";
static constexpr const char * kQosExtrinsicsDefault = "EXTRINSICS_DEFAULT";

//==============================================================================
struct __attribute__((packed)) EepromData
{
  uint16_t signature_code;
  uint64_t version;
  CalibrationModel calibration_model;
  float fx;
  float fy;
  float cx;
  float cy;
  float k1;
  float k2;
  float k3;
  float k4;
  float rms;
  float fov;
  float calibration_temperature;
  uint8_t reserved1[20];
  float p1;
  float p2;
  uint8_t reserved2[8];
  uint16_t checksum;
  char production_date[11];
};

template<typename ClpeClientApi>
class ClpeNode : public rclcpp::Node
{
private:
  using Me = ClpeNode<ClpeClientApi>;

public:
  static std::shared_ptr<Me> make_shared(
    ClpeClientApi && clpe_api, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  {
    std::shared_ptr<Me> inst(new Me(std::move(clpe_api), options));
    if (Me::kNode_) {
      RCLCPP_FATAL(inst->get_logger(), "only one instance allowed");
      exit(-1);
    }
    Me::kNode_ = static_cast<Me *>(inst->shared_from_this().get());
    return inst;
  }

  ClpeClientApi clpe_api;

  /**
   * Initialize ClpeClient
   */
  void Init()
  {
    // FIXME: This requires sudo password!!
    const auto & password = this->get_parameter(kPassword).get_value<std::string>();
    const auto result = this->clpe_api.Clpe_Connection(password);
    if (result != 0) {
      RCLCPP_FATAL(
        this->get_logger(), "Failed to initiate the clpe network connection (%s)",
        ConnectionError::get().message(result).c_str());
      exit(result);
    }

    // publish tf
    // use transient local with history depth 1 since the tf will never change.
    rclcpp::QoS tf_qos(1);
    tf_qos.reliable();
    tf_qos.transient_local();
    for (int i = 0; i < 4; ++i) {
      const auto tf_pub = this->create_publisher<geometry_msgs::msg::Transform>(
        "cam_" + std::to_string(i) + "/tf", tf_qos);
      geometry_msgs::msg::Transform tf_msg;
      auto pose = this->GetPoseParam_(i);
      tf_pub->publish(Me::CreateTfMsg_(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]));
    }

    // reading eeprom is slow so the camera info is stored and reused.
    RCLCPP_INFO(this->get_logger(), "Discovering camera properties");
    for (int i = 0; i < 4; ++i) {
      if (!this->cam_enabled_[i]) {
        RCLCPP_INFO(
          this->get_logger(), "Skipped cam_%s because it is not enabled",
          std::to_string(i).c_str());
        continue;
      }
      {
        const auto error = this->GetCameraInfo_(i, kCamInfos[i]);
        if (error) {
          RCLCPP_FATAL(
            this->get_logger(), "Failed to get camera info (%s)",
            error.message().c_str());
          exit(error.value());
        }
      }
      {
        const auto error = this->GetClpeCameraInfo_(i, kClpeCamInfos[i]);
        if (error) {
          RCLCPP_FATAL(
            this->get_logger(), "Failed to get camera info (%s)",
            error.message().c_str());
          exit(error.value());
        }
      }
    }
    RCLCPP_INFO(this->get_logger(), "Successfully discovered camera properties");

    // create camera publishers
    kImagePubs.reserve(4);
    for (int i = 0; i < 4; ++i) {
      if (!this->cam_enabled_[i]) {
        continue;
      }
      kImagePubs[i] = image_transport::create_publisher(
        this, "cam_" + std::to_string(i) + "/image_raw",
        GetQos_(this->get_parameter(kCamQos[i]).get_value<std::string>()).get_rmw_qos_profile());
      kInfoPubs[i] = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "cam_" + std::to_string(i) + "/camera_info",
        this->GetQos_(this->get_parameter(kCamInfoQos[i]).get_value<std::string>()));
      kClpeInfoPubs[i] = this->create_publisher<clpe_ros_msgs::msg::ClpeCameraInfo>(
        "cam_" + std::to_string(i) + "/clpe_camera_info",
        this->GetQos_(this->get_parameter(kCamInfoQos[i]).get_value<std::string>()));
    }

    // start publishing
    {
      RCLCPP_INFO(this->get_logger(), "Preparing camera for streaming");
      const auto result = this->clpe_api.Clpe_StartStream(
        [](unsigned int cam_id, unsigned char * buffer, unsigned int size,
        struct timeval *) -> int {
          const auto start_time = std::chrono::steady_clock::now();
          RCLCPP_DEBUG(
            Me::kNode_->get_logger(), "got new image for cam_%s",
            std::to_string(cam_id).c_str());

          // skip all work if there is no subscribers
          if (kImagePubs[cam_id].getNumSubscribers() == 0 &&
          kInfoPubs[cam_id]->get_subscription_count() == 0 &&
          kClpeInfoPubs[cam_id]->get_subscription_count() == 0)
          {
            RCLCPP_DEBUG(
              Me::kNode_->get_logger(),
              "skipped publishing for cam_%s because there are no subscribers",
              std::to_string(cam_id).c_str());
            return 0;
          }

          sensor_msgs::msg::Image image;
          const auto frame_id =
          Me::kNode_->get_parameter(kCamBaseFrame[cam_id]).get_value<std::string>();
          const rclcpp::Time stamp = Me::kNode_->get_clock()->now();
          Me::FillImageMsg_(buffer, size, stamp, frame_id, image, Me::kNode_->encoding_);
          const auto time_after_fill = std::chrono::steady_clock::now();
          kImagePubs[cam_id].publish(image);
          kCamInfos[cam_id].header.frame_id = frame_id;
          kCamInfos[cam_id].header.stamp = stamp;
          kInfoPubs[cam_id]->publish(kCamInfos[cam_id]);
          kClpeInfoPubs[cam_id]->publish(kClpeCamInfos[cam_id]);
          const auto time_after_pub = std::chrono::steady_clock::now();

          RCLCPP_DEBUG(
            Me::kNode_->get_logger(),
            "Time to fill msg: %ld us. Time to publish: %ld us",
            (time_after_fill - start_time).count() / 1000,
            (time_after_pub - time_after_fill).count() / 1000);
          return 0;
        },
        static_cast<int>(this->cam_enabled_[0]), static_cast<int>(this->cam_enabled_[1]),
        static_cast<int>(this->cam_enabled_[2]), static_cast<int>(this->cam_enabled_[3]), 0);
      if (result != 0) {
        const std::error_code error(result, clpe::StartStreamError::get());
        RCLCPP_FATAL(this->get_logger(), "Failed to start streaming (%s)", error.message().c_str());
        exit(result);
      }
      RCLCPP_INFO(this->get_logger(), "Start streaming images");
    }
  }

private:
  // needed because clpe callback does not support user data :(.
  static Me * kNode_;
  static std::unordered_map<int, image_transport::Publisher> kImagePubs;
  static std::array<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr, 4> kInfoPubs;
  static std::array<sensor_msgs::msg::CameraInfo, 4> kCamInfos;
  static std::array<rclcpp::Publisher<clpe_ros_msgs::msg::ClpeCameraInfo>::SharedPtr,
    4> kClpeInfoPubs;
  static std::array<clpe_ros_msgs::msg::ClpeCameraInfo, 4> kClpeCamInfos;

  std::string encoding_;
  std::array<bool, 4> cam_enabled_;

  explicit ClpeNode(ClpeClientApi && clpe_api, const rclcpp::NodeOptions & options)
  : rclcpp::Node("clpe", options), clpe_api(std::move(clpe_api))
  {
    // declare ros params
    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.description = "sudo password";
      desc.read_only = true;
      this->declare_parameter(kPassword, rclcpp::ParameterValue(), desc);
    }
    for (int i = 0; i < 4; ++i) {
      rcl_interfaces::msg::ParameterDescriptor enable_desc;
      enable_desc.description = "Enable camera";
      this->declare_parameter(kCamEnable[i], true, enable_desc);

      rcl_interfaces::msg::ParameterDescriptor tf_desc;
      tf_desc.description =
        "Pose relative to the base, 6 values corresponding to [x, y, z, roll, pitch, yaw]";
      this->declare_parameter(kCamPose[i], std::vector<double>({0, 0, 0, 0, 0, 0}), tf_desc);

      rcl_interfaces::msg::ParameterDescriptor base_frame_desc;
      base_frame_desc.description = "Defines the frame_id all static transformations refers to";
      base_frame_desc.read_only = true;
      this->declare_parameter(kCamBaseFrame[i], "base_link", base_frame_desc);

      rcl_interfaces::msg::ParameterDescriptor qos_desc;
      qos_desc.description =
        "Sets the QoS by which the topic is published. Available values are the following "
        "strings: SYSTEM_DEFAULT, PARAMETER_EVENTS, SERVICES_DEFAULT, PARAMETERS, DEFAULT, "
        "SENSOR_DATA, HID_DEFAULT (= DEFAULT with depth of 100), EXTRINSICS_DEFAULT (= DEFAULT "
        "with depth of 1 and transient local durability). Default is SENSOR_DATA.";
      qos_desc.read_only = true;
      this->declare_parameter(kCamQos[i], "SENSOR_DATA", qos_desc);

      rcl_interfaces::msg::ParameterDescriptor info_qos_desc;
      info_qos_desc.description =
        "Sets the QoS by which the info topic is published. Available values are the following "
        "strings: SYSTEM_DEFAULT, PARAMETER_EVENTS, SERVICES_DEFAULT, PARAMETERS, DEFAULT, "
        "SENSOR_DATA, HID_DEFAULT (= DEFAULT with depth of 100), EXTRINSICS_DEFAULT (= DEFAULT "
        "with depth of 1 and transient local durability). Default is SYSTEM_DEFAULT.";
      info_qos_desc.read_only = true;
      this->declare_parameter(kCamInfoQos[i], "SYSTEM_DEFAULT", info_qos_desc);
    }
    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.description =
        "Image encoding, supported formats are: bgr8, bgra8, rgb8, rgba8, mono16, yuv422. "
        "Defaults to yuv422. Note that encodings other than yuv422 incurs conversion overhead.";
      desc.read_only = true;
      this->declare_parameter(kEncoding, "yuv422", desc);
    }

    this->encoding_ = this->GetEncoding_();
    for (int i = 0; i < 4; ++i) {
      this->cam_enabled_[i] = this->get_parameter(kCamEnable[i]).get_value<bool>();
    }
  }

  std::vector<double> GetPoseParam_(int cam_id)
  {
    auto pose = this->get_parameter(kCamPose[cam_id]).get_value<std::vector<double>>();
    if (pose.size() != 6) {
      RCLCPP_FATAL(this->get_logger(), "Failed to get pose parameter, wrong number of elements");
      exit(-1);
    }
    return pose;
  }

  static geometry_msgs::msg::Transform CreateTfMsg_(
    double x, double y, double z, double roll,
    double pitch, double yaw)
  {
    geometry_msgs::msg::Transform tf_msg;
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

  /**
   * Reading the camera's eeprom is slow so callers should cache the result
   */
  std::error_code GetCameraInfo_(int cam_id, sensor_msgs::msg::CameraInfo & cam_info)
  {
    // reset to defaults
    cam_info = sensor_msgs::msg::CameraInfo();
    // calibration may change anytime for self calibrating systems, so we cannot cache the cam
    // info.
    cam_info.width = 1920;
    cam_info.height = 1080;
    EepromData eeprom_data;
    const auto result =
      this->clpe_api.Clpe_GetEepromData(cam_id, reinterpret_cast<unsigned char *>(&eeprom_data));
    if (result != 0) {
      return std::error_code(result, GetEepromDataError::get());
    }
    switch (eeprom_data.calibration_model) {
      case CalibrationModel::Jhang:
        cam_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
        break;
      case CalibrationModel::FishEye:
        cam_info.distortion_model = sensor_msgs::distortion_models::EQUIDISTANT;
        break;
    }
    cam_info.k = {eeprom_data.fx, 0, eeprom_data.cx, 0, eeprom_data.fy, eeprom_data.cy, 0, 0, 1};
    cam_info.d = {eeprom_data.k1, eeprom_data.k2, eeprom_data.p1,
      eeprom_data.p2, eeprom_data.k3, eeprom_data.k4};
    cam_info.p = {
      eeprom_data.fx, 0, eeprom_data.cx, 0,
      0, eeprom_data.fy, eeprom_data.cy, 0,
      0, 0, 1, 0
    };
    return kNoError;
  }

  std::error_code GetClpeCameraInfo_(int cam_id, clpe_ros_msgs::msg::ClpeCameraInfo & msg)
  {
    EepromData eeprom_data;
    const auto result =
      this->clpe_api.Clpe_GetEepromData(cam_id, reinterpret_cast<unsigned char *>(&eeprom_data));
    if (result != 0) {
      return std::error_code(result, GetEepromDataError::get());
    }

    msg.calibration_model = static_cast<uint32_t>(eeprom_data.calibration_model);
    msg.fx = eeprom_data.fx;
    msg.fy = eeprom_data.fy;
    msg.cx = eeprom_data.cx;
    msg.cy = eeprom_data.cy;
    msg.k1 = eeprom_data.k1;
    msg.k2 = eeprom_data.k2;
    msg.k3 = eeprom_data.k3;
    msg.k4 = eeprom_data.k4;
    msg.rms = eeprom_data.rms;
    msg.fov = eeprom_data.fov;
    msg.p1 = eeprom_data.p1;
    msg.p2 = eeprom_data.p2;
    msg.production_date = std::string(eeprom_data.production_date);
    return kNoError;
  }

  static void FillImageMsg_(
    unsigned char * buffer, unsigned int size, const rclcpp::Time & stamp,
    const std::string & frame_id, sensor_msgs::msg::Image & image,
    const std::string & encoding)
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

    if (encoding != "yuv422") {
      auto cv_image = cv_bridge::toCvCopy(image, encoding);
      cv_image->toImageMsg(image);
    }
  }

  std::error_code GetCameraImage_(int cam_id, sensor_msgs::msg::Image & image)
  {
    unsigned char * buffer;
    unsigned int size;
    timeval timestamp;
    const auto result = this->clpe_api.Clpe_GetFrameOneCam(cam_id, &buffer, &size, &timestamp);
    if (result != 0) {
      return std::error_code(result, GetFrameError::get());
    }
    this->FillImageMsg_(
      buffer, size, Me::kNode->get_clock()->now(),
      this->get_parameter(kCamBaseFrame[cam_id]).get_value<std::string>(), image,
      this->encoding_);
    return kNoError;
  }

  static rclcpp::QoS GetQos_(const std::string & val)
  {
    if (val == kQosSystemDefault) {
      return rclcpp::SystemDefaultsQoS();
    } else if (val == kQosParameterEvents) {
      return rclcpp::ParameterEventsQoS();
    } else if (val == kQosServicesDefault) {
      return rclcpp::ServicesQoS();
    } else if (val == kQosParameters) {
      return rclcpp::ParametersQoS();
    } else if (val == kQosDefault) {
      return rclcpp::QoS(10);
    } else if (val == kQosSensorData) {
      return rclcpp::SensorDataQoS();
    } else if (val == kQosHidDefault) {
      return rclcpp::QoS(100);
    } else if (val == kQosExtrinsicsDefault) {
      auto qos = rclcpp::QoS(1);
      qos.transient_local();
      return qos;
    }
    return rclcpp::SystemDefaultsQoS();
  }

  std::string GetEncoding_()
  {
    const auto enc = this->get_parameter(kEncoding).get_value<std::string>();
    if (std::find(kSupportedEncodings.begin(), kSupportedEncodings.end(), enc) ==
      kSupportedEncodings.end())
    {
      RCLCPP_FATAL(this->get_logger(), "Unsupported encoding");
      exit(-1);
    }
    return enc;
  }

  friend class ClpeComponentNode;
};

template<typename ClpeClientApi>
ClpeNode<ClpeClientApi> * ClpeNode<ClpeClientApi>::kNode_;

template<typename ClpeClientApi>
std::unordered_map<int, image_transport::Publisher> ClpeNode<ClpeClientApi>::kImagePubs;

template<typename ClpeClientApi>
std::array<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr, 4>
ClpeNode<ClpeClientApi>::kInfoPubs;

template<typename ClpeClientApi>
std::array<sensor_msgs::msg::CameraInfo, 4> ClpeNode<ClpeClientApi>::kCamInfos;

template<typename ClpeClientApi>
std::array<rclcpp::Publisher<clpe_ros_msgs::msg::ClpeCameraInfo>::SharedPtr, 4>
ClpeNode<ClpeClientApi>::kClpeInfoPubs;

template<typename ClpeClientApi>
std::array<clpe_ros_msgs::msg::ClpeCameraInfo, 4> ClpeNode<ClpeClientApi>::kClpeCamInfos;

}  // namespace clpe
