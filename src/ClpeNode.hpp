#pragma once

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "errors.hpp"

namespace clpe
{
enum class CalibrationModel : uint32_t {
  Jhang = 0,
  FishEye = 1,
};

// TODO: docs say 95 bytes, but reference sheet is 107 bytes
struct __attribute__((packed)) EepromData {
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
  uint8_t production_date[11];
};

// needed because clpe callback does not support user data :(.
static rclcpp::Node::SharedPtr kNode;
static std::vector<image_transport::CameraPublisher> kCameraPubs;
static std::array<sensor_msgs::msg::CameraInfo, 4> kCamInfos;

template <typename ClpeClientApi>
class ClpeNode : public rclcpp::Node
{
private:
  using Me = ClpeNode<ClpeClientApi>;

public:
  static std::shared_ptr<Me> make_shared(ClpeClientApi && clpe_api)
  {
    std::shared_ptr<Me> inst(new Me(std::move(clpe_api)));
    inst->transport_ = std::make_unique<image_transport::ImageTransport>(inst->shared_from_this());
    if (kNode) {
      RCLCPP_FATAL(inst->get_logger(), "only one instance allowed");
      exit(-1);
    }
    kNode = inst->shared_from_this();
    return inst;
  }

  ClpeClientApi clpe_api;

  /**
   * Initialize ClpeClient
   */
  void Init()
  {
    // FIXME: This requires sudo password!!
    const auto & password = this->get_parameter("password").get_value<std::string>();
    const auto result = this->clpe_api.Clpe_Connection(password);
    if (result != 0) {
      RCLCPP_FATAL(this->get_logger(), "Failed to initiate the clpe network connection (" +
                                           ConnectionError::get().message(result) + ")");
      exit(result);
    }

    // publish tf
    // use transient local with history depth 1 since the tf will never change.
    rclcpp::QoS tf_qos(1);
    tf_qos.reliable();
    tf_qos.transient_local();
    const auto tf_pub = this->create_publisher<geometry_msgs::msg::Transform>("tf", tf_qos);
    geometry_msgs::msg::Transform tf_msg;
    auto pose = this->GetPoseParam_();
    tf_pub->publish(Me::CreateTfMsg_(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]));

    // reading eeprom is slow so the camera info is stored and reused.
    RCLCPP_INFO(this->get_logger(), "Discovering camera properties");
    for (int i = 0; i < 4; ++i) {
      const auto error = this->GetCameraInfo_(i, kCamInfos[i]);
      if (error) {
        RCLCPP_FATAL(this->get_logger(), "Failed to get camera info (" + error.message() + ")");
        exit(error.value());
      }
    }
    RCLCPP_INFO(this->get_logger(), "Successfully discovered camera properties");

    // create camera publishers
    kCameraPubs.reserve(4);
    for (int i = 0; i < 4; ++i) {
      kCameraPubs.emplace_back(
          this->transport_->advertiseCamera("cam_" + std::to_string(i) + "/image_raw", 10));
    }

    // start publishing
    {
      RCLCPP_INFO(this->get_logger(), "Preparing camera for streaming");
      const auto result = this->clpe_api.Clpe_StartStream(
          [](unsigned int cam_id, unsigned char * buffer, unsigned int size,
             struct timeval * frame_us) -> int {
            RCLCPP_DEBUG(kNode->get_logger(), "got new image for cam_" + std::to_string(cam_id));
            sensor_msgs::msg::Image image;
            Me::FillImageMsg_(buffer, size, *frame_us, image);
            kCameraPubs[cam_id].publish(image, kCamInfos[cam_id]);
            return 0;
          },
          1, 1, 1, 1, 0);
      if (result != 0) {
        const std::error_code error(result, clpe::StartStreamError::get());
        RCLCPP_FATAL(this->get_logger(), "Failed to start streaming (" + error.message() + ")");
        exit(result);
      }
      RCLCPP_INFO(this->get_logger(), "Start streaming images");
    }

    // listen for param updates
    {
      const auto onSetParamCbHdl =
          this->add_on_set_parameters_callback([&](const std::vector<rclcpp::Parameter> & params) {
            for (const auto & p : params) {
              if (p.get_name() == "pose") {
                auto pose = this->GetPoseParam_();
                tf_pub->publish(
                    Me::CreateTfMsg_(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]));
              }
            }
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            return result;
          });
    }
  }

private:
  std::unique_ptr<image_transport::ImageTransport> transport_;

  ClpeNode(ClpeClientApi && clpe_api) : rclcpp::Node("clpe"), clpe_api(std::move(clpe_api))
  {
    // declare ros params
    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.description = "sudo password";
      desc.read_only = true;
      this->declare_parameter("password", rclcpp::ParameterValue(), desc);
    }
    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.description =
          "Pose relative to the base, 6 values corresponding to [x, y, z, roll, pitch, yaw]";
      this->declare_parameter("pose",
                              rclcpp::ParameterValue(std::vector<double>({0, 0, 0, 0, 0, 0})));
    }
  }

  std::vector<double> GetPoseParam_()
  {
    auto pose = this->get_parameter("pose").get_value<std::vector<double>>();
    if (pose.size() != 6) {
      RCLCPP_FATAL(this->get_logger(), "Failed to get pose parameter, wrong number of elements");
      exit(-1);
    }
    return pose;
  }

  static geometry_msgs::msg::Transform CreateTfMsg_(double x, double y, double z, double roll,
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
    // calibration may change anytime for self calibrating systems, so we cannot cache the cam info.
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
    return kNoError;
  }

  static void FillImageMsg_(unsigned char * buffer, unsigned int size, const timeval & timestamp,
                            sensor_msgs::msg::Image & image)
  {
    image.header.frame_id = "base_link";
    image.header.stamp.sec = timestamp.tv_sec;
    image.header.stamp.nanosec = timestamp.tv_usec * 1000;
    // buffer is only valid for 16 frames, since ros2 publish has no real time guarantees, we must
    // copy the data out to avoid UB.
    image.data.reserve(size);
    std::copy(buffer, buffer + size, image.data.data());
    image.encoding = sensor_msgs::image_encodings::YUV422;
    image.width = 1920;
    image.height = 1080;
    // assume that each row is same sized.
    image.step = size / 1080;
    image.is_bigendian = false;
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
    this->FillImageMsg_(buffer, size, timestamp, image);
    return kNoError;
  }
};
}  // namespace clpe
