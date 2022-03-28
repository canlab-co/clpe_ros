#pragma once

#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/transform.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "errors.hpp"

namespace clpe
{
enum class CalibrationModel : uint32_t {
  Jhang = 0,
  FishEye = 1,
};

static constexpr const char * kPassword = "password";
static constexpr const char * kCamPose[] = {"cam_0_pose", "cam_1_pose", "cam_2_pose", "cam_3_pose"};
static constexpr const char * kCamBaseFrame[] = {"cam_0_base_frame", "cam_1_base_frame",
                                                 "cam_2_base_frame", "cam_3_base_frame"};
static constexpr const char * kCamQos[] = {"cam_0_qos", "cam_1_qos", "cam_2_qos", "cam_3_qos"};
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
static std::vector<image_transport::Publisher> kImagePubs;
static std::array<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr, 4> kInfoPubs;
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
    const auto & password = this->get_parameter(kPassword).get_value<std::string>();
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
      const auto error = this->GetCameraInfo_(i, kCamInfos[i]);
      if (error) {
        RCLCPP_FATAL(this->get_logger(), "Failed to get camera info (" + error.message() + ")");
        exit(error.value());
      }
    }
    RCLCPP_INFO(this->get_logger(), "Successfully discovered camera properties");

    // create camera publishers
    kImagePubs.reserve(4);
    for (int i = 0; i < 4; ++i) {
      kImagePubs.emplace_back(image_transport::create_publisher(
          this, "cam_" + std::to_string(i) + "/image_raw",
          GetQos_(this->get_parameter(kCamQos[i]).get_value<std::string>()).get_rmw_qos_profile()));
      kInfoPubs[i] = this->create_publisher<sensor_msgs::msg::CameraInfo>(
          "cam_" + std::to_string(i) + "/camera_info",
          this->GetQos_(this->get_parameter(kCamInfoQos[i]).get_value<std::string>()));
    }

    // start publishing
    {
      RCLCPP_INFO(this->get_logger(), "Preparing camera for streaming");
      const auto result = this->clpe_api.Clpe_StartStream(
          [](unsigned int cam_id, unsigned char * buffer, unsigned int size,
             struct timeval * frame_us) -> int {
            RCLCPP_DEBUG(kNode->get_logger(), "got new image for cam_" + std::to_string(cam_id));

            // skip all work if there is no subscribers
            if (kImagePubs[cam_id].getNumSubscribers() == 0 &&
                kInfoPubs[cam_id]->get_subscription_count() == 0) {
              RCLCPP_DEBUG(kNode->get_logger(), "skipped publishing for cam_" +
                                                    std::to_string(cam_id) +
                                                    " because there are no subscribers");
              return 0;
            }

            sensor_msgs::msg::Image image;
            Me::FillImageMsg_(buffer, size, *frame_us, image);
            kImagePubs[cam_id].publish(image);
            kInfoPubs[cam_id]->publish(kCamInfos[cam_id]);
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
  }

private:
  explicit ClpeNode(ClpeClientApi && clpe_api) : rclcpp::Node("clpe"), clpe_api(std::move(clpe_api))
  {
    // declare ros params
    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.description = "sudo password";
      desc.read_only = true;
      this->declare_parameter(kPassword, rclcpp::ParameterValue(), desc);
    }
    for (int i = 0; i < 4; ++i) {
      rcl_interfaces::msg::ParameterDescriptor tf_desc;
      tf_desc.description =
          "Pose relative to the base, 6 values corresponding to [x, y, z, roll, pitch, yaw]";
      this->declare_parameter(kCamPose[i], std::vector<double>({0, 0, 0, 0, 0, 0}));

      rcl_interfaces::msg::ParameterDescriptor base_frame_desc;
      base_frame_desc.description = "Defines the frame_id all static transformations refers to";
      base_frame_desc.read_only = true;
      this->declare_parameter(kCamBaseFrame[i], "base_link");

      rcl_interfaces::msg::ParameterDescriptor qos_desc;
      qos_desc.description =
          "Sets the QoS by which the topic is published. Available values are the following "
          "strings: SYSTEM_DEFAULT, PARAMETER_EVENTS, SERVICES_DEFAULT, PARAMETERS, DEFAULT, "
          "SENSOR_DATA, HID_DEFAULT (= DEFAULT with depth of 100), EXTRINSICS_DEFAULT (= DEFAULT "
          "with depth of 1 and transient local durabilty). Default is SENSOR_DATA.";
      qos_desc.read_only = true;
      this->declare_parameter(kCamQos[i], "SENSOR_DATA");

      rcl_interfaces::msg::ParameterDescriptor info_qos_desc;
      info_qos_desc.description =
          "Sets the QoS by which the info topic is published. Available values are the following "
          "strings: SYSTEM_DEFAULT, PARAMETER_EVENTS, SERVICES_DEFAULT, PARAMETERS, DEFAULT, "
          "SENSOR_DATA, HID_DEFAULT (= DEFAULT with depth of 100), EXTRINSICS_DEFAULT (= DEFAULT "
          "with depth of 1 and transient local durabilty). Default is SYSTEM_DEFAULT.";
      info_qos_desc.read_only = true;
      this->declare_parameter(kCamInfoQos[i], "SYSTEM_DEFAULT");
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
    image.data = std::vector<uint8_t>(buffer, buffer + size);
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

  friend class ClpeComponentNode;
};
}  // namespace clpe
