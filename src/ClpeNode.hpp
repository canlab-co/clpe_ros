#pragma once

#include <geometry_msgs/Transform.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2/LinearMath/Quaternion.h>

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
static constexpr const char * kCamQueueSize[] = {"cam_0_queue_size", "cam_1_queue_size",
                                                 "cam_2_queue_size", "cam_3_queue_size"};
static constexpr const char * kCamLatch[] = {"cam_0_latch", "cam_1_latch", "cam_2_latch",
                                             "cam_3_latch"};
static constexpr const char * kCamInfoLatch[] = {"cam_0_info_latch", "cam_1_info_latch",
                                                 "cam_2_info_latch", "cam_3_info_latch"};
static constexpr const char * kCamInfoQueueSize[] = {
    "cam_0_info_queue_size", "cam_1_info_queue_size", "cam_2_info_queue_size",
    "cam_3_info_queue_size"};
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
static ros::NodeHandle * kNode;
static std::vector<image_transport::Publisher> kImagePubs;
static std::array<ros::Publisher, 4> kInfoPubs;
static std::array<sensor_msgs::CameraInfo, 4> kCamInfos;

template <typename ClpeClientApi>
class ClpeNode : public ros::NodeHandle,
                 public std::enable_shared_from_this<ClpeNode<ClpeClientApi>>
{
private:
  using Me = ClpeNode<ClpeClientApi>;

public:
  static std::shared_ptr<Me> make_shared(ClpeClientApi && clpe_api)
  {
    std::shared_ptr<Me> inst(new Me(std::move(clpe_api)));
    inst->transport_ = std::make_unique<image_transport::ImageTransport>(*inst->shared_from_this());
    if (kNode) {
      ROS_FATAL("only one instance allowed");
      exit(-1);
    }
    kNode = inst->shared_from_this().get();
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
    if (!this->getParam(kPassword, password)) {
      ROS_FATAL("Password is required");
      exit(-1);
    }
    const auto result = this->clpe_api.Clpe_Connection(password);
    if (result != 0) {
      ROS_FATAL("Failed to initiate the clpe network connection (%s)",
                ConnectionError::get().message(result).c_str());
      exit(result);
    }

    // publish tf
    for (int i = 0; i < 4; ++i) {
      const auto tf_pub =
          this->advertise<geometry_msgs::Transform>("cam_" + std::to_string(i) + "/tf", 1, true);
      geometry_msgs::Transform tf_msg;
      auto pose = this->GetPoseParam_(i);
      tf_pub.publish(Me::CreateTfMsg_(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]));
    }

    // reading eeprom is slow so the camera info is stored and reused.
    ROS_INFO("Discovering camera properties");
    for (int i = 0; i < 4; ++i) {
      const auto error = this->GetCameraInfo_(i, kCamInfos[i]);
      if (error) {
        ROS_FATAL("Failed to get camera info (%s)", error.message().c_str());
        exit(error.value());
      }
    }
    ROS_INFO("Successfully discovered camera properties");

    // create camera publishers
    kImagePubs.reserve(4);
    for (int i = 0; i < 4; ++i) {
      bool latch = this->param<bool>(kCamLatch[i], false);
      int queue_size = this->param<int>(kCamQueueSize[i], 10);
      kImagePubs.emplace_back(this->transport_->advertise("cam_" + std::to_string(i) + "/image_raw",
                                                          queue_size, latch));
      bool info_latch = this->param<bool>(kCamInfoLatch[i], false);
      bool info_queue_size = this->param<int>(kCamInfoQueueSize[i], 10);
      kInfoPubs[i] = this->advertise<sensor_msgs::CameraInfo>(
          "cam_" + std::to_string(i) + "/camera_info", info_queue_size, info_latch);
    }

    // start publishing
    {
      ROS_INFO("Preparing camera for streaming");
      const auto result = this->clpe_api.Clpe_StartStream(
          [](unsigned int cam_id, unsigned char * buffer, unsigned int size,
             struct timeval * frame_us) -> int {
            ROS_DEBUG("got new image for cam_%i", cam_id);

            // skip all work if there is no subscribers
            if (kImagePubs[cam_id].getNumSubscribers() == 0 &&
                kInfoPubs[cam_id].getNumSubscribers() == 0) {
              ROS_DEBUG("skipped publishing for cam_%i because there are no subscribers", cam_id);
              return 0;
            }

            sensor_msgs::Image image;
            const auto frame_id = kNode->param<std::string>(kCamBaseFrame[cam_id], "base_link");
            Me::FillImageMsg_(buffer, size, *frame_us, frame_id, image);
            kImagePubs[cam_id].publish(image);
            kInfoPubs[cam_id].publish(kCamInfos[cam_id]);
            return 0;
          },
          1, 1, 1, 1, 0);
      if (result != 0) {
        const std::error_code error(result, clpe::StartStreamError::get());
        ROS_FATAL("Failed to start streaming (%s)", error.message().c_str());
        exit(result);
      }
      ROS_INFO("Start streaming images");
    }
  }

private:
  std::unique_ptr<image_transport::ImageTransport> transport_;

  explicit ClpeNode(ClpeClientApi && clpe_api) : ros::NodeHandle("~"), clpe_api(std::move(clpe_api))
  {
  }

  std::vector<double> GetPoseParam_(int cam_id)
  {
    std::vector<double> defaultPose({0, 0, 0, 0, 0, 0});
    std::vector<double> pose = this->param<std::vector<double>>(kCamPose[cam_id], defaultPose);
    if (pose.size() != 6) {
      ROS_FATAL("Failed to get pose parameter, wrong number of elements");
      exit(-1);
    }
    return pose;
  }

  static geometry_msgs::Transform CreateTfMsg_(double x, double y, double z, double roll,
                                               double pitch, double yaw)
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

  /**
   * Reading the camera's eeprom is slow so callers should cache the result
   */
  std::error_code GetCameraInfo_(int cam_id, sensor_msgs::CameraInfo & cam_info)
  {
    // reset to defaults
    cam_info = sensor_msgs::CameraInfo();
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
    cam_info.K = {eeprom_data.fx, 0, eeprom_data.cx, 0, eeprom_data.fy, eeprom_data.cy, 0, 0, 1};
    cam_info.D = {eeprom_data.k1, eeprom_data.k2, eeprom_data.p1,
                  eeprom_data.p2, eeprom_data.k3, eeprom_data.k4};
    return kNoError;
  }

  static void FillImageMsg_(unsigned char * buffer, unsigned int size, const timeval & timestamp,
                            const std::string & frame_id, sensor_msgs::Image & image)
  {
    image.header.frame_id = frame_id;
    image.header.stamp.sec = timestamp.tv_sec;
    image.header.stamp.nsec = timestamp.tv_usec * 1000;
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

  std::error_code GetCameraImage_(int cam_id, sensor_msgs::Image & image)
  {
    unsigned char * buffer;
    unsigned int size;
    timeval timestamp;
    const auto result = this->clpe_api.Clpe_GetFrameOneCam(cam_id, &buffer, &size, &timestamp);
    if (result != 0) {
      return std::error_code(result, GetFrameError::get());
    }
    this->FillImageMsg_(buffer, size, timestamp,
                        this->param<std::string>(kCamBaseFrame[cam_id], "base_link"), image);
    return kNoError;
  }

  friend class ClpeComponentNode;
};
}  // namespace clpe
