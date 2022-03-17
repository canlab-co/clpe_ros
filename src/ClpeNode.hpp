#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

enum class ClpeCalibrationModel : uint32_t {
  Jhang = 0,
  FishEye = 1,
};

// TODO: docs say 95 bytes, but reference sheet is 107 bytes
struct __attribute__((packed)) EepromData {
  uint16_t signature_code;
  uint64_t version;
  ClpeCalibrationModel calibration_model;
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

template <typename ClpeClientApi>
class ClpeNode : public rclcpp::Node
{
public:
  ClpeClientApi clpe_api;

  ClpeNode(ClpeClientApi && clpe_api)
      : rclcpp::Node("clpe"), clpe_api(std::move(clpe_api))
  {
    // declare ros params
    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.description = "sudo password";
      desc.read_only = true;
      this->declare_parameter("password", rclcpp::ParameterValue(), desc);
    }

    // Initialize ClpeClient
    {
      // FIXME: This requires sudo password!!
      const auto& password = this->get_parameter("password").get_value<std::string>();
      const auto result = this->clpe_api.Clpe_Connection(password);
      if (result != 0) {
        RCLCPP_FATAL(this->get_logger(),
                     "Failed to initiate the clpe network connection. Error number = (" +
                         std::to_string(result) + ")");
        exit(result);
      } else {
        RCLCPP_INFO(this->get_logger(), "Successfully initialized");
      }
    }
  }

  /**
   * Reading the camera's eeprom is slow so callers should cache the result
   */
  int GetCameraInfo(int cam_id, sensor_msgs::msg::CameraInfo & cam_info)
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
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to get eeprom data (" + std::to_string(result) + ")");
      return result;
    }
    switch (eeprom_data.calibration_model) {
      case ClpeCalibrationModel::Jhang:
        cam_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
        break;
      case ClpeCalibrationModel::FishEye:
        cam_info.distortion_model = sensor_msgs::distortion_models::EQUIDISTANT;
        break;
    }
    cam_info.k = {eeprom_data.fx, 0, eeprom_data.cx, 0, eeprom_data.fy, eeprom_data.cy, 0, 0, 1};
    cam_info.d = {eeprom_data.k1, eeprom_data.k2, eeprom_data.p1,
                  eeprom_data.p2, eeprom_data.k3, eeprom_data.k4};
    return result;
  }

  static void FillImageMsg(unsigned char * buffer, unsigned int size, const timeval & timestamp,
                           sensor_msgs::msg::Image & image)
  {
    image.header.frame_id = "base_link";
    image.header.stamp.sec = timestamp.tv_sec;
    image.header.stamp.nanosec = timestamp.tv_usec * 1000;
    // TODO: confirm that the buffer is valid for duration of the publish
    image.data = std::vector<unsigned char>(buffer, buffer + size);
    image.encoding = sensor_msgs::image_encodings::YUV422;
    image.width = 1920;
    image.height = 1080;
    // assume that each row is same sized.
    image.step = size / 1080;
    image.is_bigendian = false;
  }

  int GetCameraImage(int cam_id, sensor_msgs::msg::Image & image)
  {
    unsigned char * buffer;
    unsigned int size;
    timeval timestamp;
    const auto result = this->clpe_api.Clpe_GetFrameOneCam(cam_id, &buffer, &size, &timestamp);
    if (result != 0) {
      return result;
    }
    this->FillImageMsg(buffer, size, timestamp, image);
    return result;
  }
};
