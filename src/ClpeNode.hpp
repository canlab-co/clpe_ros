#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

// TODO: docs say 95 bytes, but reference sheet is 107 bytes
struct __attribute__((packed)) EepromData {
  uint16_t signature_code;
  uint64_t version;
  uint32_t calibration_model;
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

  ClpeNode(ClpeClientApi && clpe_api) : rclcpp::Node("clpe"), clpe_api(std::move(clpe_api))
  {
    // Initialize ClpeClient
    {
      // FIXME: This requires sudo password!!
      const auto result = this->clpe_api.Clpe_Connection("");
      if (result != 0) {
        RCLCPP_FATAL(this->get_logger(),
                     "Failed to initiate the clpe network connection. Error number = ( ", result,
                     " )");
        exit(result);
      } else {
        RCLCPP_INFO(this->get_logger(), "Successfully initialized");
      }
    }
  }

  sensor_msgs::msg::CameraInfo GetCameraInfo(int cam_id)
  {
    // calibration may change anytime for self calibrating systems, so we cannot cache the cam info.
    sensor_msgs::msg::CameraInfo cam_info;
    cam_info.width = 1920;
    cam_info.height = 1080;
    EepromData eeprom_data;
    const auto result =
        this->clpe_api.Clpe_GetEepromData(cam_id, reinterpret_cast<unsigned char *>(&eeprom_data));
    if (result != 0) {
      RCLCPP_FATAL(this->get_logger(),
                   "Failed to get eeprom data ( " + std::to_string(result) + " )");
      exit(result);
    }
    cam_info.k = {eeprom_data.fx, 0, eeprom_data.cx, 0, eeprom_data.fy, eeprom_data.cy, 0, 0, 1};
    // TODO: is this calibration model in eeprom? It only supports "Jhang" and "FishEye" neither
    // of which is supported by ROS.
    // cam_info.distortion_model
    return cam_info;
  }

  static sensor_msgs::msg::Image CreateImageMsg(unsigned char * buffer, unsigned int size)
  {
    sensor_msgs::msg::Image image;
    // TODO: confirm that the buffer is valid for duration of the publish
    image.data = std::vector<unsigned char>(buffer, buffer + size);
    image.encoding = sensor_msgs::image_encodings::YUV422;
    image.width = 1920;
    image.height = 1080;
    // assume that each row is same sized.
    image.step = size / 1080;
    return image;
  }

  sensor_msgs::msg::Image GetCameraImage(int cam_id)
  {
    unsigned char * buffer;
    unsigned int size;
    timeval timestamp;
    const auto result = this->clpe_api.Clpe_GetFrameOneCam(cam_id, &buffer, &size, &timestamp);
    if (result != 0) {
      RCLCPP_WARN(this->get_logger(),
                  "Failed to get camera frame ( " + std::to_string(result) + " )");
    }
    return this->CreateImageMsg(buffer, size);
  }
};
