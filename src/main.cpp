#include <ClpeClientApi.h>

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

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

    // start clpe stream
    {
      const auto result = this->clpe_api.Clpe_StartStream([](auto...) { return 0; }, 1, 1, 1, 1, 0);
      if (result != 0) {
        RCLCPP_FATAL(this->get_logger(), "Failed to start streaming");
        exit(result);
      }
    }
  }

  sensor_msgs::msg::CameraInfo GetCameraInfo(int cam_id)
  {
    sensor_msgs::msg::CameraInfo cam_info;
    cam_info.width = 1920;
    cam_info.height = 1080;
    // TODO: what's binning?
    // cam_info.binning_x
    // cam_info.binning_y
    // TODO: is this calibration model in eeprom? It only supports "Jhang" and "FishEye" neither
    // of which is supported by ROS.
    // cam_info.distortion_model
    // std::array<uint8_t, 95> eeprom_data;
    // this->clpe_api.Clpe_GetEepromData(i, eeprom_data.data());
    return cam_info;
  }

  sensor_msgs::msg::Image GetCameraImage(int cam_id)
  {
    sensor_msgs::msg::Image image;
    unsigned char * buffer;
    unsigned int size;
    timeval timestamp;
    const auto result = this->clpe_api.Clpe_GetFrameOneCam(cam_id, &buffer, &size, &timestamp);
    if (result != 0) {
      RCLCPP_WARN(this->get_logger(),
                  "Failed to get camera frame ( " + std::to_string(result) + " )");
    }
    image.data.reserve(size);
    memcpy(image.data.data(), buffer, size);
    image.encoding = sensor_msgs::image_encodings::YUV422;
    image.width = 1920;
    image.height = 1080;
    // TODO: what's clpe endianess?
    // image.is_bigendian
    return image;
  }

private:
  rclcpp::TimerBase::SharedPtr image_pub_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ClpeNode<ClpeClientApi>>(ClpeClientApi());
  image_transport::ImageTransport image_transport(node);

  // declare ROS params
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = "The number of images published per second";
    node->declare_parameter("update_rate", rclcpp::ParameterValue(60.0), desc);
  }

  // create camera publishers
  std::vector<image_transport::CameraPublisher> camera_pubs;
  camera_pubs.reserve(4);
  for (int i = 0; i < 4; ++i) {
    camera_pubs[i] = image_transport.advertiseCamera("cam_" + std::to_string(i), 10);
  }

  // functor to start publishing
  rclcpp::TimerBase::SharedPtr pub_timer;
  const auto StartPublish = [&](double rate) {
    pub_timer = node->create_wall_timer(std::chrono::duration<double>(1.0 / rate), [&]() {
      const auto impl = [&](const auto impl) {
        for (int i = 0; i < 4; ++i) {
          camera_pubs[i].publish(node->GetCameraImage(i), node->GetCameraInfo(i));
        }
      };
      impl(impl);
    });
  };

  // listen for param updates
  const auto onSetParamCbHdl =
      node->add_on_set_parameters_callback([&](const std::vector<rclcpp::Parameter> & params) {
        for (const auto & p : params) {
          if (p.get_name() == "update_rate") {
            pub_timer.reset();
            const auto update_rate = p.get_value<double>();
            StartPublish(update_rate);
          }
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });

  // start publishing
  const auto update_rate = node->get_parameter("update_rate").get_value<double>();
  StartPublish(update_rate);

  rclcpp::spin(node);

  return 0;
}
