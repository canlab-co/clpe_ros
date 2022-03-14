#include <ClpeClientApi.h>

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "ClpeNode.hpp"

// needed because clpe callback does not support user data :(.
static std::shared_ptr<ClpeNode<ClpeClientApi>> node;
static std::vector<image_transport::CameraPublisher> camera_pubs;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  node = std::make_shared<ClpeNode<ClpeClientApi>>(ClpeClientApi());
  image_transport::ImageTransport transport(node);

  // declare ROS params
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = "Frames per second, must be >=15,<=30";
    desc.integer_range.emplace_back();
    auto & range = desc.integer_range.back();
    range.from_value = 15;
    range.to_value = 30;
    range.step = 1;
    node->declare_parameter("fps", rclcpp::ParameterValue(30), desc);
  }

  // create camera publishers
  camera_pubs.reserve(4);
  for (int i = 0; i < 4; ++i) {
    camera_pubs.emplace_back(
        transport.advertiseCamera("cam_" + std::to_string(i) + "/image_raw", 10));
  }

  // listen for param updates
  const auto onSetParamCbHdl =
      node->add_on_set_parameters_callback([&](const std::vector<rclcpp::Parameter> & params) {
        for (const auto & p : params) {
          if (p.get_name() == "fps") {
            const auto fps = p.get_value<int>();
            // TODO: header is missing Clpe_SetCamFPS in the docs?
          }
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });

  // start publishing
  rclcpp::TimerBase::SharedPtr pub_timer;
  const auto result = node->clpe_api.Clpe_StartStream(
      [](unsigned int inst, unsigned char * buffer, unsigned int size,
         struct timeval * frame_us) -> int {
        const auto image = node->CreateImageMsg(buffer, size);
        const auto cam_info = node->GetCameraInfo(inst);
        // publishing is threadsafe in ROS
        camera_pubs[inst].publish(image, cam_info);
        return 0;
      },
      1, 1, 1, 1, 0);
  if (result != 0) {
    RCLCPP_FATAL(node->get_logger(), "Failed to start streaming");
    exit(result);
  }

  rclcpp::spin(node);

  return 0;
}
