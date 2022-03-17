#include <ClpeClientApi.h>

#include <geometry_msgs/msg/transform.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "ClpeNode.hpp"

// needed because clpe callback does not support user data :(.
static std::shared_ptr<ClpeNode<ClpeClientApi>> node;
static std::vector<image_transport::CameraPublisher> camera_pubs;
static std::array<sensor_msgs::msg::CameraInfo, 4> cam_infos;

/**
 * Publishes camera images by polling with a fixed interval. There is no synchronization
 * so an image may be published multiple times.
 */
rclcpp::TimerBase::SharedPtr PollPublish(int fps)
{
  return node->create_wall_timer(std::chrono::duration<double>(1.0 / fps), []() {
    RCLCPP_DEBUG(node->get_logger(), "publishing images");
    for (int i = 0; i < 4; ++i) {
      sensor_msgs::msg::Image image;
      int result = node->GetCameraImage(i, image);
      if (result == -2) {
        // no new frame
        RCLCPP_WARN(node->get_logger(),
                    "cam_" + std::to_string(i) + " missed frame update (no new frame available)");
        continue;
      } else if (result != 0) {
        RCLCPP_FATAL(node->get_logger(), "Error getting frame (" + std::to_string(result) + ")");
        exit(result);
      }
      camera_pubs[i].publish(image, cam_infos[i]);
    }
  });
}

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
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description =
        "Pose relative to the base, 6 values corresponding to [x, y, z, roll, pitch, yaw]";
    node->declare_parameter("pose",
                            rclcpp::ParameterValue(std::vector<double>({0, 0, 0, 0, 0, 0})));
  }

  // publish tf
  // use transient local with history depth 1 since the tf will never change.
  rclcpp::QoS tf_qos(1);
  tf_qos.reliable();
  tf_qos.transient_local();
  const auto tf_pub = node->create_publisher<geometry_msgs::msg::Transform>("tf", tf_qos);
  geometry_msgs::msg::Transform tf_msg;
  const auto & pose = node->get_parameter("pose").get_value<std::vector<double>>();
  tf_msg.translation.x = pose[0];
  tf_msg.translation.y = pose[1];
  tf_msg.translation.z = pose[2];
  tf2::Quaternion quat;
  quat.setRPY(pose[3], pose[4], pose[5]);
  tf_msg.rotation.x = quat.x();
  tf_msg.rotation.y = quat.y();
  tf_msg.rotation.z = quat.z();
  tf_msg.rotation.w = quat.w();
  tf_pub->publish(tf_msg);

  // reading eeprom is slow so the camera info is stored and reused.
  RCLCPP_INFO(node->get_logger(), "Discovering camera properties");
  for (int i = 0; i < 4; ++i) {
    const auto result = node->GetCameraInfo(i, cam_infos[i]);
    if (result != 0) {
      RCLCPP_FATAL(node->get_logger(), "Failed to get camera info");
      exit(result);
    }
  }
  RCLCPP_INFO(node->get_logger(), "Successfully discovered camera properties");

  // create camera publishers
  camera_pubs.reserve(4);
  for (int i = 0; i < 4; ++i) {
    camera_pubs.emplace_back(
        transport.advertiseCamera("cam_" + std::to_string(i) + "/image_raw", 10));
  }

  // start publishing
  RCLCPP_INFO(node->get_logger(), "Preparing camera for streaming");
  const auto result = node->clpe_api.Clpe_StartStream(
      [](unsigned int inst, unsigned char * buffer, unsigned int size,
         struct timeval * frame_us) -> int {
        // FIXME: Stream api is not working
        // RCLCPP_DEBUG(node->get_logger(), "got new image for cam_" + std::to_string(inst));
        // sensor_msgs::msg::Image image;
        // node->FillImageMsg(buffer, size, *frame_us, image);
        // sensor_msgs::msg::CameraInfo cam_info;
        // // publishing is threadsafe in ROS
        // camera_pubs[inst].publish(image, cam_infos[inst]);
        return 0;
      },
      1, 1, 1, 1, 0);
  if (result != 0) {
    RCLCPP_FATAL(node->get_logger(), "Failed to start streaming (" + std::to_string(result) + ")");
    exit(result);
  }
  // FIXME: use polling api since stream api is not working
  rclcpp::TimerBase::SharedPtr pub_timer;
  auto fps = node->get_parameter("fps").get_value<int>();
  pub_timer = PollPublish(fps);
  RCLCPP_INFO(node->get_logger(), "Started publishing camera images");

  // listen for param updates
  const auto onSetParamCbHdl =
      node->add_on_set_parameters_callback([&](const std::vector<rclcpp::Parameter> & params) {
        for (const auto & p : params) {
          if (p.get_name() == "fps") {
            const auto fps = p.get_value<int>();
            // TODO: header is missing Clpe_SetCamFPS in the docs?
            pub_timer.reset();
            pub_timer = PollPublish(fps);
          }
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });

  rclcpp::spin(node);

  return 0;
}
