#include <ClpeClientApi.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/transform.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "ClpeNode.hpp"

// needed because clpe callback does not support user data :(.
static std::shared_ptr<clpe::ClpeNode<ClpeClientApi>> node;
static std::vector<image_transport::CameraPublisher> camera_pubs;
static std::array<sensor_msgs::msg::CameraInfo, 4> cam_infos;

geometry_msgs::msg::Transform CreateTfMsg(double x, double y, double z, double roll, double pitch,
                                          double yaw)
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
 * Gets and validate the pose param.
 * Exits the program with -1 if the param is invalid.
 */
std::vector<double> GetPoseParam()
{
  auto pose = node->get_parameter("pose").get_value<std::vector<double>>();
  if (pose.size() != 6) {
    RCLCPP_FATAL(node->get_logger(), "Failed to get pose parameter, wrong number of elements");
    exit(-1);
  }
  return pose;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  node = std::make_shared<clpe::ClpeNode<ClpeClientApi>>(ClpeClientApi());
  {
    const auto error = node->Init();
    if (error) {
      RCLCPP_FATAL(node->get_logger(),
                   "Failed to initiate the clpe network connection (" + error.message() + ")");
      exit(error.value());
    }
    RCLCPP_INFO(node->get_logger(), "Successfully initialized");
  }
  image_transport::ImageTransport transport(node);

  // declare ROS params
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
  auto pose = GetPoseParam();
  tf_pub->publish(CreateTfMsg(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]));

  // reading eeprom is slow so the camera info is stored and reused.
  RCLCPP_INFO(node->get_logger(), "Discovering camera properties");
  for (int i = 0; i < 4; ++i) {
    const auto error = node->GetCameraInfo(i, cam_infos[i]);
    if (error) {
      RCLCPP_FATAL(node->get_logger(), "Failed to get camera info (" + error.message() + ")");
      exit(error.value());
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
  {
    RCLCPP_INFO(node->get_logger(), "Preparing camera for streaming");
    const auto result = node->clpe_api.Clpe_StartStream(
        [](unsigned int inst, unsigned char * buffer, unsigned int size,
           struct timeval * frame_us) -> int {
          RCLCPP_DEBUG(node->get_logger(), "got new image for cam_" + std::to_string(inst));
          sensor_msgs::msg::Image image;
          node->FillImageMsg(buffer, size, *frame_us, image);
          camera_pubs[inst].publish(image, cam_infos[inst]);
          return 0;
        },
        1, 1, 1, 1, 0);
    if (result != 0) {
      const std::error_code error(result, clpe::StartStreamError::get());
      RCLCPP_FATAL(node->get_logger(), "Failed to start streaming (" + error.message() + ")");
      exit(result);
    }
    RCLCPP_INFO(node->get_logger(), "Start streaming images");
  }

  // listen for param updates
  const auto onSetParamCbHdl =
      node->add_on_set_parameters_callback([&](const std::vector<rclcpp::Parameter> & params) {
        for (const auto & p : params) {
          if (p.get_name() == "pose") {
            auto pose = GetPoseParam();
            tf_pub->publish(CreateTfMsg(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]));
          }
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });

  rclcpp::spin(node);

  return 0;
}
