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

#include <ros/ros.h>

#include <iomanip>
#include <sensor_msgs/Image.h>

//==============================================================================
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "clpe");

  const std::size_t cam_count = 4;

  auto node = ros::NodeHandle("canlab_benchmark_latency");
  std::array<std::size_t, cam_count> cam_sub_count;
  // Latency in millis
  std::array<double, cam_count> avg_latencies;
  for (auto i = 0; i < cam_count; ++i)
  {
    cam_sub_count[i] = 1;
    avg_latencies[i] = 0.0;
  }

  std::array<ros::Subscriber, cam_count> subs;
  using sensor_msgs::Image;
  for (int i = 0; i < cam_count; ++i)
  {
    subs[i] = node.subscribe<Image>("/clpe_node/cam_" + std::to_string(i) + "/image_raw", 10,
                                    [i, &node, &cam_sub_count, &avg_latencies](const Image::ConstPtr& image) {
                                      const auto local_time = ros::Time::now().toNSec();
                                      const auto cam_time = image->header.stamp.toNSec();
                                      const auto diff_ns = local_time - cam_time;
                                      const double latency = diff_ns / 1000000.0;
                                      avg_latencies[i] =
                                          (avg_latencies[i] * cam_sub_count[i] + latency) / (cam_sub_count[i] + 1);
                                      cam_sub_count[i]++;
                                      std::cout << "cam_" << i << " latency = " << std::fixed << std::setprecision(3)
                                                << avg_latencies[i] << "ms" << std::endl;
                                    });
  }

  ros::spin();

  return 0;
}
