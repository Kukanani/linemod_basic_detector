// Copyright 2017 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LINEMOD_BASIC_DETECTOR_LINEMOD_NODE_HPP_
#define LINEMOD_BASIC_DETECTOR_LINEMOD_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <atomic>
#include <cmath>

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "rcl/rcl.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "sensor_msgs/msg/image.hpp"

#include "vision_msgs/msg/detection3_d.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"

#include "linemod_basic_detector/linemod.hpp"
#include "linemod_basic_detector/image_utils.hpp"

class LinemodNode : public rclcpp::Node
{
public:
  LinemodNode(
    const std::string & templates_path, bool use_rgb, bool use_depth,
    const std::string & input, const std::string & depth_input,
    const std::string & output_image,
    const std::string & output_detections,
    const std::string & node_name = "linemod_node")
  : Node(node_name, "", true),
    linemod(templates_path, use_rgb, use_depth)
  {
    std::cout << "loaded Linemod templates from " << templates_path << std::endl;
    cv::namedWindow("detection");
    // cv::namedWindow("depth");
    auto qos = rmw_qos_profile_sensor_data;
    // Create a publisher on the input topic.
    pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_image, qos);
    detections_pub_ =
        this->create_publisher<vision_msgs::msg::Detection3DArray>(
          output_detections,
          qos);
    std::weak_ptr<std::remove_pointer<decltype(pub_.get())>::type>
        captured_pub = pub_;
    // Create a subscription on the output topic.
    std::cout << "creating subscribers..." << std::endl;
    sub_color_ = this->create_subscription<sensor_msgs::msg::Image>(
        input, [&](sensor_msgs::msg::Image::SharedPtr msg)
    {
      color_image_ = msg;

      // Make a Mat so we can convert color space for this image, but do not
      // store the Mat to avoid memory corruption
      cv::Mat color(
        color_image_->height, color_image_->width,
        encoding2mat_type(color_image_->encoding),
        color_image_->data.data());
      if(!color.empty()) {
        cv::cvtColor(color, color, CV_BGR2RGB);
      }
    }, qos);

    sub_depth_ = this->create_subscription<sensor_msgs::msg::Image>(
        depth_input, [&](sensor_msgs::msg::Image::SharedPtr msg)
    {
      std::cout << "got depth image" << std::endl;
      // Create a cv::Mat from the image message (without copying).
      cv::Mat cv_mat(
        msg->height, msg->width,
        encoding2mat_type(msg->encoding),
        msg->data.data());

      frame_depth_ = cv_mat;

      if (color_image_ != NULL)
      {
        // Create a Mat once depth and color are both acquired.
        cv::Mat color(
          color_image_->height, color_image_->width,
          encoding2mat_type(color_image_->encoding),
          color_image_->data.data());

        cv::Mat display;

        if (frame_depth_.depth() == CV_32F)
        {
          float scale = 1000;
          // float scale = 1;
          frame_depth_.convertTo(frame_depth_, CV_16UC1, scale);
        }

        if(frame_depth_.data != NULL)
        {
          auto detections = linemod.detect(color, frame_depth_, display);

          // convert the list of LinemodDetections to something that can be
          // published - namely, a Detection3DArray from vision_msgs.

          if(detections.size() > 0)
          {
            vision_msgs::msg::Detection3DArray detections_msg;
            // TODO make this and other frame ids match whatever comes on the
            // input cloud.
            detections_msg.header.frame_id = "openni_depth_optical_frame";
            detections_msg.header.stamp = rclcpp::Time::now();

            for(auto it = detections.begin(); it != detections.end(); ++it)
            {
              bool any_nans =
                std::isnan(it->translation[0]) |
                std::isnan(it->translation[1]) |
                std::isnan(it->translation[2]) |
                std::isnan(it->class_id);
              for(size_t i=0; i<3; ++i) {
                for (size_t j=0; j<3; ++j) {
                  any_nans |= std::isnan(it->rotation(i,j));
                }
              }
              if(!any_nans)
              {
                vision_msgs::msg::Detection3D detection;
                detection.header.frame_id = "openni_depth_optical_frame";
                detection.header.stamp = rclcpp::Time::now();

                detection.source_cloud.header.frame_id = "";
                detection.source_cloud.width=0;
                detection.source_cloud.height=0;
                detection.source_cloud.row_step=0;

                detection.bbox.center.position.x = 0;
                detection.bbox.center.position.y = 0;
                detection.bbox.center.position.z = 0;
                // detection.bbox.center.orientation.w = 1;

                vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
                // scores from the linemod detector are supplied as percentage
                // values.
                hypothesis.score = it->score/100.f;
                hypothesis.id = it->class_id;
                hypothesis.pose.position.x = it->translation[0];
                hypothesis.pose.position.y = it->translation[1];
                hypothesis.pose.position.z = it->translation[2];

                // use this instead of the following quaternion section for a
                // default object orientation.
                // hypothesis.pose.orientation.w = 1.0;

                // std::cout << "incoming rotation matrix:" << std::endl <<
                //   it->rotation(0,0) << ", " <<
                //   it->rotation(0,1) << ", " <<
                //   it->rotation(0,2) << ", " << std::endl <<
                //   it->rotation(1,0) << ", " <<
                //   it->rotation(1,1) << ", " <<
                //   it->rotation(1,2) << ", " << std::endl <<
                //   it->rotation(2,0) << ", " <<
                //   it->rotation(2,1) << ", " <<
                //   it->rotation(2,2) << ", " << std::endl;

                tf2::Matrix3x3 mat33(
                  it->rotation(0,0),
                  it->rotation(0,1),
                  it->rotation(0,2),
                  it->rotation(1,0),
                  it->rotation(1,1),
                  it->rotation(1,2),
                  it->rotation(2,0),
                  it->rotation(2,1),
                  it->rotation(2,2)
                );

                tf2::Quaternion quat;
                mat33.getRotation(quat);

                geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat);
                hypothesis.pose.orientation = quat_msg;

                detection.results.push_back(hypothesis);

                detections_msg.detections.push_back(detection);
                // std::cout << "added detection to detection list" << std::endl;
              }
            }
            std::cout << "publishing detections" << std::endl;
            detections_pub_->publish(detections_msg);
          }
          std::cout << "------------------------------------------------------"
                    << std::endl;
          cv::imshow("detection", display);
          // cv::imshow("depth", frame_depth_);
          cv::waitKey(1);
        }
      }
    }, qos);
    std::cout << "linemod node set up" << std::endl;
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr
      detections_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_color_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_;
  cv::VideoCapture cap_;
  Linemod linemod;
  sensor_msgs::msg::Image::SharedPtr color_image_;
  cv::Mat frame_depth_;
};

#endif
