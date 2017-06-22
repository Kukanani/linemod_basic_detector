// Copyright 2015 Open Source Robotics Foundation, Inc.
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

// TODO: this file is duplicated in multiple places. See also:
//   ros2/demos/image_pipeline/common.hpp
//   ros2/demos/image_tools/src/showimage.cpp

#ifndef LINEMOD_BASIC_DETECTOR__IMAGE_UTILS_HPP_
#define LINEMOD_BASIC_DETECTOR__IMAGE_UTILS_HPP_

#include <chrono>
#include <string>

#include "opencv2/opencv.hpp"
#include "builtin_interfaces/msg/time.hpp"

// namespace to avoid collision with the intra_process_demo

namespace linemod {

int
encoding2mat_type(const std::string & encoding)
{
  if (encoding == "mono8") {
    return CV_8UC1;
  } else if (encoding == "bgr8" || encoding == "rgb8") {
    return CV_8UC3;
  } else if (encoding == "mono16") {
    return CV_16SC1;
  } else if (encoding == "32FC1" || encoding == "depth32") {
    // Note: this may not be a display-friendly format!
    return CV_32FC1;
  }
  throw std::runtime_error(std::string("Unsupported encoding ").append(encoding));
}


std::string
mat_type2encoding(int mat_type)
{
  switch (mat_type) {
    case CV_8UC1:
      return "mono8";
    case CV_8UC3:
      return "bgr8";
    case CV_16SC1:
      return "mono16";
    case CV_32FC1:
      // Note: this may not be a display-friendly format!
      return "depth32";
    default:
      std::stringstream stream;
      stream << mat_type;
      throw std::runtime_error(std::string("Unsupported mat type ").append(stream.str()).append("'"));
  }
}

}

#endif  // IMAGE_PIPELINE__COMMON_HPP_
