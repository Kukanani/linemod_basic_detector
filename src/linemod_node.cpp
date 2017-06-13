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

#include "linemod_basic_detector/linemod_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if(argc < 3) {
    std::cerr << "usage is: linemod_node templates_file <r|d|b>"
              << std::endl
              << "where:" << std::endl
              << "\td: use depth modality only" << std::endl
              << "\tr: use rgb modality only" << std::endl
              << "\tb: use both modalities" << std::endl;
    return 1;
  }
  bool depth = (argv[2][0] == 'd' || argv[2][0] == 'b');
  bool rgb = (argv[2][0] == 'r' || argv[2][0] == 'b');
  if(!depth && !rgb) {
    std::cerr << "you must specify usage of at least one of depth or rgb." <<
    std::endl;
    return 1;
  }

  std::shared_ptr<LinemodNode> linemod_node = nullptr;
  try
  {
    std::cout << "starting linemod node with template file " << argv[1] << std::endl;
    linemod_node = std::make_shared<LinemodNode>(argv[1], rgb, depth,
                                                 "image", "depth",
                                                 "image_with_detections",
                                                 "detections");
  }
  catch (const std::exception & e)
  {
    std::cerr << e.what() << "," << std::endl << "Exiting..." << std::endl;
    return 1;
  }

  rclcpp::spin(linemod_node);
  return 0;
}
