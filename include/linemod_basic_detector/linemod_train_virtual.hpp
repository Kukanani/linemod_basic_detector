/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 * Modifications to convert to ROS2 and remove dependencies on Ecto and ORK.
 * Open Robotics, 2017.
 */

#ifndef LINEMOD_BASIC_DETECTOR_LINEMOD_TRAIN_VIRTUAL_HPP_
#define LINEMOD_BASIC_DETECTOR_LINEMOD_TRAIN_VIRTUAL_HPP_

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#if CV_MAJOR_VERSION == 3
  #include <opencv2/rgbd.hpp>
  namespace cv {using namespace cv::rgbd;}
#else
  #include <opencv2/objdetect/objdetect.hpp>
#endif

#include <ork_renderer/utils.h>
#include <ork_renderer/renderer3d.h>
#include <opencv2/highgui/highgui.hpp>

class Trainer
{
public:
  /** Whether or not to output debug image */
  bool visualize_ = true;
  /** The DB parameters as a JSON string */
  std::string json_db_;
  /** The id of the object to generate a trainer for */
  std::string object_id_;
  /** The path to the mesh representing the object */
  std::string mesh_path_;
  cv::linemod::Detector detector_;
  std::vector<cv::Mat>  Rs_;
  std::vector<cv::Mat>  Ts_;
  std::vector<float>  distances_;
  std::vector<cv::Mat>  Ks_;

  int renderer_n_points_= 150;
  int renderer_angle_step_= 10;
  double renderer_radius_min_= 0.6;
  double renderer_radius_max_= 1.1;
  double renderer_radius_step_= 0.4;
  int renderer_width_= 640;
  int renderer_height_= 480;
  double renderer_near_= 0.1;
  double renderer_far_= 1000.0;
  double renderer_focal_length_x_= 525;
  double renderer_focal_length_y_= 525;


  void train(std::string filename, bool use_rgb = true, bool use_depth = true);
  void writeLinemod(const std::string& filename);
private:
  cv::Ptr<cv::linemod::Detector> generateLinemod(bool rgb, bool depth);
};

#endif