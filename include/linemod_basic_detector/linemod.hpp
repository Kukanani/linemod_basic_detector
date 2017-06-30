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

#ifndef LINEMOD_BASIC_DETECTOR_LINEMOD_HPP_
#define LINEMOD_BASIC_DETECTOR_LINEMOD_HPP_

#include <opencv2/core/version.hpp>
#if CV_MAJOR_VERSION == 3
  #include <opencv2/core.hpp>
  #include <opencv2/core/utility.hpp>
  #include <opencv2/rgbd.hpp>
  namespace cv {using namespace cv::rgbd;}
#else
  #include <opencv2/core/core.hpp>
  #include <opencv2/objdetect/objdetect.hpp>
#endif

#include <vector>

#include "ork_renderer/utils.h"

struct LinemodDetection {
  cv::Matx33f rotation;
  cv::Vec3f translation;
  int class_id;

  /// similarity score in percent.
  float score;
};

class Linemod {
public:
  Linemod(std::string input_path, bool use_rgb=true, bool use_depth=true);

  std::vector<LinemodDetection> detect(cv::Mat& color_in, cv::Mat& depth_in, cv::Mat& display);

private:
  void drawResponse(const std::vector<cv::linemod::Template>& templates,
                    int num_modalities, cv::Mat& dst, cv::Point offset, int T);

  cv::linemod::Detector readLinemod(const std::string& filename);

  bool inferDepth(cv::Mat_<cv::Vec3f>& depth_real_ref_raw, cv::linemod::Match& match, cv::Vec3f& T_real_icp, cv::Matx33f& R_real_icp);

  cv::linemod::Detector detector_;
  // Various settings and flags
  bool show_match_result = true;
  bool learn_online = false;
  int num_classes = 0;
  int matching_threshold = 1;
  int learning_lower_bound = 90;
  int learning_upper_bound = 95;
  cv::Mat display;

  bool use_rgb_ = true;
  bool use_depth_ = true;

  std::map<std::string, std::vector<cv::Mat> > Ts;
  std::map<std::string, std::vector<cv::Mat> > Rs;
  std::map<std::string, std::vector<cv::Mat> > Ks;
  std::map<std::string, std::vector<float> > distances;
  std::map<std::string, RendererIterator*> renderer_iterators;
};

#endif