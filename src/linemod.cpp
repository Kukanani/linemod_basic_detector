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

#include "linemod_basic_detector/linemod.hpp"

#include <iterator>
#include <cstdio>
#include <iostream>
#include <set>

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include "linemod_basic_detector/linemod_icp.hpp"
#include "linemod_basic_detector/linemod_train_virtual.hpp"

typedef std::vector<cv::linemod::Template> TemplatePyramid;
typedef std::map<cv::String, std::vector<TemplatePyramid> > TemplatesMap;

// Functions to store detector and templates in single XML/YAML file
cv::linemod::Detector Linemod::readLinemod(const std::string& filename)
{
  std::cout << "reading linemod, filename: " << filename << std::endl;
  cv::linemod::Detector detector;
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  detector.read(fs.root());

  cv::FileNode fn = fs["classes"];
  for (cv::FileNodeIterator i = fn.begin(), iend = fn.end(); i != iend; ++i)
  {
    detector.readClass(*i);
    cv::String class_id = (*i)["class_id"];

    cv::FileNode tps_fn = (*i)["template_pyramids"];
    cv::FileNodeIterator tps_it = tps_fn.begin(), tps_it_end = tps_fn.end();

    // std::map<std::string, std::vector<cv::Mat> >::value_type T_val(class_id, cv::Mat());
    // cv::Mat& T_ref = T_val.second;

    // std::map<std::string, std::vector<cv::Mat> >::value_type R_val(class_id, cv::Mat());
    // cv::Mat& R_ref = R_val.second;

    // std::map<std::string, std::vector<float> >::value_type distance_val(class_id, float);
    // float& distance_ref = distance_val.second;
    int expected_id = 0;

    // cv::FileNode tps_fn = fn["template_pyramids"];
    for ( ; tps_it != tps_it_end; ++tps_it, ++expected_id)
    {
      // int template_id = (*tps_it)["template_id"];

      cv::Mat T_new, R_new, K_new;

      cv::FileNodeIterator fni = (*tps_it)["T"].begin();
      fni >> T_new;
      Ts[class_id].push_back(T_new);

      fni = (*tps_it)["R"].begin();
      fni >> R_new;
      Rs[class_id].push_back(R_new);

      fni = (*tps_it)["K"].begin();
      fni >> K_new;
      Ks[class_id].push_back(K_new);

      distances[class_id].push_back((*tps_it)["distance"]);
    }
    std::cout << "loaded " << Rs[class_id].size() << " templates for class ID "
              << class_id << std::endl;

    Renderer3d *renderer_ = new Renderer3d((*i)["mesh_path"]);
    Trainer dummy;
    renderer_->set_parameters(
      dummy.renderer_width_,
      dummy.renderer_height_,
      dummy.renderer_focal_length_x_,
      dummy.renderer_focal_length_y_,
      dummy.renderer_near_,
      dummy.renderer_far_);

    RendererIterator *renderer_iterator_ = new RendererIterator(renderer_,
        dummy.renderer_n_points_);
    renderer_iterator_->angle_step_ = dummy.renderer_angle_step_;
    renderer_iterator_->radius_min_ = float(dummy.renderer_radius_min_);
    renderer_iterator_->radius_max_ = float(dummy.renderer_radius_max_);
    renderer_iterator_->radius_step_ = float(dummy.renderer_radius_step_);

    renderer_iterators.insert(
        std::pair<std::string,RendererIterator*>(class_id, renderer_iterator_));
  }
  return detector;
}

Linemod::Linemod(std::string input_path, bool use_rgb, bool use_depth) :
  use_rgb_(use_rgb),
  use_depth_(use_depth)
{
  if(!use_rgb && !use_depth) {
    std::cerr << "you must specify either use_rgb or use_depth as true, without"
              << "either, serious errors will occur in linemod." << std::endl;
  }
  // not initializer list because it also fills the T, R and distance vectors.
  detector_ = readLinemod(input_path);
  // cv::namedWindow("normals");

  std::vector<cv::String> ids = detector_.classIds();
  num_classes = detector_.numClasses();
  std::cout << "Loaded linemod templates with " << num_classes <<
      " classes, " << detector_.numTemplates() << " templates, and " <<
      detector_.getModalities().size() << " modalities." << std::endl;
  if (!ids.empty())
  {
    printf("Class ids:\n");
    std::copy(ids.begin(), ids.end(),
              std::ostream_iterator<std::string>(std::cout, "\n"));
  }
}


std::vector<LinemodDetection> Linemod::detect(cv::Mat& color_in,
                                              cv::Mat& depth_in,
                                              cv::Mat& display)
{
  int num_modalities = detector_.getModalities().size();
  std::cout << "performing linemod with " << num_modalities
            << " modalities..." << std::endl;

  std::vector<cv::Mat> sources;
  if(use_rgb_)
  {
    sources.push_back(color_in);
  }
  if(use_depth_)
  {
    sources.push_back(depth_in);
  }
  display = color_in.clone();
  cv::Mat depth_proc = depth_in.clone();

  // Perform matching
  std::vector<cv::linemod::Match> matches;
  std::vector<cv::String> class_ids;
  std::vector<cv::Mat> quantized_images;
  float matching_threshold = 87;
  detector_.match(sources, (float)matching_threshold, matches, class_ids,
                  quantized_images);

  int classes_visited = 0;
  std::set<std::string> visited;

  cv::Mat_<cv::Vec3f> depth_m_input;
  // TODO: this is hard-coded from the Astra camera parameters.
  float Kvals[9] = {570.3422241210938, 0.0, 314.5, 0.0,
                    570.3422241210938, 235.5, 0.0, 0.0, 1.0};
  cv::Mat K = cv::Mat(3,3,CV_32F, Kvals);
  // std::cout << "depth_in :"<< std::endl << depth_in << std::endl;
  cv::rgbd::depthTo3d(depth_in, K, depth_m_input);
  // std::cout << "depth_in :"<< std::endl << depth_m_input << std::endl;

  // double min;
  // double max;
  // cv::minMaxIdx(depth_m_input, &min, &max);
  // cv::Mat adjMap;
  // cv::convertScaleAbs(depth_m_input, adjMap, 255 / max);
  // cv::imshow("depth_m_input", adjMap);

  auto lds = std::vector<LinemodDetection>();
  // for(auto m : matches)
  // {
  for (int i = 0; (i < (int)matches.size()) && (classes_visited < num_classes); ++i)
  {
    cv::linemod::Match m = matches[i];
    cv::Vec3f translation;
    cv::Matx33f rotation;
    if(std::find(visited.begin(), visited.end(), m.class_id) == visited.end())
    {
      if(!inferDepth(depth_m_input, m, translation, rotation))
        continue;
      visited.insert(m.class_id);
      ++classes_visited;

      // Draw matching template
      const std::vector<cv::linemod::Template>& templates =
          detector_.getTemplates(m.class_id, m.template_id);

      drawResponse(templates, num_modalities, display, cv::Point(m.x, m.y),
                   detector_.getT(0));

      if (show_match_result)
      {
        std::cout << "Similarity: " << m.similarity << ", x: " << m.x << ", y: "
                  << m.y << ", class: " << m.class_id.c_str() << ", template: "
                  << m.template_id << std::endl;

        float distance = distances.at(m.class_id).at(m.template_id);
        std::cout << "distance " << distance << std::endl;

      }
      LinemodDetection ld;

      ld.score = m.similarity;
      ld.class_id = atoi(m.class_id.substr(5).c_str());
      ld.translation = translation;
      ld.rotation = rotation;

      lds.push_back(ld);
    }
  }

  if (show_match_result && matches.empty())
    printf("No matches found...\n");


  // cv::imshow("normals", quantized_images[1]);
  cv::waitKey(1);

  std::cout << "linemod detector detection complete, found " << lds.size()
            << "matches" << std::endl;
  return lds;
}

bool Linemod::inferDepth(cv::Mat_<cv::Vec3f>& depth_m_input,
                         cv::linemod::Match& match,
                         cv::Vec3f& T_real_icp,
                         cv::Matx33f& R_real_icp)
{
  cv::Matx33d match_rotation =
      Rs.at(match.class_id)[match.template_id].clone();
  cv::Vec3d match_translation =
      Ts.at(match.class_id)[match.template_id].clone();
  float match_depth_dist = distances.at(match.class_id)[match.template_id];
  cv::Mat K_match = Ks.at(match.class_id)[match.template_id];

  // render the depth of the reference object
  cv::Mat mask;
  cv::Rect rect;
  cv::Matx33d R_temp(match_rotation.inv());
  cv::Vec3d up(-R_temp(0,1), -R_temp(1,1), -R_temp(2,1));
  RendererIterator* it_r = renderer_iterators.at(match.class_id);
  cv::Mat depth_ref_;
  it_r->renderDepthOnly(depth_ref_, mask, rect, -match_translation, up);

  cv::Mat_<cv::Vec3f> depth_m_rendered;
  cv::depthTo3d(depth_ref_, K_match, depth_m_rendered);
  //prepare the bounding box for the model and reference point clouds
  cv::Rect_<int> rect_model(0, 0, depth_m_rendered.cols, depth_m_rendered.rows);
  //prepare the bounding box for the reference point cloud: add the offset
  cv::Rect_<int> rect_ref(rect_model);
  rect_ref.x += match.x;
  rect_ref.y += match.y;

  rect_ref = rect_ref & cv::Rect(0, 0, depth_m_input.cols, depth_m_input.rows);
  if ((rect_ref.width < 5) || (rect_ref.height < 5))
  {
    return false;
    // std::cerr << "error: rect_ref is too small: " << rect_ref.width << "x"
    //           << rect_ref.height << std::endl;
  }
  //adjust both rectangles to be equal to the smallest among them
  if (rect_ref.width > rect_model.width)
    rect_ref.width = rect_model.width;
  if (rect_ref.height > rect_model.height)
    rect_ref.height = rect_model.height;
  if (rect_model.width > rect_ref.width)
    rect_model.width = rect_ref.width;
  if (rect_model.height > rect_ref.height)
    rect_model.height = rect_ref.height;

  //crop the input data and the render to the object size
  cv::Mat_<cv::Vec3f> depth_m_input_cropped = depth_m_input(rect_ref);
  // cv::imshow("cropped depth object from camera", depth_m_input_cropped);
  cv::Mat_<cv::Vec3f> depth_m_rendered_cropped = depth_m_rendered(rect_model);

  // std::cout << "depth_m_input_cropped" << depth_m_input_cropped << std::endl;
  //initialize the translation based on the points in the observed point cloud
  cv::Vec3f T_crop = depth_m_input_cropped(depth_m_input_cropped.rows / 2.0f,
                                           depth_m_input_cropped.cols / 2.0f);
  //add the object's depth
  // std::cout << "initial T_crop : " << T_crop << std::endl;
  T_crop(2)  += match_depth_dist;
  // std::cout << "match_depth_dist = " << match_depth_dist << std::endl;
  // std::cout << "after depth adj: " << T_crop << std::endl;

  if (!cv::checkRange(T_crop))
  {
    return false;
    // std::cerr << "error: T_crop out of range" << std::endl;
  }
  T_real_icp = cv::Vec3f(T_crop);

  //initialize the rotation based on model data
  if (!cv::checkRange(match_rotation))
  {
    return false;
    // std::cerr << "error: match_rotation out of range" << std::endl;
  }
  R_real_icp = cv::Matx33f(match_rotation);

  //get the point clouds (for both reference and model)
  std::vector<cv::Vec3f> pts_real_model_temp;
  std::vector<cv::Vec3f> pts_real_ref_temp;
  float px_ratio_missing = matToVec(depth_m_input_cropped,
                                    depth_m_rendered_cropped,
                                    pts_real_ref_temp,
                                    pts_real_model_temp);
  float px_match_min_ = 0.25;
  if (px_ratio_missing > (1.0f-px_match_min_))
  {
    return false;
    // std::cerr << "ratio missing too high!" << std::endl;
  }

  //perform the first approximate ICP
  float px_ratio_match_inliers = 0.0f;
  float icp_dist = icpCloudToCloud(pts_real_ref_temp,
                                   pts_real_model_temp, R_real_icp, T_real_icp,
                                   px_ratio_match_inliers, 1);
  //reject the match if the icp distance is too big

  float icp_dist_min_ = 0.05;
  if (icp_dist > icp_dist_min_)
  {
    return false;
    // std::cerr << "icp dist too high!" << std::endl;
  }

  //perform a finer ICP
  icp_dist = icpCloudToCloud(pts_real_ref_temp, pts_real_model_temp, R_real_icp,
                             T_real_icp, px_ratio_match_inliers, 2);

  // std::cout << "final R_real_icp:" << R_real_icp << std::endl;
  // std::cout << "final T_real_icp:" << T_real_icp << std::endl;
  return true;
}

void Linemod::drawResponse(const std::vector<cv::linemod::Template>& templates,
                  int num_modalities, cv::Mat& dst, cv::Point offset, int T)
{
  static const cv::Scalar COLORS[5] = { CV_RGB(0, 0, 255),
                                        CV_RGB(0, 255, 0),
                                        CV_RGB(255, 255, 0),
                                        CV_RGB(255, 140, 0),
                                        CV_RGB(255, 0, 0) };

  for (int m = 0; m < num_modalities; ++m)
  {
    // NOTE: Original demo recalculated max response for each feature in the TxT
    // box around it and chose the display color based on that response. Here
    // the display color just depends on the modality.
    cv::Scalar color = COLORS[m];

    for (int i = 0; i < (int)templates[m].features.size(); ++i)
    {
      cv::linemod::Feature f = templates[m].features[i];
      cv::Point pt(f.x + offset.x, f.y + offset.y);
      cv::circle(dst, pt, T / 2, color);
    }
  }
}
