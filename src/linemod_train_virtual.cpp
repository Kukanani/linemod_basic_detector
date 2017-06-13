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

#include <exception>
#include "linemod_basic_detector/linemod_train_virtual.hpp"

void Trainer::writeLinemod(const std::string& filename)
{
  cv::FileStorage fs(filename, cv::FileStorage::WRITE);
  detector_.write(fs);

  std::cout << "total # of templates: " << detector_.numTemplates() << std::endl;
  std::cout << "# of stored translations: " << Ts_.size() << std::endl;

  std::vector<cv::String> ids = detector_.classIds();
  fs << "classes" << "[";
  int template_counter = 0;
  for (int i = 0; i < (int)ids.size(); ++i)
  {
    fs << "{";

    fs << "class_id" << detector_.classIds()[i];
    fs << "mesh_path" << mesh_path_;
    fs << "modalities" << "[:";
    for (size_t i = 0; i < detector_.getModalities().size(); ++i)
      fs << detector_.getModalities()[i]->name();
    fs << "]"; // modalities
    fs << "pyramid_levels" << detector_.pyramidLevels();
    fs << "template_pyramids" << "[";
    for (int k = 0; k < detector_.numTemplates(ids[i]); ++k)
    {
      const auto& tp = detector_.getTemplates(ids[i], k);
      fs << "{";
      fs << "template_id" << int(k); //TODO is this cast correct? won't be good if rolls over...

      // detector_.writeClass(ids[i], fs);
      fs << "distance" << distances_[template_counter];
      fs << "T" << Ts_[template_counter];
      fs << "R" << Rs_[template_counter];
      fs << "K" << Ks_[template_counter];
      template_counter++;

      fs << "templates" << "[";
      for (size_t j = 0; j < tp.size(); ++j)
      {
        fs << "{";
        tp[j].write(fs);
        fs << "}"; // current template
      }
      fs << "]"; // templates
      fs << "}"; // current pyramid
    }
    fs << "]"; // pyramids

    fs << "}"; // current class
  }
  fs << "]"; // classes
}

static const int T_DEFAULTS[] = {5, 8};
cv::Ptr<cv::linemod::Detector> Trainer::generateLinemod(bool rgb, bool depth) {
  std::vector< cv::Ptr<cv::linemod::Modality> > modalities;
  assert(rgb | depth);
  if(rgb)
  {
    std::cout << "using rgb as a linemod modality" << std::endl;
    modalities.push_back(cv::makePtr<cv::linemod::ColorGradient>());
  }
  if(depth)
  {
    std::cout << "using depth as a linemod modality" << std::endl;
    modalities.push_back(cv::makePtr<cv::linemod::DepthNormal>());
  }
  return cv::makePtr<cv::linemod::Detector>(modalities, std::vector<int>(T_DEFAULTS, T_DEFAULTS + 2));
}

void Trainer::train(std::string filename, bool use_rgb, bool use_depth)
{

  mesh_path_ = filename;

  cv::Ptr<cv::linemod::Detector> detector_ptr = generateLinemod(use_rgb, use_depth);
  detector_ = *detector_ptr;

  // the model name can be specified on the command line.
  if (mesh_path_.empty())
  {
    std::remove(mesh_path_.c_str());
    std::cerr << "The mesh path is empty for the object id \"" << object_id_<< std::endl;
    return;
  }

  Renderer3d renderer = Renderer3d(mesh_path_);
  renderer.set_parameters(renderer_width_, renderer_height_, renderer_focal_length_x_,
                          renderer_focal_length_y_, renderer_near_, renderer_far_);

  RendererIterator renderer_iterator = RendererIterator(&renderer, renderer_n_points_);
  //set the RendererIterator parameters
  renderer_iterator.angle_step_ = renderer_angle_step_;
  renderer_iterator.radius_min_ = float(renderer_radius_min_);
  renderer_iterator.radius_max_ = float(renderer_radius_max_);
  renderer_iterator.radius_step_ = float(renderer_radius_step_);

  cv::Mat image, depth, mask;
  cv::Matx33d R;
  cv::Vec3d T;
  cv::Matx33f K;
  for (size_t i = 0; !renderer_iterator.isDone(); ++i, ++renderer_iterator)
  {
    std::stringstream status;
    status << "Loading images " << (i+1) << "/"
        << renderer_iterator.n_templates();
    std::cout << status.str();

    cv::Rect rect;
    renderer_iterator.render(image, depth, mask, rect);

    R = renderer_iterator.R_obj();
    T = renderer_iterator.T();
    float distance = renderer_iterator.D_obj() - float(depth.at<ushort>(depth.rows/2.0f, depth.cols/2.0f)/1000.0f);
    K = cv::Matx33f(float(renderer_focal_length_x_), 0.0f, float(rect.width)/2.0f, 0.0f, float(renderer_focal_length_y_), float(rect.height)/2.0f, 0.0f, 0.0f, 1.0f);

    std::vector<cv::Mat> sources(1);
    if(use_rgb)
    {
      sources[0] = image;
      if(use_depth)
      {
        sources.push_back(depth);
      }
    } else
    {
        sources[0] = depth;
    }

    // Display the rendered image
    if (visualize_)
    {
      cv::namedWindow("Rendering");
      if (!image.empty()) {
        cv::imshow("Rendering", image);
        cv::waitKey(1);
      }
    }

    int template_in = detector_.addTemplate(sources, "class1", mask);
    if (template_in == -1)
    {
      // Delete the status
      for (size_t j = 0; j < status.str().size(); ++j)
        std::cout << '\b';
      continue;
    }

    // Also store the pose of each template
    Rs_.push_back(cv::Mat(R));
    Ts_.push_back(cv::Mat(T));
    distances_.push_back(distance);
    Ks_.push_back(cv::Mat(K));

    // Delete the status
    for (size_t j = 0; j < status.str().size(); ++j)
      std::cout << '\b';
  }
  return;
}

int main(int argc, char** argv)
{
  if(argc < 4)
  {
    std::cout << "usage is: linemod_train_virtual <d|r|b> input_file "
              << "output_file [n_points] [angle_step] [radius_min]"
              << "[radius_max] [radius_step] [render_width] [render_height] "
              << "[render_near] [render_far] [focal_x] [focal_y]"
              << std::endl
              << "where:" << std::endl
              << "\td: use depth modality only" << std::endl
              << "\tr: use rgb modality only" << std::endl
              << "\tb: use both modalities" << std::endl
              << "and: " << std::endl
              << "\tn_points: density of render point sampling" << std::endl
              << "\tangle_step: distance between consecutive render points "
                 << "[deg]" << std::endl
              << "\tradius_min: minimum distance to object" << std::endl
              << "\tradius_max: maximum distance to object" << std::endl
              << "\tradius_step: radial distance between render points "
                 << "[m]" << std::endl
              << "\trender_width: width of rendered image [px]" << std::endl
              << "\trender_height: height of rendered image [px]" << std::endl
              << "\trender_near: near clipping plane for render" << std::endl
              << "\trender_far: far clipping plane for render" << std::endl
              << "\tfocal_x: focal length, x [px]" << std::endl
              << "\tfocal_y: focal length, y [px]" << std::endl;
    return 1;
  }
  bool valid = true;

  bool depth = (argv[1][0] == 'd' || argv[1][0] == 'b');
  bool rgb = (argv[1][0] == 'r' || argv[1][0] == 'b');

  valid &= (depth | rgb);


  Trainer t;
  try {
    if(argc > 4) {
      t.renderer_n_points_ = std::stoi(argv[4]);
    } if(argc > 5) {
      t.renderer_angle_step_ = std::stoi(argv[5]);
    } if(argc > 6) {
      t.renderer_radius_min_ = std::stod(argv[6]);
    } if(argc > 7) {
      t.renderer_radius_max_ = std::stod(argv[7]);
    } if(argc > 8) {
      t.renderer_radius_step_ = std::stod(argv[8]);
    } if(argc > 9) {
      t.renderer_width_ = std::stoi(argv[9]);
    } if(argc > 10) {
      t.renderer_height_ = std::stoi(argv[10]);
    } if(argc > 11) {
      t.renderer_near_ = std::stod(argv[11]);
    } if(argc > 12) {
      t.renderer_far_ = std::stod(argv[12]);
    } if(argc > 13) {
      t.renderer_focal_length_x_ = std::stod(argv[13]);
    } if(argc > 14) {
      t.renderer_focal_length_y_ = std::stod(argv[14]);
    }
  }
  catch(std::invalid_argument e) {
    valid = false;
  }
  if(!valid) {
    std::cerr << "error parsing arguments" << std::endl;
    return -1;
  }

  t.train(argv[2], rgb, depth);

  // save the data in a form easily readable by linemod.
  // For now we're using a flat file representation, but we could be using
  // a database, like ORK does. This is just more straightforward
  std::cout << "training complete, saving results to file...." << std::endl;
  t.writeLinemod(argv[3]);
}