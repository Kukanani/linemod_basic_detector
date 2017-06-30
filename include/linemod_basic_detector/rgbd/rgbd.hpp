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

#ifndef __OPENCV_RGBD_HPP__
#define __OPENCV_RGBD_HPP__

#ifdef __cplusplus

#include <limits>

#include <opencv2/core/core.hpp>
#include <opencv2/core/types_c.h>

namespace cv
{
  /** Checks if the value is a valid depth. For CV_16U or CV_16S, the convention is to be invalid if it is
   * a limit. For a float/double, we just check if it is a NaN
   * @param depth the depth to check for validity
   */
  CV_EXPORTS
  inline bool
  isValidDepth(const float & depth)
  {
    return !cvIsNaN(depth);
  }
  CV_EXPORTS
  inline bool
  isValidDepth(const double & depth)
  {
    return !cvIsNaN(depth);
  }
  CV_EXPORTS
  inline bool
  isValidDepth(const short int & depth)
  {
    return (depth != std::numeric_limits<short int>::min()) && (depth != std::numeric_limits<short int>::max());
  }
  CV_EXPORTS
  inline bool
  isValidDepth(const unsigned short int & depth)
  {
    return (depth != std::numeric_limits<unsigned short int>::min())
        && (depth != std::numeric_limits<unsigned short int>::max());
  }
  CV_EXPORTS
  inline bool
  isValidDepth(const int & depth)
  {
    return (depth != std::numeric_limits<int>::min()) && (depth != std::numeric_limits<int>::max());
  }
  CV_EXPORTS
  inline bool
  isValidDepth(const unsigned int & depth)
  {
    return (depth != std::numeric_limits<unsigned int>::min()) && (depth != std::numeric_limits<unsigned int>::max());
  }

  /**
   * @param depth the depth image
   * @param K
   * @param in_points the list of xy coordinates
   * @param points3d the resulting 3d points
   */
  CV_EXPORTS
  void
  depthTo3dSparse(InputArray depth, InputArray in_K, InputArray in_points, OutputArray points3d);

  /** Converts a depth image to an organized set of 3d points.
   * The coordinate system is x pointing left, y down and z away from the camera
   * @param depth the depth image (if given as short int CV_U, it is assumed to be the depth in millimeters
   *              (as done with the Microsoft Kinect), otherwise, if given as CV_32F or CV_64F, it is assumed in meters)
   * @param K The calibration matrix
   * @param points3d the resulting 3d points. They are of depth the same as `depth` if it is CV_32F or CV_64F, and the
   *        depth of `K` if `depth` is of depth CV_U
   * @param mask the mask of the points to consider (can be empty)
   */
  CV_EXPORTS
  void
  depthTo3d(InputArray depth, InputArray K, OutputArray points3d, InputArray mask = noArray());

  /** If the input image is of type CV_16UC1 (like the Kinect one), the image is converted to floats, divided
   * by 1000 to get a depth in meters, and the values 0 are converted to std::numeric_limits<float>::quiet_NaN()
   * Otherwise, the image is simply converted to floats
   * @param in the depth image (if given as short int CV_U, it is assumed to be the depth in millimeters
   *              (as done with the Microsoft Kinect), it is assumed in meters)
   * @param the desired output depth (floats or double)
   * @param out The rescaled float depth image
   */
  CV_EXPORTS
  void
  rescaleDepth(InputArray in, int depth, OutputArray out);

  /** Object that contains a frame data.
   */
  CV_EXPORTS struct RgbdFrame
  {
      RgbdFrame();
      RgbdFrame(const Mat& image, const Mat& depth, const Mat& mask=Mat(), const Mat& normals=Mat(), int ID=-1);

      virtual void
      release();

      int ID;
      Mat image;
      Mat depth;
      Mat mask;
      Mat normals;
  };
} /* namespace cv */

#endif /* __cplusplus */

#endif

/* End of file. */
