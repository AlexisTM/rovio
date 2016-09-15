/*
* Copyright (c) 2014, Autonomous Systems Lab
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
* names of its contributors may be used to endorse or promote products
* derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/


#include <memory>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#pragma GCC diagnostic pop

#include "rovio/RovioFilter.hpp"
#include "rovio/RovioNode.hpp"
#include "rovio/RovioScene.hpp"

#ifndef ROVIO_NMAXFEATURE
#define ROVIO_NMAXFEATURE 25
#endif

#ifndef ROVIO_NLEVELS
#define ROVIO_NLEVELS 4
#endif

#ifndef ROVIO_PATCHSIZE
#define ROVIO_PATCHSIZE 6
#endif

#ifndef ROVIO_NCAM
#define ROVIO_NCAM 1
#endif

#ifndef ROVIO_NPOSE
#define ROVIO_NPOSE 0
#endif

static rovio::RovioScene<mtFilter> mRovioScene;

void idleFunc(){
  ros::spinOnce();
  mRovioScene.drawScene(mRovioScene.mpFilter_->safe_);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "rovio");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string rootdir = ros::package::getPath("rovio"); // Leaks memory
  std::string filter_config = rootdir + "/cfg/rovio.info";

  static int nMax_, nLevels_, patchSize_, nCam_, nPose_;
  static bool make_scene_;
  nh.param("features", nMax_, ROVIO_NMAXFEATURE); // Maximal number of considered features in the filter state.
  nh.param("nlevel", nLevels_, ROVIO_NLEVELS); // Total number of pyramid levels considered.
  nh.param("patchsize", patchSize_, ROVIO_PATCHSIZE); // Edge length of the patches (in pixel). Must be a multiple of 2!
  nh.param("ncam", nCam_, ROVIO_NCAM); // Used total number of cameras.
  nh.param("npose", nPose_, ROVIO_NPOSE); // Additional pose states.
  nh.param("makescene", make_scene_, false); // do we make the scene

  typedef rovio::RovioFilter<rovio::FilterState<nMax_, nLevels_, patchSize_, nCam_, nPose_>> mtFilter;

  nh_private.param("filter_config", filter_config, filter_config);

  // Filter
  std::shared_ptr<mtFilter> mpFilter(new mtFilter);
  mpFilter->readFromInfo(filter_config);

  // Force the camera calibration paths to the ones from ROS parameters.
  for (unsigned int camID = 0; camID < nCam_; ++camID) {
    std::string camera_config;
    if (nh_private.getParam("camera" + std::to_string(camID)
                            + "_config", camera_config)) {
      mpFilter->cameraCalibrationFile_[camID] = camera_config;
    }
  }
  mpFilter->refreshProperties();

  // Node
  rovio::RovioNode<mtFilter> rovioNode(nh, nh_private, mpFilter);
  rovioNode.makeTest();

  if (make_scene_) {
    // Scene
    std::string mVSFileName = rootdir + "/shaders/shader.vs";
    std::string mFSFileName = rootdir + "/shaders/shader.fs";
    mRovioScene.initScene(argc, argv, mVSFileName, mFSFileName, mpFilter);
    mRovioScene.setIdleFunction(idleFunc);
    mRovioScene.addKeyboardCB('r', [&rovioNode]() mutable {rovioNode.requestReset(); });
    glutMainLoop();
  }
  else {
    ros::spin();
  }
  
  return 0;
}
