#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "include/ground_search.hpp"

#include <opencv2/core/eigen.hpp>

using namespace Ground_Detection;
using namespace cv;

int main() {

  Camera *camera = new Camera(0, 5.8269e+02, 0, 2.3844e+02); // Those are Kinect V1 intrinsics, because frames from data folder are from Kinect v1.
  DCGD *dcgd = new DCGD(camera, 800, 4000, 20, 3, 15, true);

  Mat img = imread("data/depth/depth.png",-1);
  Mat rgbImg = imread("data/rgb/rgb.jpg",-1);
  Mat img_32FC1;

  img.convertTo(img_32FC1, CV_32FC1);
  //std::cout << img << '\n';
  size_t size = img_32FC1.total();
  size_t channels = img_32FC1.channels();
  std::vector<size_t> shape = {img_32FC1.cols, img_32FC1.rows, img_32FC1.channels()};

  // Apply median filter on depth map
  medianBlur(img_32FC1, img_32FC1, 3);
  //xarray<float> depthMap = adapt((float*)img_32FC1.data, size * channels, no_ownership(), shape);
  Eigen::MatrixXd depthMap;
  cv2eigen(img_32FC1, depthMap);
  //std::cout << depthMap << '\n';

  std::map<int, std::vector<DCGD::PointT>> minimal_ground;
  std::vector<DCGD::PointT> ground;

  dcgd->set_depth_map(depthMap.array());
  dcgd->set_rgb_image(rgbImg);
  minimal_ground = dcgd->dcgd_process_donsampling();
  std::cout << "[DCGD] Found : " << minimal_ground.size() << " minimal points." << '\n';
  ground = dcgd->get_all_floor_points(minimal_ground);
  std::cout << "[DCGD] Found : " << ground.size() << " points." << '\n';

  namedWindow( "windowName_", CV_WINDOW_AUTOSIZE);
  imshow( "windowName_", rgbImg); // Show the image.

  waitKey(0); // Wait for a keystroke in the window

  return 0;
}
