#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include <xtensor/xadapt.hpp>

#include "include/ground_search.hpp"

using namespace Ground_Detection;
using namespace xt;
using namespace cv;

int main() {

  Camera *camera = new Camera(0, 5.8269e2, 0, 2.3844e2);
  DCGD *dcgd = new DCGD(camera, 800, 4000, 20, 3, 15);

  Mat img = imread("data/depth/depth.png",-1);
  Mat img_32FC1;

  img.convertTo(img_32FC1, CV_32F);
  //std::cout << img << '\n';
  size_t size = img.total();
  size_t channels = img.channels();
  std::vector<size_t> shape = {img.cols, img.rows, img.channels()};

  xarray<float> depthMap = adapt((float*)img_32FC1.data, size * channels, no_ownership(), shape);
  //std::cout << depthMap << '\n';
  dcgd->dcgd_process_donsampling(depthMap);

}
