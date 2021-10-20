#include "ground_search.hpp"


using namespace Ground_Detection;

DCGD::DCGD(Camera *_camera, double _mini, double _maxi, double _height_err,
          double _size_err, int _step, bool _visualize) {

  camera = _camera;
  mini = _mini;
  maxi = _maxi;
  height_err = _height_err;
  size_err = _size_err;
  step = _step;
  visualize = _visualize;
}


void DCGD::compute_subcuts(std::vector<PointT> cut, std::vector<std::vector<PointT>> &sub_subcuts) {

  std::vector<PointT> sub_sub_subcuts;
  PointT pt, pre_pt;
  double diff;
  int countNanValues = 0; // Where yReal == 10000

  if (cut[0].yReal != 10000)
    sub_sub_subcuts.push_back(cut[0]);
  else
    countNanValues += 1;

  pre_pt = cut[0];
  for ( int j = 1; j < cut.size(); j++ ) {

    pt = cut[j];

    if (pt.yReal != 10000) { // meaning : pt is actually a yReal minimum

      diff = abs(pt.yReal - pre_pt.yReal);

      if (diff < height_err)
        sub_sub_subcuts.push_back(pt);
      else {
        sub_subcuts.push_back(sub_sub_subcuts);
        sub_sub_subcuts.clear();
        sub_sub_subcuts.push_back(pt);
      }
      pre_pt = cut[j];
    }
    else
      countNanValues += 1;

  }

  if (countNanValues == cut.size() || countNanValues == cut.size() - 1)
    return;
  else if (sub_sub_subcuts.size() > 0)
    sub_subcuts.push_back(sub_sub_subcuts);

}


void DCGD::compute_all_subcuts(Array<PointT, Dynamic, Dynamic, RowMajor> cuts,
                            std::map<int, std::vector<std::vector<PointT>>> &subcuts)
{
  std::vector<std::vector<PointT>> sub_subcuts;
  std::vector<PointT> cut_vector(cuts.cols());

  for ( int i = 0; i < cuts.rows(); i++ ) { // Iterate per depth interval

    cut_vector = std::vector<PointT>(cuts.row(i).data(), cuts.row(i).data() + cuts.cols());
    compute_subcuts(cut_vector, sub_subcuts);

    if ( !sub_subcuts.empty() ) {
      subcuts[i] = sub_subcuts;
      sub_subcuts.clear();
    }

  }

}


void DCGD::compute_cuts_downsampling(Array<PointT, Dynamic, Dynamic, RowMajor> &cuts)
{
  Eigen::ArrayXXd  depthMap_ind;
  Eigen::ArrayXXd y_ind;

  int lenList = (maxi - mini + 1) / step + 1;
  int boundary = (maxi - mini) / step + 1;

  PointT ptMax = {0, 0, 10000};

  cuts = Array<PointT, Dynamic, Dynamic, RowMajor>::Constant(lenList, width, ptMax);

  depthMap_ind = (depthMap - mini) / step;

  y_ind = VectorXd::LinSpaced(height, 0.0, height - 1).replicate(1, width);
  y_ind = - (y_ind - camera->get_cy());

  arr_yReal = (depthMap * y_ind) / camera->get_fy();

  PointT pt;
  int indice;

  for ( int i = 0; i < height; i++ ) { // i stands for Ys
    for ( int j = 0; j < width; j++ ) { // j stands for Xs

      indice = (int) depthMap_ind(i, j);

      if (indice >= 0 && indice < boundary)
        if ( arr_yReal(i, j) < cuts(indice, j).yReal ) {

          pt = {j, i, arr_yReal(i, j)};
          cuts(indice, j) = pt;
          //std::cout << cuts(indice, j).yReal << '\n';
        }

    }
  }

}


void DCGD::filter_subcuts(std::vector<std::vector<PointT>> sub_subcuts,
                        std::vector<std::vector<PointT>> &filtered)
{
  int i = 0;
  bool skipped = false;

  for ( const auto &sub_sub_subcuts : sub_subcuts ) {

    if ( skipped ) {
      skipped = false;
      continue;
    }
    if (sub_sub_subcuts.size() >= size_err)
      filtered.push_back(sub_sub_subcuts);

    else if ( !filtered.empty() && (i + 1) < sub_subcuts.size() ) {
      if ( abs(filtered.back().back().yReal - sub_subcuts[i + 1][0].yReal) <= height_err ) {

        filtered.back().insert(filtered.back().end(), sub_subcuts[i + 1].begin(), sub_subcuts[i + 1].end());
        skipped = true;
        i++;
      }
    }

    i++;
  }

}


void DCGD::filter_all_subcuts(std::map<int, std::vector<std::vector<PointT>>> subcuts,
                            std::map<int, std::vector<std::vector<PointT>>> &all_filtered)
{
  std::vector<std::vector<PointT>> filtered;
  int ind;

  for ( const auto &sub_subcuts : subcuts ) {

    ind = sub_subcuts.first;
    filter_subcuts(sub_subcuts.second, filtered);

    if( !filtered.empty() ) {
      all_filtered[ind] = filtered;
      filtered.clear();
    }

  }

}


void DCGD::find_concave_points( std::vector<std::vector<PointT>> filtered,
                                std::vector<PointT> &sub_floorPoints )
{
  const int size = filtered.size();
  int annotations[size - 1];

  if ( size == 1 ) {
    sub_floorPoints.insert(sub_floorPoints.end(), filtered[0].begin(), filtered[0].end());
    return;
  }

  int i = 0;
  for ( const auto &sub_filtered : filtered ) {

    if ( i == 0 ) {

      const auto &next_sub_filtered = filtered[1];
      if ( sub_filtered.back().yReal < next_sub_filtered[0].yReal ) {
        sub_floorPoints.insert(sub_floorPoints.end(), sub_filtered.begin(), sub_filtered.end());
        annotations[0] = 0;
        annotations[1] = 1;
      }
      else {
        annotations[0] = 1;
        annotations[1] = 0;
      }
    }
    else if ( i == size - 1 ) {

      const auto &pre_sub_filtered = filtered[i - 1];
      if ( sub_filtered[0].yReal < pre_sub_filtered.back().yReal )
        sub_floorPoints.insert(sub_floorPoints.end(), sub_filtered.begin(), sub_filtered.end());
    }
    else {

      const auto &next_sub_filtered = filtered[i + 1];

      switch (annotations[i]) {
        case 0: // Marked concave
          if ( sub_filtered.back().yReal < next_sub_filtered[0].yReal ) {
            sub_floorPoints.insert(sub_floorPoints.end(), sub_filtered.begin(), sub_filtered.end());
            annotations[i + 1] = 1;
          }
          else {
            annotations[i] = 1;
            annotations[i + 1] = 0;
          }
          break;

        case 1: // Marked convex
          if ( sub_filtered.back().yReal < next_sub_filtered[0].yReal )
            annotations[i + 1] = 1;
          else
            annotations[i + 1] = 0;
          break;
      }
    }
    
    i ++ ;
  }

}


void DCGD::find_all_concave_points(std::map<int, std::vector<std::vector<PointT>>> all_filtered,
                            std::map<int, std::vector<PointT>> &floorPoints)
{

  std::vector<PointT> sub_floorPoints;
  int ind;

  for ( const auto &filtered : all_filtered )
    if ( filtered.second.size() > 0 ) {
      find_concave_points(filtered.second, sub_floorPoints);
      ind = filtered.first;

      if ( !sub_floorPoints.empty() ) {
        floorPoints[ind] = sub_floorPoints;
        sub_floorPoints.clear();
      }
    }

}


std::vector<DCGD::PointT> DCGD::get_all_floor_points( std::map<int, std::vector<PointT>> floorPoints ) {

  std::vector<PointT> ground;
  PointT curr_pt;
  double yReal;
  int ind;

  for ( int i = 0; i < height; i++ ) {
    for ( int j = 0; j < width; j++ ) {

      ground_indices[i * width + j] = false;
      const auto &d = depthMap(i, j);

      if ( d >= mini && d <= maxi ) {

        ind = (int) ((d - mini) / step);
        if (floorPoints.count(ind) > 0)
          for (const auto &pt : floorPoints[ind])
            if ( j == pt.x ) {
              yReal = arr_yReal(i, j);
              if ( abs(yReal - pt.yReal) <= (height_err * 2) ) {
                curr_pt = {j, i, yReal};
                ground.push_back(curr_pt);
                ground_indices[i * width + j] = true;
                // Visualize
                if (visualize) {
                  rgbImg.at<cv::Vec4b>(i, j)[0] = 255;
                  rgbImg.at<cv::Vec4b>(i, j)[1] = 255;
                  rgbImg.at<cv::Vec4b>(i, j)[2] = 0;
                }
                break;
              }

        }
      }

    }
  }

  return ground;
}


std::vector<DCGD::PointT> DCGD::run(cv::Mat mDepth) {

  Array<PointT, Dynamic, Dynamic, RowMajor> cuts;
  std::map<int, std::vector<std::vector<PointT>>> subcuts, all_filtered, all_labeled;
  std::map<int, std::vector<PointT>> floorPoints;
  std::vector<DCGD::PointT> ground_points;
  Eigen::MatrixXd depthM;

  cv::Mat img_32FC1;

  time_t start, end;

  mDepth.convertTo(img_32FC1, CV_32FC1);

  // Apply median filter on depth map
  medianBlur(img_32FC1, img_32FC1, 5);
  
  cv2eigen(img_32FC1, depthM);
  set_depth_map(depthM);

  time(&start);

  std::cout << "[Info] Computing cuts" << '\n';
  compute_cuts_downsampling(cuts);
  std::cout << "[Info] Cuts computed" << '\n';

  std::cout << "[Info] Computing all subcuts" << '\n';
  compute_all_subcuts(cuts, subcuts);
  std::cout << "[Info] All subcuts computed" << '\n';

  std::cout << "[Info] Filtering all subcuts" << '\n';
  filter_all_subcuts(subcuts, all_filtered);
  std::cout << "[Info] All subcuts filtered" << '\n';

  std::cout << "[Info] Labeling all subcuts" << '\n';
  find_all_concave_points(all_filtered, floorPoints);
  std::cout << "[Info] All filtered subcuts labeled" << '\n';

  std::cout << "[DCGD] Found : " << floorPoints.size() << " minimal points." << '\n';
  ground_points = get_all_floor_points(floorPoints);
  std::cout << "[DCGD] Found : " << ground_points.size() << " points." << '\n';

  time(&end);
  std::cout << "[Execution] Consumed time : " << '\n' << difftime(end,start) << '\n';

  return ground_points;
}
