#include "ground_search.hpp"


using namespace Ground_Detection;

DCGD::DCGD(Camera *_camera, double _mini, double _maxi, double _height_err, double _size_err, int _step) {

  camera = _camera;
  mini = _mini;
  maxi = _maxi;
  height_err = _height_err;
  size_err = _size_err;
  step = _step;
}


void DCGD::compute_subcuts(xarray<PointT> cuts) {


}

/*
void DCGD::compute_all_subcuts(std::vector<std::vector<float>> cuts,
                          &subcuts) {


}

*/
void DCGD::compute_cuts_downsampling( xarray<float> depthMap, xarray<PointT> &cuts)
{
  xarray<int>  depthMap_ind, y_ind;
  xarray<double> arr_yReal;

  int  width = depthMap.shape(0), height = depthMap.shape(1);
  int boundary = (maxi - mini) / step + 1;

  PointT ptMax = {0, 0, 10000};
  xarray<PointT>::shape_type shape = {boundary, width};

  cuts = xarray<PointT>(shape, ptMax);

  depthMap_ind = (depthMap - mini) / step;
  //std::cout << depthMap_ind << '\n';

  // USING XARRAY IMPLEMENTATION
  /*
  filter1 = depthMap_ind >= 0;
  filter2 = depthMap_ind < boundary;
  depthMap_ind = where(depthMap_ind >= 0 && depthMap_ind < boundary, depthMap_ind, NULL);

  for( int i = 0; i < boundary; i++) {

    arr = where(equal(depthMap_ind, 0), arr_yReal, NULL);
    auto min = amin(arr, 0, evaluation_strategy::immediate);
    //std::cout << min.size() << '\n';
    min_yReal_ind = from_indices(where(equal(arr, min)));
    //std::cout << min_yReal_ind << '\n';
  }*/

  y_ind = arange(height);
  //for (auto& el : y_ind.shape()) {std::cout << el << ", "; }
  y_ind = - (y_ind - camera->get_cy());
  arr_yReal = linalg::dot(depthMap, view(y_ind, all(), NULL)) / camera->get_fy();
  //std::cout << arr_yReal << '\n';

  PointT pt;
  int indice;

  for ( int i = 0; i < width; i++ ) {
    for ( int j = 0; j < height; j++ ) {

      indice = depthMap_ind(i, j);

      if (indice >= 0 && indice < boundary) {
        if ( arr_yReal(i, j) < cuts(indice, i).yReal ) {

          pt = {i, j, arr_yReal(i, j)};
          cuts(indice, i) = pt;
          //std::cout <<  i << '\t' << j << '\t' << arr(indice, i).yReal<< '\n';
        }
      }

    }
  }

}


void DCGD::dcgd_process_donsampling(xarray<float> depthMap) {

  xarray<PointT> cuts;
  std::vector<std::vector<float>> subcuts, filtered, noise, labels, floorPoints;
  time_t start, end;
  int lenList = (maxi - mini + 1) / (step + 1);

  time(&start);

  std::cout << "[Info] Computing cuts" << '\n';
  compute_cuts_downsampling(depthMap, cuts);
  std::cout << "[Info] Cuts computed" << '\n';

  std::cout << "[Info] Computing all subcuts" << '\n';
  compute_all_subcuts(cuts);
  std::cout << "[Info] All subcuts computed" << '\n';

  std::cout << "[Info] Filtering all subcuts" << '\n';
  //filter_all_subcuts(subcuts, filtered, noise);
  std::cout << "[Info] All subcuts filtered" << '\n';

  std::cout << "[Info] Labeling all subcuts" << '\n';
  //label_all_subcuts(filtered, labels);
  std::cout << "[Info] All filtered subcuts labeled" << '\n';

  time(&end);
  std::cout << "[Execution] Consumed time : " << '\n' << difftime(end,start) << '\n';

  // TODO
  //return floorPoints;

}
