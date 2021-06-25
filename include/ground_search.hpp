#include <stdio.h>
#include <stdlib.h>
#include <map>
#include <time.h>
// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
// OPENCV
#include <opencv2/opencv.hpp>
// Internal Libraries
#include "camera.hpp"

using namespace Eigen;
using namespace cv;

namespace Ground_Detection{

  class DCGD {

  private:

    ArrayXXd depthMap;
    ArrayXXd arr_yReal;

    cv::Mat rgbImg;

    int width, height;

    Camera *camera;
    double mini, maxi;
    double height_err, size_err;
    int step;

    bool visualize;

  public:

    struct PointT { int x; int y; double yReal; };

    DCGD( Camera *_camera, double _mini, double _maxi, double _height_err,
          double _size_err, int _step, bool _visualize);

    void set_depth_map( ArrayXXd _depthMap ) {
      depthMap = _depthMap;
      width = depthMap.cols();
      height = depthMap.rows();
    }

    void set_rgb_image( cv::Mat _rgbImg ) {
      rgbImg = _rgbImg;
    }

    void compute_subcuts( std::vector<PointT> cut,
                        std::vector<std::vector<PointT>> &sub_subcuts );

    void compute_all_subcuts( Array<PointT, Dynamic, Dynamic, RowMajor> cuts,
                            std::map<int, std::vector<std::vector<PointT>>> &subcuts );

    void find_concave_points( std::vector<std::vector<PointT>> filtered,
                            std::vector<PointT> &sub_floorPoints );

    void find_all_concave_points( std::map<int, std::vector<std::vector<PointT>>> all_filtered,
                                std::map<int, std::vector<PointT>> &floorPoints );

    void filter_subcuts( std::vector<std::vector<PointT>> sub_subcuts, std::vector<std::vector<PointT>> &filtered );

    void filter_all_subcuts( std::map<int, std::vector<std::vector<PointT>>> subcuts,
                                std::map<int, std::vector<std::vector<PointT>>> &all_filtered );

    void compute_cuts_downsampling(Array<PointT, Dynamic, Dynamic, RowMajor> &cuts );

    std::vector<PointT> get_all_floor_points( std::map<int, std::vector<PointT>> floorPoints );

    std::map<int, std::vector<PointT>> dcgd_process_donsampling( );

  };
}
