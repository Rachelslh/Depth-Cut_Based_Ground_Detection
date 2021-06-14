#include <stdio.h>
#include <time.h>
// Base Arrays
#include <xtensor/xarray.hpp>
// XTensor IO:
#include <xtensor/xio.hpp>
// XTensor View:
#include "xtensor/xview.hpp"
#include <xtensor/xreducer.hpp>
// XTensor BLAS
#include <xtensor-blas/xlinalg.hpp>
// OPENCV
#include <opencv2/opencv.hpp>
// Internal Libraries
#include "camera.hpp"

using namespace xt;

namespace Ground_Detection{

  class DCGD {

  private:

    Camera *camera;
    double mini, maxi;
    double height_err, size_err;
    int step;

  public:

    struct PointT { int x; int y; double yReal;};

    DCGD(Camera *_camera, double _mini, double _maxi, double _height_err, double _size_err, int _step);

    void compute_subcuts();

    void compute_all_subcuts( xarray<PointT> &cuts );

    void check_and_update_annotation();

    void label_subcuts();

    void label_all_subcuts();

    void filter_subcuts();

    void filter_all_subcuts();

    void compute_cuts_downsampling(xarray<float> depthMap, xarray<PointT> &cuts);

    void dcgd_process_donsampling(xarray<float> depthMap);

  };
}
