namespace Ground_Detection {

  class Camera {

  private:

    double fx;
    double fy;
    double cx;
    double cy;

  public:

    Camera(double _fx, double _fy, double _cx, double _cy) {

      fx = _fx;
      fy = _fy;
      cx = _cx;
      cy = _cy;
    }

    double get_fx() {
      return fx;
    }

    double get_fy() {
      return fy;
    }

    double get_cx() {
      return cx;
    }

    double get_cy() {
      return cy;
    }

  };
}
