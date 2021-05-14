
#ifndef INCLUDE_FW_COLL_ENV_UTILS_H_
#define INCLUDE_FW_COLL_ENV_UTILS_H_

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include <limits>
#include <string>

namespace fw_coll_env {

struct Point {
  Point() : x(std::numeric_limits<double>::quiet_NaN()),
            y(std::numeric_limits<double>::quiet_NaN()),
            z(std::numeric_limits<double>::quiet_NaN()) {}
  Point(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}

  double x;
  double y;
  double z;

  bool operator==(const Point &p) const;
  double dist(const Point &p) const;
  std::string to_string() const;
};

struct FwSingleState {
  FwSingleState() : th(std::numeric_limits<double>::quiet_NaN()) {}
  FwSingleState(const Point &_p, double _th) : p(_p), th(_th) {}
  Point p;
  double th;

  bool operator==(const FwSingleState &_x) const;
  std::string to_string() const;
};

struct FwState {
  FwState() {}
  FwState(const FwSingleState &_x1, const FwSingleState &_x2) :
    x1(_x1), x2(_x2) {}

  pybind11::array_t<double> asarray() const;

  FwSingleState x1;
  FwSingleState x2;

  std::string to_string() const;
};

struct FwSingleAction {
  // this is basically the same thing as Point
  // but with different names to the members.
  // An initial implementation using only a 3 entry vector
  // led to bugs where the wrong members were called
  // so this is more explicit
  FwSingleAction() : v(std::numeric_limits<double>::quiet_NaN()),
                     w(std::numeric_limits<double>::quiet_NaN()),
                     dz(std::numeric_limits<double>::quiet_NaN()) {}
  FwSingleAction(double _v, double _w, double _dz) : v(_v), w(_w), dz(_dz) {}
  double v;
  double w;
  double dz;

  bool operator==(const FwSingleAction &ac) const;
  bool operator<(const FwSingleAction &ac) const;
  double dist(const FwSingleAction &ac) const;
  double dist_sq(const FwSingleAction &ac) const;
  std::string to_string() const;
};

struct FwAction {
  FwAction(const FwSingleAction &_a1, const FwSingleAction &_a2) : a1(_a1), a2(_a2) {}
  FwSingleAction a1;
  FwSingleAction a2;

  bool operator==(const FwAction &ac) const;
  double dist(const FwAction &ac) const;
  std::string to_string() const;
};

class Rho {
 public:
  Rho() : safety_dist_(std::numeric_limits<double>::quiet_NaN()),
          max_val_(std::numeric_limits<double>::quiet_NaN()) {}
  explicit Rho(double safety_dist, double max_val) :
      safety_dist_(safety_dist), max_val_(max_val) {}

  double operator()(const FwState &x) const;

  double get_safety_dist() const {return safety_dist_;}
  double get_max_val() const {return max_val_;}
  std::string to_string() const;

 protected:
  double safety_dist_;
  double max_val_;
};

void fw_dynamics(double dt, const FwSingleAction &a, FwSingleState &x);
std::string bool2str(bool val);
double deg2rad(double val);
double rad2deg(double val);
int to_int(double val);

} // namespace fw_coll_env
#endif // INCLUDE_FW_COLL_ENV_UTILS_H_
