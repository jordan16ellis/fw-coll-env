#include <fw-coll-env/Utils.h>

#include <cmath>
#include <stdexcept>

namespace fw_coll_env {

void fw_dynamics(double dt, const FwSingleAction &a, FwSingleState &x) {
  x.p.x += a.v * std::cos(x.th) * dt;
  x.p.y += a.v * std::sin(x.th) * dt;
  x.th += a.w * dt;
  x.p.z += a.dz * dt;
}

std::string bool2str(bool val) {
  return val ? "True" : "False";
}

double Point::dist(const Point &p) const {
  return std::sqrt(
    std::pow(x - p.x, 2) +
    std::pow(y - p.y, 2) +
    std::pow(z - p.z, 2));
}

bool Point::operator==(const Point &p) const {
  return x == p.x && y == p.y && z == p.z;
}

std::string Point::to_string() const {
  return std::string("Point(x=") + std::to_string(x) +
    ",y=" + std::to_string(y) + ",z=" + std::to_string(z) + ")";
}

bool FwSingleState::operator==(const FwSingleState &_x) const {
  return p == _x.p && th == _x.th;
}

std::string FwSingleState::to_string() const {
  return std::string("FwSingleState(p=") + p.to_string() +
    ",th=" + std::to_string(th) + ")";
}

pybind11::array_t<double> FwState::asarray() const {
  pybind11::array_t<double> out{8};
  out.mutable_at(0) = x1.p.x;
  out.mutable_at(1) = x1.p.y;
  out.mutable_at(2) = x1.th;
  out.mutable_at(3) = x1.p.z;

  out.mutable_at(4) = x2.p.x;
  out.mutable_at(5) = x2.p.y;
  out.mutable_at(6) = x2.th;
  out.mutable_at(7) = x2.p.z;
  return out;
}

std::string FwState::to_string() const {
  return std::string("FwState(x1=") + x1.to_string() +
    ",x2=" + x2.to_string() + ")";
}

double FwSingleAction::dist_sq(const FwSingleAction &ac) const {
  return std::pow(v - ac.v, 2) +
    std::pow(w - ac.w, 2) +
    std::pow(dz - ac.dz, 2);
}

double FwSingleAction::dist(const FwSingleAction &ac) const {
  return std::sqrt(dist_sq(ac));
}

bool FwSingleAction::operator==(const FwSingleAction &ac) const {
  return v == ac.v && w == ac.w && dz == ac.dz;
}

bool FwSingleAction::operator<(const FwSingleAction &ac) const {
  return v < ac.v ||
    (v == ac.v && w < ac.w) ||
    (v == ac.v && w == ac.w && dz < ac.dz);
}

std::string FwSingleAction::to_string() const {
  return std::string("FwSingleAction(v=") + std::to_string(v) +
    ",w=" + std::to_string(w) + ",dz=" + std::to_string(dz) + ")";
}

bool FwAction::operator==(const FwAction &ac) const {
  return a1 == ac.a1 && a2 == ac.a2;
}

std::string FwAction::to_string() const {
  return std::string("FwAction(a1=") + a1.to_string() +
    ",a2=" + a2.to_string() + ")";
}

double FwAction::dist(const FwAction &ac) const {
  return std::sqrt(a1.dist_sq(ac.a1) + a2.dist_sq(ac.a2));
}

double Rho::operator()(const FwState &x) const {
  return std::min(max_val_, x.x1.p.dist(x.x2.p) - safety_dist_);
}

std::string Rho::to_string() const {
  return std::string("Rho(safety_dist=") + std::to_string(safety_dist_) +
      ",max_val=" + std::to_string(max_val_) + ")";
}

double deg2rad(double val) {
  return val * M_PI / 180.0;
}

double rad2deg(double val) {
  return val * 180.0 / M_PI;
}

int to_int(double val) {
  double out = std::round(val);
  if (std::abs(val - out) > 0.01) {
    throw std::runtime_error(std::to_string(val) + " is not an int");
  }
  return static_cast<int>(out);
}
} // namespace fw_coll_env
