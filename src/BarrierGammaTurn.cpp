
#include <fw-coll-env/BarrierGammaTurn.h>

#include <cmath>

namespace fw_coll_env {

BarrierGammaTurn::BarrierGammaTurn(
      double dt, double max_val, double v, double w_deg_per_sec, double safety_dist,
      const FwAvailActions &avail_actions) :
    dt_(dt), max_val_(max_val),
    v_(v), w_rad_per_sec_(deg2rad(w_deg_per_sec)),
    safety_dist_(safety_dist), avail_actions_(avail_actions),
    action_index_(avail_actions) {

  const double freq = 2 * M_PI / w_rad_per_sec_;
  to_int(freq);
  to_int(1 / dt_);

  // make sure v, w, and 0 for dt are in avail actions
  // so throw exception on action_to_idx if this is not the case.
  FwSingleAction ac {v_, w_rad_per_sec_, 0};
  avail_actions_.action_to_idx(ac);
}

double BarrierGammaTurn::calc_h(const FwState &x0) {
  double d = closest_future_dist(x0);
  return std::min(max_val_, d - safety_dist_);
}

double BarrierGammaTurn::calc_dh(const FwState &x0, const FwAction &ac) {
  FwState x = x0;
  fw_dynamics(dt_, ac.a1, x.x1);
  fw_dynamics(dt_, ac.a2, x.x2);
  return calc_h(x) - calc_h(x0);
}

size_t BarrierGammaTurn::steps_per_revolution() const {
  // already checked this is an integer in constructor
  return 2 * M_PI / w_rad_per_sec_ / dt_;
}

double BarrierGammaTurn::closest_future_dist(const FwState &x0) {

  FwState x = x0;
  FwSingleAction ac {v_, w_rad_per_sec_, 0};
  double closest_dist = x.x1.p.dist(x.x2.p);
  const size_t n = steps_per_revolution();
  for (size_t i = 0; i < n; i++) {
      fw_dynamics(dt_, ac, x.x1);
      fw_dynamics(dt_, ac, x.x2);
      closest_dist = std::min(closest_dist, x.x1.p.dist(x.x2.p));
  }
  return closest_dist;
}

double BarrierGammaTurn::bf_constraint(
    double h, const FwState &x0, const FwAction &_ac) {
  FwState x = x0;
  fw_dynamics(dt_, _ac.a1, x.x1);
  fw_dynamics(dt_, _ac.a2, x.x2);
  double hnext = calc_h(x);
  return (hnext - h) + lambda_ * h;
}

pybind11::array_t<int> BarrierGammaTurn::choose_u(
    pybind11::array_t<double> x, pybind11::array_t<int> uhat_idx) {

  int num_rows = x.shape(0);
  if (x.shape(1) != 8 || uhat_idx.shape(0) != num_rows) {
    throw std::runtime_error("invalid shape given to choose_u");
  }

  pybind11::array_t<int> out {num_rows};

  auto _x = x.unchecked<2>();
  auto _uhat_idx = uhat_idx.unchecked<1>();
  auto _out = out.mutable_unchecked<1>();

  for (int i = 0; i < num_rows; i++) {
    FwState x_state {
      FwSingleState(Point(_x(i, 0), _x(i, 1), _x(i, 3)), _x(i, 2)),
      FwSingleState(Point(_x(i, 4), _x(i, 5), _x(i, 7)), _x(i, 6))
    };

    FwAction uhat_ac = action_index_.idx_to_action(_uhat_idx(i));
    FwAction safe_ac = choose_u_single(x_state, uhat_ac);
    _out(i) = action_index_.action_to_idx(safe_ac);
  }

  return out;
}

FwAction BarrierGammaTurn::choose_u_single(const FwState &x0, const FwAction &uhat) {
  double h = calc_h(x0);

  double orig_bf_val = bf_constraint(h, x0, uhat);
  if (orig_bf_val >= 0) {
    return uhat;
  }

  double best_ac_dist = std::numeric_limits<double>::infinity();
  FwAction best_ac = uhat;
  double best_bf_val = orig_bf_val;

  const auto &all_actions = avail_actions_.get_all_actions();
  for (const auto &ac1 : all_actions) {
    for (const auto &ac2 : all_actions) {
      FwAction ac {ac1, ac2};
      double temp_bf_val = bf_constraint(h, x0, ac);

      if ((best_bf_val >= 0 && temp_bf_val < 0) ||
          (best_bf_val < 0 && temp_bf_val < best_bf_val)) {
        // exclude actions that are not safe
        // and are farther outside the safe
        // set than the currently selected action
        continue;
      }

      double temp_ac_dist = ac1.dist(uhat.a1) + ac2.dist(uhat.a2);

      if ((best_bf_val < 0 && temp_bf_val > best_bf_val) ||
          (best_bf_val >= 0 && temp_bf_val >= 0 && best_ac_dist > temp_ac_dist)) {
        // if the current action is not safe and this is safer
        // or if the current acton safe and this one is both
        // safe and closer
        best_bf_val = temp_bf_val;
        best_ac = ac;
        best_ac_dist = temp_ac_dist;
      }
    }
  }

  return best_ac;
}

std::string BarrierGammaTurn::to_string() const {
  return std::string("BarrierGammaTurn(dt=") + std::to_string(dt_) +
    ",max_val=" + std::to_string(max_val_) +
    ",v=" + std::to_string(v_) +
    ",w_deg_per_sec=" + std::to_string(rad2deg(w_rad_per_sec_)) +
    ",safety_dist=" + std::to_string(safety_dist_) + ")";
}
} // namespace fw_coll_env
