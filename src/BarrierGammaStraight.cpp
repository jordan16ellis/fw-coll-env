#include <fw-coll-env/BarrierGammaStraight.h>

#include <cmath>
#include <iostream>

namespace fw_coll_env {

BarrierGammaStraight::BarrierGammaStraight(
      double dt, double max_val, double v, double safety_dist,
      const FwAvailActions &avail_actions) :
        BarrierGammaTurn(
            dt, max_val, v, avail_actions.get_w_deg_per_sec()[0],
            safety_dist, avail_actions) {
  w_rad_per_sec_ = 0;
  // make sure v, 0, and 0 for dt are in avail actions
  // so throw exception on action_to_idx if this is not the case.
  FwSingleAction ac {v_, 0, 0};
  avail_actions_.action_to_idx(ac);
}

double BarrierGammaStraight::closest_future_dist(const FwState &x0) {

  // integrate a maximum of num seconds (currently hardcoded to match env)
  const int n = 30 / dt_;
  FwState x = x0;
  FwSingleAction ac {v_, 0, 0};
  double closest_dist = x.x1.p.dist(x.x2.p);
  for (int i = 0; i < n; i++) {
      fw_dynamics(dt_, ac, x.x1);
      fw_dynamics(dt_, ac, x.x2);

      const double dist = x.x1.p.dist(x.x2.p);
      if (dist < closest_dist) {
        closest_dist = dist;
      } else {
        // since this is a straight trajectory we have already found
        // the smallest point
        break;
      }
  }
  return closest_dist;
}

std::string BarrierGammaStraight::to_string() const {
  return std::string("BarrierGammaStraight(dt=") + std::to_string(dt_) +
    ",max_val=" + std::to_string(max_val_) +
    ",v=" + std::to_string(v_) +
    ",safety_dist=" + std::to_string(safety_dist_) + ")";
}
} // namespace fw_coll_env
