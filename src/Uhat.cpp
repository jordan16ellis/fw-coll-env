
#include <fw-coll-env/Uhat.h>

#include <cmath>

namespace fw_coll_env {

FwSingleAction Uhat::calc(const FwSingleState &x0) {
  double gain = 1;
  double l = 1;

  double vx = gain * (goal_.x - x0.p.x);
  double vy = gain * (goal_.y - x0.p.y);
  double v_norm = std::max(1.0, std::sqrt(vx*vx + vy*vy));
  vx /= v_norm;
  vy /= v_norm;

  double M00 = std::cos(x0.th);
  double M01 = std::sin(x0.th);
  double M10 = -M01 / l;
  double M11 = M00 / l;

  double v = M00 * vx + M01 * vy;
  double omega = M10 * vx + M11 * vy;

  auto closest = [&](auto &vals, double val) {
    for (int i = 0; i < vals.size(); i++) {
      if (vals[i] > val) {
        if (i == 0) {
          return vals[0];
        } else {
          return vals[i] - val > val - vals[i] ? vals[i - 1] : vals[i];
        }
      }
    }

    double a = vals[vals.size() - 1];
    return a;
  };

  return {closest(avail_actions_.get_v(), v),
          closest(avail_actions_.get_w_rad_per_sec(), omega),
          0};


  // double best_dist = std::numeric_limits<double>::infinity();
  // FwSingleAction best_action;

  // const auto &all_actions = avail_actions_.get_all_actions();

  // for (const auto &ac : all_actions) {
  //   FwSingleState x = x0;

  //   // apply dynamics twice so that theta has an effect
  //   fw_dynamics(dt_, ac, x);
  //   fw_dynamics(dt_, ac, x);

  //   double d = x.p.dist(goal_);
  //   if (d < best_dist) {
  //     best_dist = d;
  //     best_action = ac;
  //   }
  // }

  // return best_action;
}
} // namespace fw_coll_env
