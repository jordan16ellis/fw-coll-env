#ifndef INCLUDE_FW_COLL_ENV_FWCOLLISIONENV_H_
#define INCLUDE_FW_COLL_ENV_FWCOLLISIONENV_H_

#include <fw-coll-env/Utils.h>

#include <vector>
#include <limits>
#include <string>
#include <chrono>  // NOLINT

namespace fw_coll_env {

struct FwEnvStats {
  bool done_time = false;
  bool done_goal = false;
  bool done_collision = false;

  double dist_to_goal1 = std::numeric_limits<double>::quiet_NaN();
  double dist_to_goal2 = std::numeric_limits<double>::quiet_NaN();
  double dist_to_veh = std::numeric_limits<double>::quiet_NaN();

  std::string to_string() const;
};

class FwCollisionEnv {
 public:
  FwCollisionEnv() {}
  FwCollisionEnv(
    double dt, double max_sim_time, double done_dist,
    double safety_dist,
    const Point &goal1,
    const Point &goal2,
    double time_warp);

  bool step(const FwSingleAction &a1, const FwSingleAction &a2);
  void reset(const FwSingleState &x1, const FwSingleState &x2, double t);
  std::string to_string() const;

  const FwSingleState& get_x1() const {return x1_;}
  const FwSingleState& get_x2() const {return x2_;}
  double get_dt() const {return dt_;}
  double get_t() const {return t_;}
  const Point &get_goal1() const {return goal1_;}
  const Point &get_goal2() const {return goal2_;}
  double get_safety_dist() const {return safety_dist_;}
  double get_done_dist() const {return done_dist_;}
  double get_max_sim_time() const {return max_sim_time_;}
  double get_time_warp() const {return time_warp_;}

  bool get_done() const {return stats.done_time || stats.done_goal;}

  bool get_collided() const {return stats.done_collision;}

  FwEnvStats stats;

 protected:
  void update_stats();

  double dt_;
  double max_sim_time_;
  double done_dist_;
  double safety_dist_;
  Point goal1_;
  Point goal2_;

  double time_warp_;
  std::chrono::high_resolution_clock::time_point last_update_time_;
  double t_;

  FwSingleState x1_;
  FwSingleState x2_;
};

} // namespace fw_coll_env
#endif // INCLUDE_FW_COLL_ENV_FWCOLLISIONENV_H_
