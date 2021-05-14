
#include <fw-coll-env/FwCollisionEnv.h>

#include <thread>  // NOLINT

namespace fw_coll_env {

std::string FwEnvStats::to_string() const {
  return std::string("FwEnvStats(done_time=") + bool2str(done_time) +
    ",done_goal=" + bool2str(done_goal) +
    ",done_collision=" + bool2str(done_collision) +
    ",dist_to_goal1=" + std::to_string(dist_to_goal1) +
    ",dist_to_goal2=" + std::to_string(dist_to_goal2) +
    ",dist_to_veh=" + std::to_string(dist_to_veh) + ")";
}

FwCollisionEnv::FwCollisionEnv(
  double dt, double max_sim_time, double done_dist,
  double safety_dist,
  const Point &goal1,
  const Point &goal2,
  double time_warp) :
    dt_(dt), max_sim_time_(max_sim_time), done_dist_(done_dist),
    safety_dist_(safety_dist), goal1_(goal1), goal2_(goal2),
    time_warp_(time_warp),
    last_update_time_(std::chrono::high_resolution_clock::now()),
    t_(0) {}

bool FwCollisionEnv::step(const FwSingleAction &a1, const FwSingleAction &a2) {
  if (time_warp_ > 0 and t_ > dt_ / 2) {
    auto tgt_time = last_update_time_ +
      std::chrono::duration<int, std::milli>(1000 * static_cast<int>(dt_ / time_warp_));
    std::this_thread::sleep_for(tgt_time - std::chrono::high_resolution_clock::now());
    last_update_time_ = std::chrono::high_resolution_clock::now();
  }
  t_ += dt_;
  fw_dynamics(dt_, a1, x1_);
  fw_dynamics(dt_, a2, x2_);

  update_stats();

  return get_done();
}

void FwCollisionEnv::update_stats() {
  stats.dist_to_goal1 = x1_.p.dist(goal1_);
  stats.dist_to_goal2 = x2_.p.dist(goal2_);
  stats.dist_to_veh = x1_.p.dist(x2_.p);

  stats.done_time = t_ >= max_sim_time_;
  stats.done_collision = stats.dist_to_veh <= safety_dist_;
  stats.done_goal =
    stats.dist_to_goal1 <= done_dist_ ||
    stats.dist_to_goal2 <= done_dist_;
}

void FwCollisionEnv::reset(const FwSingleState &x1, const FwSingleState &x2, double t) {
  x1_ = x1;
  x2_ = x2;
  t_ = t;
  last_update_time_ = std::chrono::high_resolution_clock::now();

  update_stats();
}

std::string FwCollisionEnv::to_string() const {
  return std::string("FwCollisionEnv::(dt=") + std::to_string(dt_) +
    ", max_sim_time=" + std::to_string(max_sim_time_) +
    ", done_dist=" + std::to_string(done_dist_) +
    ", safety_dist=" + std::to_string(safety_dist_) +
    ", goal1=" + goal1_.to_string() +
    ", goal2=" + goal2_.to_string() +
    ", time_warp=" + std::to_string(time_warp_) + ")";
}
} // namespace fw_coll_env
