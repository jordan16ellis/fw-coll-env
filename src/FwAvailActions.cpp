#include <fw-coll-env/FwAvailActions.h>

#include <algorithm>

namespace fw_coll_env {

FwAvailActions::FwAvailActions(
  const std::vector<double> &v,
  const std::vector<double> &w,
  const std::vector<double> &dz) : v_(v), w_deg_per_sec_(w), dz_(dz) {

  w_rad_per_sec_.reserve(w.size());
  std::transform(w.begin(), w.end(), std::back_inserter(w_rad_per_sec_), deg2rad);

  all_actions_.reserve(v.size() * w.size() * dz.size());

  for (double _v : v) {
    for (double _w : w_rad_per_sec_) {
      for (double _dz : dz) {
        FwSingleAction ac {_v, _w, _dz};
        all_actions_.push_back(ac);
        action_to_idx_[ac] = all_actions_.size() - 1;
      }
    }
  }

  auto vec2str = [&](const auto &vec) {
    std::string out;
    for (size_t i = 0; i < vec.size(); i++) {
      out += std::to_string(vec[i]);
      if (i < vec.size() - 1) {
        out += ",";
      }
    }
    return out;
  };
  repr_ = "FwAvailActions(v=[" +
    vec2str(v) + "],w=[" + vec2str(w) + "],dz=[" + vec2str(dz) + "])";
}

size_t FwAvailActions::action_to_idx(const FwSingleAction &ac) const {
  auto it = action_to_idx_.find(ac);
  if (it == action_to_idx_.end()) {
    std::string msg = std::string("could not find action in action_to_idx: ") +
      ac.to_string() + "\n" +
      "All actions are:\n";
    for (auto &ac : all_actions_) {
      msg += ac.to_string() + "\n";
    }
    throw std::runtime_error(msg);
  }
  return it->second;
}

FwSingleAction FwAvailActions::idx_to_action(size_t idx) {
  if (idx >= all_actions_.size()) {
    throw std::runtime_error("idx too large for all_actions");
  }
  return all_actions_[idx];
}
} // namespace fw_coll_env
