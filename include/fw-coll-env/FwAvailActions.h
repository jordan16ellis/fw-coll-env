
#ifndef INCLUDE_FW_COLL_ENV_FWAVAILACTIONS_H_
#define INCLUDE_FW_COLL_ENV_FWAVAILACTIONS_H_

#include <fw-coll-env/Utils.h>

#include <vector>
#include <map>
#include <string>

namespace fw_coll_env {

class FwAvailActions {
 public:
  FwAvailActions(
    const std::vector<double> &v,
    const std::vector<double> &w,
    const std::vector<double> &dz);

  const std::vector<FwSingleAction> & get_all_actions() const {
    return all_actions_;
  }

  size_t action_to_idx(const FwSingleAction &ac) const;
  FwSingleAction idx_to_action(size_t idx);
  std::string to_string() const {return repr_;}

  const std::vector<double> &get_v() const {return v_;}
  const std::vector<double> &get_w_deg_per_sec() const {return w_deg_per_sec_;}
  const std::vector<double> &get_w_rad_per_sec() const {return w_rad_per_sec_;}
  const std::vector<double> &get_dz() const {return dz_;}

 protected:
  std::vector<double> v_, w_deg_per_sec_, w_rad_per_sec_, dz_;

  std::vector<FwSingleAction> all_actions_;
  std::map<FwSingleAction, size_t> action_to_idx_;

  std::string repr_;
};

} // namespace fw_coll_env
#endif // INCLUDE_FW_COLL_ENV_FWAVAILACTIONS_H_
