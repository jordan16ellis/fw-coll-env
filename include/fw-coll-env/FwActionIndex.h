
#ifndef INCLUDE_FW_COLL_ENV_FWACTIONINDEX_H_
#define INCLUDE_FW_COLL_ENV_FWACTIONINDEX_H_

#include <fw-coll-env/FwAvailActions.h>

namespace fw_coll_env {

class FwActionIndex {
 public:
  explicit FwActionIndex(const FwAvailActions &avail_actions);
  int action_to_idx(const FwAction &ac);
  FwAction idx_to_action(int idx);

 protected:
  FwAvailActions avail_actions_;
  int ac_per_veh_;
};

} // namespace fw_coll_env
#endif // INCLUDE_FW_COLL_ENV_FWACTIONINDEX_H_
