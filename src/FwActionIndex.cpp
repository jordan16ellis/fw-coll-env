#include <fw-coll-env/FwActionIndex.h>

namespace fw_coll_env {

FwActionIndex::FwActionIndex(const FwAvailActions &avail_actions) :
    avail_actions_(avail_actions),
    ac_per_veh_(avail_actions.get_all_actions().size()) {}

int FwActionIndex::action_to_idx(const FwAction &ac) {
  int ac1_idx = avail_actions_.action_to_idx(ac.a1);
  int ac2_idx = avail_actions_.action_to_idx(ac.a2);
  return ac1_idx * ac_per_veh_ + ac2_idx;
}

FwAction FwActionIndex::idx_to_action(int idx) {
  auto[ac1_idx, ac2_idx] = std::div(idx, ac_per_veh_);
  FwSingleAction ac1 = avail_actions_.idx_to_action(ac1_idx);
  FwSingleAction ac2 = avail_actions_.idx_to_action(ac2_idx);
  return {ac1, ac2};
}

} // namespace fw_coll_env
