#ifndef INCLUDE_FW_COLL_ENV_BARRIERGAMMASTRAIGHT_H_
#define INCLUDE_FW_COLL_ENV_BARRIERGAMMASTRAIGHT_H_

#include <fw-coll-env/FwAvailActions.h>
#include <fw-coll-env/BarrierGammaTurn.h>

#include <string>
#include <vector>
#include <tuple>

namespace fw_coll_env {

class BarrierGammaStraight : public BarrierGammaTurn {
 public:
  BarrierGammaStraight(
      double dt, double max_val, double v, double safety_dist,
      const FwAvailActions &avail_actions);

  std::string to_string() const override;

 protected:
  double closest_future_dist(const FwState &x) override;
};

} // namespace fw_coll_env
#endif // INCLUDE_FW_COLL_ENV_BARRIERGAMMASTRAIGHT_H_
