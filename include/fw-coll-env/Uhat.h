
#ifndef INCLUDE_FW_COLL_ENV_UHAT_H_
#define INCLUDE_FW_COLL_ENV_UHAT_H_

#include <fw-coll-env/FwAvailActions.h>
#include <fw-coll-env/Utils.h>

#include <vector>

namespace fw_coll_env {

class Uhat {
 public:
  Uhat(const Point &goal, double dt, const FwAvailActions &avail_actions) :
    goal_(goal), dt_(dt), avail_actions_(avail_actions) {}

  FwSingleAction calc(const FwSingleState &x0);

  const Point &set_goal(const fw_coll_env::Point &goal) {goal_ = goal;}
  const Point &get_goal() const {return goal_;}
  double get_dt() const {return dt_;}
  const FwAvailActions &get_fw_avail_actions() const {return avail_actions_;}

 protected:
  Point goal_;
  double dt_;
  FwAvailActions avail_actions_;
};

} // namespace fw_coll_env
#endif // INCLUDE_FW_COLL_ENV_UHAT_H_
