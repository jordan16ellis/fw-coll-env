
#ifndef INCLUDE_FW_COLL_ENV_BARRIERGAMMATURN_H_
#define INCLUDE_FW_COLL_ENV_BARRIERGAMMATURN_H_

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include <fw-coll-env/FwAvailActions.h>
#include <fw-coll-env/FwActionIndex.h>

#include <string>
#include <vector>

namespace fw_coll_env {

class BarrierGammaTurn {
 public:
  BarrierGammaTurn(
      double dt, double max_val, double v, double w_deg_per_sec, double safety_dist,
      const FwAvailActions &avail_actions);

  double calc_h(const FwState &x0);
  double calc_dh(const FwState &x0, const FwAction &ac);
  pybind11::array_t<int> choose_u(
      pybind11::array_t<double> x, pybind11::array_t<int> uhat_idx);

  virtual std::string to_string() const;

  double get_dt() const {return dt_;}
  double get_max_val() const {return max_val_;}
  double get_v() const {return v_;}
  double get_w_rad_per_sec() const {return w_rad_per_sec_;}
  double get_safety_dist() const {return safety_dist_;}
  const FwAvailActions get_avail_actions() const {return avail_actions_;}

 protected:
  size_t steps_per_revolution() const;
  virtual double closest_future_dist(const FwState &x);
  double bf_constraint(double h, const FwState &x0, const FwAction &_ac);
  FwAction choose_u_single(const FwState &x0, const FwAction &uhat);

  double dt_;
  double max_val_;
  double v_;
  double w_rad_per_sec_;
  double safety_dist_;
  FwAvailActions avail_actions_;
  FwActionIndex action_index_;

  const double lambda_ = 0.99;
};

} // namespace fw_coll_env
#endif // INCLUDE_FW_COLL_ENV_BARRIERGAMMATURN_H_
