#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

#include <fw-coll-env/BarrierGammaTurn.h>
#include <fw-coll-env/BarrierGammaStraight.h>
#include <fw-coll-env/FwActionIndex.h>
#include <fw-coll-env/FwAvailActions.h>
#include <fw-coll-env/FwCollisionEnv.h>
#include <fw-coll-env/Uhat.h>
#include <fw-coll-env/Utils.h>

namespace py = pybind11;

PYBIND11_MODULE(fw_coll_env_c, m) {
  m.def("fw_dynamics", &fw_coll_env::fw_dynamics,
        py::arg("dt"), py::arg("ac"), py::arg("x"));

  using Pt = fw_coll_env::Point;
  using FwSngSt = fw_coll_env::FwSingleState;
  using FwSt = fw_coll_env::FwState;
  using FwSngAc = fw_coll_env::FwSingleAction;
  using FwAc = fw_coll_env::FwAction;
  using BFTurn = fw_coll_env::BarrierGammaTurn;
  using BFStraight = fw_coll_env::BarrierGammaStraight;
  using FwEnv = fw_coll_env::FwCollisionEnv;

  py::class_<Pt>(m, "Point")
    .def(py::init<double, double, double>(),
         py::arg("x"), py::arg("y"), py::arg("z"))
    .def("dist", &Pt::dist)
    .def("__repr__", &Pt::to_string)
    .def("__eq__", &Pt::operator==)
    .def_static("from_numpy",
        [&](py::array_t<double> arr) {
          return Pt(arr.at(0), arr.at(1), arr.at(2));
        })
    .def("__array__",
      [&](const Pt &p) {
         py::array_t<double> out{3};
         out.mutable_at(0) = p.x;
         out.mutable_at(1) = p.y;
         out.mutable_at(2) = p.z;
         return out;
      })
    .def(py::pickle(
        [](const Pt &p) {return py::make_tuple(p.x, p.y, p.z);},
        [](py::tuple t) { // __setstate__
            if (t.size() != 3) {
                throw std::runtime_error("Invalid tuple provided for fw_coll_env::Point!");
            }
            return Pt(t[0].cast<double>(), t[1].cast<double>(), t[2].cast<double>());
        }))
    .def("__copy__", [](const Pt &p){return Pt(p);})
    .def("__deepcopy__", [](const Pt &p, py::dict){return Pt(p);})
    .def_readwrite("x", &Pt::x)
    .def_readwrite("y", &Pt::y)
    .def_readwrite("z", &Pt::z);

  py::class_<FwSngSt>(m, "FwSingleState")
    .def(py::init<const Pt&, double>(), py::arg("p"), py::arg("th"))
    .def("__repr__", &FwSngSt::to_string)
    .def("__eq__", &FwSngSt::operator==)
    .def("__copy__", [](const FwSngSt &x){return FwSngSt(x);})
    .def("__deepcopy__", [](const FwSngSt &x, py::dict){return FwSngSt(x);})
    .def(py::pickle(
        [](const FwSngSt &x) {return py::make_tuple(x.p, x.th);},
        [](py::tuple t) { // __setstate__
            if (t.size() != 2) {
                throw std::runtime_error("Invalid tuple provided for fw_coll_env::FwSingleState!");
            }
            return FwSngSt(t[0].cast<Pt>(), t[1].cast<double>());
        }))
    .def_static("from_numpy",
        [&](py::array_t<double> arr) {
          return FwSngSt(Pt(arr.at(0), arr.at(1), arr.at(3)), arr.at(2));
        })
    .def("__array__",
      [&](const FwSngSt &x) {
        py::array_t<double> out{4};
        out.mutable_at(0) = x.p.x;
        out.mutable_at(1) = x.p.y;
        out.mutable_at(2) = x.th;
        out.mutable_at(3) = x.p.z;
        return out;
      })
    .def_readwrite("p", &FwSngSt::p)
    .def_readwrite("th", &FwSngSt::th);

  py::class_<FwSt>(m, "FwState")
    .def(py::init<const FwSngSt&, const FwSngSt&>(),
         py::arg("x1"), py::arg("x2"))
    .def("__repr__", &FwSt::to_string)
    .def("__copy__", [](const FwSt &x){return FwSt(x);})
    .def("__deepcopy__", [](const FwSt &x, py::dict){return FwSt(x);})
    .def_static("from_numpy",
        [&](py::array_t<double> arr) {
          return FwSt(
              FwSngSt(Pt(arr.at(0), arr.at(1), arr.at(3)), arr.at(2)),
              FwSngSt(Pt(arr.at(4), arr.at(5), arr.at(7)), arr.at(6)));
        })
    .def(py::pickle(
        [](const FwSt &x) {return py::make_tuple(x.x1, x.x2);},
        [](py::tuple t) { // __setstate__
            if (t.size() != 2) {
                throw std::runtime_error("Invalid tuple provided for fw_coll_env::FwState!");
            }
            return FwSt(t[0].cast<FwSngSt>(), t[1].cast<FwSngSt>());
        }))
    .def("__array__", &FwSt::asarray)
    .def_readwrite("x1", &FwSt::x1)
    .def_readwrite("x2", &FwSt::x2);

  py::class_<FwSngAc>(m, "FwSingleAction")
    .def(py::init<double, double, double>(),
         py::arg("v"), py::arg("w"), py::arg("dz"))
    .def("dist", &FwSngAc::dist)
    .def("__repr__", &FwSngAc::to_string)
    .def("__eq__", &FwSngAc::operator==)
    .def("__copy__", [](const FwSngAc &x){return FwSngAc(x);})
    .def("__deepcopy__", [](const FwSngAc &x, py::dict){return FwSngAc(x);})
    .def(py::pickle(
        [](const FwSngAc &a) {return py::make_tuple(a.v, a.w, a.dz);},
        [](py::tuple t) { // __setstate__
            if (t.size() != 3) {
                throw std::runtime_error("Invalid tuple provided for fw_coll_env::FwSingleAction!");
            }
            return FwSngAc(t[0].cast<double>(), t[1].cast<double>(), t[2].cast<double>());
        }))
    .def_readwrite("v", &FwSngAc::v)
    .def_readwrite("w", &FwSngAc::w)
    .def_readwrite("dz", &FwSngAc::dz);

  py::class_<FwAc>(m, "FwAction")
    .def(py::init<const FwSngAc&, const FwSngAc&>(),
         py::arg("a1"), py::arg("a2"))
    .def("__repr__", &FwAc::to_string)
    .def("__eq__", &FwAc::operator==)
    .def("__copy__", [](const FwAc &x){return FwAc(x);})
    .def("__deepcopy__", [](const FwAc &x, py::dict){return FwAc(x);})
    .def(py::pickle(
        [](const FwAc &a) {return py::make_tuple(a.a1, a.a2);},
        [](py::tuple t) { // __setstate__
            if (t.size() != 2) {
                throw std::runtime_error("Invalid tuple provided for fw_coll_env::FwAction!");
            }
            return FwAc(t[0].cast<FwSngAc>(), t[1].cast<FwSngAc>());
        }))
    .def("dist", &FwAc::dist)
    .def_readwrite("a1", &FwAc::a1)
    .def_readwrite("a2", &FwAc::a2);

  py::class_<fw_coll_env::Rho>(m, "Rho")
    .def(py::init<double, double>(), py::arg("safety_dist"), py::arg("max_val"))
    .def("__repr__", &fw_coll_env::Rho::to_string)
    .def("__call__", &fw_coll_env::Rho::operator())
    .def(py::pickle(
        [](const fw_coll_env::Rho &rho) {return py::make_tuple(rho.get_safety_dist(), rho.get_max_val());},
        [](py::tuple t) { // __setstate__
            if (t.size() != 2) {
                throw std::runtime_error("Invalid tuple provided for fw_coll_env::FwAction!");
            }
            return fw_coll_env::Rho(t[0].cast<double>(), t[1].cast<double>());
        }))
    .def_property_readonly("safety_dist", &fw_coll_env::Rho::get_safety_dist)
    .def_property_readonly("max_val", &fw_coll_env::Rho::get_max_val);

  py::class_<fw_coll_env::FwAvailActions>(m, "FwAvailActions")
    .def(py::init<const std::vector<double>&,
                  const std::vector<double>&,
                  const std::vector<double>&>(),
         py::arg("v"), py::arg("w"), py::arg("dz"))
    .def("__repr__", &fw_coll_env::FwAvailActions::to_string)
    .def(py::pickle(
        [](const fw_coll_env::FwAvailActions &a) {return py::make_tuple(a.get_v(), a.get_w_deg_per_sec(), a.get_dz());},
        [](py::tuple t) { // __setstate__
            if (t.size() != 3) {
                throw std::runtime_error("Invalid tuple provided for fw_coll_env::FwAvailActions!");
            }
            return fw_coll_env::FwAvailActions(
                t[0].cast<std::vector<double>>(),
                t[1].cast<std::vector<double>>(),
                t[2].cast<std::vector<double>>());
        }))
    .def_property_readonly("v", &fw_coll_env::FwAvailActions::get_v)
    .def_property_readonly("w_rad_per_sec", &fw_coll_env::FwAvailActions::get_w_rad_per_sec)
    .def_property_readonly("w_deg_per_sec", &fw_coll_env::FwAvailActions::get_w_deg_per_sec)
    .def_property_readonly("dz", &fw_coll_env::FwAvailActions::get_dz)
    .def("get_all_actions", &fw_coll_env::FwAvailActions::get_all_actions)
    .def("action_to_idx", &fw_coll_env::FwAvailActions::action_to_idx)
    .def("idx_to_action", &fw_coll_env::FwAvailActions::idx_to_action);

  py::class_<fw_coll_env::Uhat>(m, "Uhat")
    .def(py::init<const Pt &, double, const fw_coll_env::FwAvailActions&>(),
         py::arg("goal"), py::arg("dt"), py::arg("avail_actions"))
    .def(py::pickle(
        [](const fw_coll_env::Uhat &u) {return py::make_tuple(u.get_goal(), u.get_dt(), u.get_fw_avail_actions());},
        [](py::tuple t) { // __setstate__
            if (t.size() != 3) {
                throw std::runtime_error("Invalid tuple provided for fw_coll_env::Uhat!");
            }
            return fw_coll_env::Uhat(t[0].cast<Pt>(), t[1].cast<double>(), t[2].cast<fw_coll_env::FwAvailActions>());
        }))
    .def_property("goal", &fw_coll_env::Uhat::get_goal, &fw_coll_env::Uhat::set_goal)
    .def("calc", &fw_coll_env::Uhat::calc);

  py::class_<fw_coll_env::FwEnvStats>(m, "FwEnvStats")
    .def("__repr__", &fw_coll_env::FwEnvStats::to_string)
    .def_readonly("done_time", &fw_coll_env::FwEnvStats::done_time)
    .def_readonly("done_goal", &fw_coll_env::FwEnvStats::done_goal)
    .def_readonly("done_collision", &fw_coll_env::FwEnvStats::done_collision)
    .def_readonly("dist_to_goal1", &fw_coll_env::FwEnvStats::dist_to_goal1)
    .def_readonly("dist_to_goal2", &fw_coll_env::FwEnvStats::dist_to_goal2)
    .def_readonly("dist_to_veh", &fw_coll_env::FwEnvStats::dist_to_veh);

  py::class_<FwEnv>(m, "FwCollisionEnv")
    .def(py::init<double, double, double, double,
                  const Pt&, const Pt&, double>(),
         py::arg("dt"), py::arg("max_sim_time"), py::arg("done_dist"),
         py::arg("safety_dist"), py::arg("goal1"), py::arg("goal2"),
         py::arg("time_warp"))
    .def("__copy__", [](const FwEnv &e){return FwEnv(e);})
    .def("__deepcopy__", [](const FwEnv &e, py::dict){return FwEnv(e);})
    .def(py::pickle(
        [](const FwEnv &e) {return py::make_tuple(
          e.get_dt(), e.get_max_sim_time(), e.get_done_dist(), e.get_safety_dist(),
          e.get_goal1(), e.get_goal2(), e.get_time_warp(),
          e.get_x1(), e.get_x2(), e.get_t());},
        [](py::tuple t) { // __setstate__
            if (t.size() != 10) {
                throw std::runtime_error("Invalid tuple provided for FwEnv!");
            }
            FwEnv e = FwEnv(
                t[0].cast<double>(), t[1].cast<double>(),
                t[2].cast<double>(), t[3].cast<double>(),
                t[4].cast<Pt>(), t[5].cast<Pt>(), t[6].cast<double>());
            e.reset(t[7].cast<FwSngSt>(), t[8].cast<FwSngSt>(), t[9].cast<double>());
            return e;
        }))
    .def("step", &FwEnv::step)
    .def("reset", &FwEnv::reset)
    .def_property_readonly("x1", &FwEnv::get_x1)
    .def_property_readonly("x2", &FwEnv::get_x2)
    .def_property_readonly("t", &FwEnv::get_t)
    .def_property_readonly("dt", &FwEnv::get_dt)
    .def_property_readonly("safety_dist", &FwEnv::get_safety_dist)
    .def_property_readonly("goal1", &FwEnv::get_goal1)
    .def_property_readonly("goal2", &FwEnv::get_goal2)
    .def_property_readonly("done", &FwEnv::get_done)
    .def_property_readonly("done_dist", &FwEnv::get_done_dist)
    .def_property_readonly("max_sim_time", &FwEnv::get_max_sim_time)
    .def_property_readonly("time_warp", &FwEnv::get_time_warp)
    .def_property_readonly("collided", &FwEnv::get_collided)
    .def_readonly("stats", &FwEnv::stats);

  py::class_<BFTurn>(m, "BarrierGammaTurn")
    .def(py::init<double, double, double,
                  double, double, const fw_coll_env::FwAvailActions&>(),
         py::arg("dt"), py::arg("max_val"), py::arg("v"),
         py::arg("w_deg_per_sec"), py::arg("safety_dist"), py::arg("avail_actions"))
    .def("__copy__", [](const BFTurn &b){return BFTurn(b);})
    .def("__deepcopy__", [](const BFTurn &b, py::dict){return BFTurn(b);})
    .def(py::pickle(
        [](const BFTurn &b) {return py::make_tuple(
          b.get_dt(), b.get_max_val(), b.get_v(), fw_coll_env::rad2deg(b.get_w_rad_per_sec()),
          b.get_safety_dist(), b.get_avail_actions());},
        [](py::tuple t) { // __setstate__
            if (t.size() != 6) {
                throw std::runtime_error("Invalid tuple provided for BarrierGammaTurn!");
            }
            BFTurn b = BFTurn(
                t[0].cast<double>(), t[1].cast<double>(),
                t[2].cast<double>(), t[3].cast<double>(),
                t[4].cast<double>(), t[5].cast<fw_coll_env::FwAvailActions>());
            return b;
        }))
    .def("__repr__", &BFTurn::to_string)
    .def("calc_h", &BFTurn::calc_h)
    .def("calc_dh", &BFTurn::calc_dh)
    .def("choose_u", &BFTurn::choose_u)
    .def_property_readonly("dt", &BFTurn::get_dt)
    .def_property_readonly("max_val", &BFTurn::get_max_val)
    .def_property_readonly("v", &BFTurn::get_v)
    .def_property_readonly("w_rad_per_sec", &BFTurn::get_w_rad_per_sec)
    .def_property_readonly("safety_dist", &BFTurn::get_safety_dist)
    .def_property_readonly("avail_actions", &BFTurn::get_avail_actions);

  py::class_<BFStraight, BFTurn>(m, "BarrierGammaStraight")
    .def(py::init<double, double, double,
                  double, const fw_coll_env::FwAvailActions&>(),
         py::arg("dt"), py::arg("max_val"), py::arg("v"),
         py::arg("safety_dist"), py::arg("avail_actions"))
    .def("__copy__", [](const BFStraight &b){return BFStraight(b);})
    .def("__deepcopy__", [](const BFStraight &b, py::dict){return BFStraight(b);})
    .def(py::pickle(
        [](const BFStraight &b) {return py::make_tuple(
          b.get_dt(), b.get_max_val(), b.get_v(),
          b.get_safety_dist(), b.get_avail_actions());},
        [](py::tuple t) { // __setstate__
            if (t.size() != 5) {
                throw std::runtime_error("Invalid tuple provided for BarrierGammaTurn!");
            }
            BFStraight b = BFStraight(
                t[0].cast<double>(), t[1].cast<double>(),
                t[2].cast<double>(), t[3].cast<double>(),
                t[4].cast<fw_coll_env::FwAvailActions>());
            return b;
        }))
    .def("__repr__", &BFStraight::to_string)
    .def("calc_h", &BFStraight::calc_h)
    .def("calc_dh", &BFStraight::calc_dh)
    .def("choose_u", &BFStraight::choose_u)
    .def_property_readonly("dt", &BFStraight::get_dt)
    .def_property_readonly("max_val", &BFStraight::get_max_val)
    .def_property_readonly("v", &BFStraight::get_v)
    .def_property_readonly("w_rad_per_sec", &BFStraight::get_w_rad_per_sec)
    .def_property_readonly("safety_dist", &BFStraight::get_safety_dist)
    .def_property_readonly("avail_actions", &BFStraight::get_avail_actions);

  py::class_<fw_coll_env::FwActionIndex>(m, "FwActionIndex")
    .def(py::init<fw_coll_env::FwAvailActions&>(), py::arg("avail_actions"))
    .def("idx_to_action", &fw_coll_env::FwActionIndex::idx_to_action)
    .def("action_to_idx", &fw_coll_env::FwActionIndex::action_to_idx);

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
