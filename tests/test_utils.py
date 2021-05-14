import copy
import pickle

import numpy as np

from fw_coll_env_c import Point, FwSingleState, FwState, FwSingleAction, \
    FwAction, fw_dynamics
import fw_coll_env_c


def test_point() -> None:
    p1 = Point(1, 2, 3)
    p2 = Point(x=0, y=1, z=2)
    assert np.isclose(p1.dist(p2), np.sqrt(3), atol=1e-9)

    assert p1.x == 1
    assert p1.y == 2
    assert p1.z == 3

    assert p2.x == 0
    assert p2.y == 1
    assert p2.z == 2

    exp_np = np.array([1, 2, 3])
    actual_np = np.asarray(p1)
    assert np.allclose(exp_np, actual_np, atol=1e-9)
    assert np.allclose(p1, Point.from_numpy(exp_np), atol=1e-9)

    p1_unpickle = pickle.loads(pickle.dumps(p1))
    assert p1_unpickle == p1


def test_fw_single_state() -> None:
    x = FwSingleState(p=fw_coll_env_c.Point(1, 2, 3), th=-1)
    assert np.isclose(
        x.p.dist(fw_coll_env_c.Point(0, 1, 2)), np.sqrt(3), atol=1e-9)

    exp_np = np.array([1, 2, -1, 3])
    actual_np = np.asarray(x)
    assert np.allclose(exp_np, actual_np, atol=1e-9)
    assert np.allclose(x, FwSingleState.from_numpy(exp_np), atol=1e-9)
    assert x.th == -1

    x_unpickle = pickle.loads(pickle.dumps(x))
    assert x_unpickle == x


def test_fw_state() -> None:
    x1 = FwSingleState(p=fw_coll_env_c.Point(0, 1, 2), th=0)
    x2 = FwSingleState(p=fw_coll_env_c.Point(0, 1, 2), th=0)
    x = FwState(x1=x1, x2=x2)

    assert np.isclose(x.x1.p.dist(fw_coll_env_c.Point(0, 1, 2)), 0, atol=1e-9)
    assert np.isclose(x.x2.p.dist(fw_coll_env_c.Point(0, 1, 2)), 0, atol=1e-9)

    exp_np = np.array([0, 1, 0, 2, 0, 1, 0, 2])
    actual_np = np.asarray(x)
    assert np.allclose(exp_np, actual_np, atol=1e-9)
    assert np.allclose(x, FwState.from_numpy(exp_np), atol=1e-9)


def test_fw_action() -> None:
    single_ac = FwSingleAction(v=1, w=2, dz=3)
    assert single_ac.v == 1
    assert single_ac.w == 2
    assert single_ac.dz == 3
    assert single_ac.dist(single_ac) == 0

    assert single_ac == pickle.loads(pickle.dumps(single_ac))

    ac = FwAction(single_ac, single_ac)
    assert ac.a1 == single_ac
    assert ac.a2 == single_ac

    assert ac.dist(ac) == 0
    assert ac == pickle.loads(pickle.dumps(ac))


def check_state(state: FwSingleState,
                x: float, y: float, th: float, z: float) -> None:
    assert np.isclose(state.p.x, x, atol=1e-9)
    assert np.isclose(state.p.y, y, atol=1e-9)
    assert np.isclose(state.p.z, z, atol=1e-9)
    assert np.isclose(state.th, th, atol=1e-9)


def test_fw_dynamics_velocity() -> None:
    ac = FwSingleAction(v=1, w=0, dz=0)
    x = FwSingleState(p=fw_coll_env_c.Point(1, 0, 0), th=0)
    fw_coll_env_c.fw_dynamics(dt=1, ac=ac, x=x)
    check_state(x, 2, 0, 0, 0)

    x = FwSingleState(p=fw_coll_env_c.Point(1, 0, 0), th=np.pi)
    fw_coll_env_c.fw_dynamics(dt=1, ac=ac, x=x)
    check_state(x, 0, 0, np.pi, 0)

    x = FwSingleState(p=fw_coll_env_c.Point(1, 0, 0), th=np.pi / 2)
    fw_coll_env_c.fw_dynamics(dt=1, ac=ac, x=x)
    check_state(x, 1, 1, np.pi / 2, 0)

    x = FwSingleState(p=fw_coll_env_c.Point(1, 0, 0), th=-np.pi / 2)
    fw_coll_env_c.fw_dynamics(dt=1, ac=ac, x=x)
    check_state(x, 1, -1, -np.pi / 2, 0)


def test_fw_dynamics_turn_rate() -> None:
    ac = FwSingleAction(v=0, w=1, dz=0)
    x = FwSingleState(p=fw_coll_env_c.Point(0, 0, 0), th=0)
    fw_coll_env_c.fw_dynamics(dt=1, ac=ac, x=x)
    check_state(x, 0, 0, 1, 0)

    ac = FwSingleAction(v=0, w=-1, dz=0)
    x = FwSingleState(p=fw_coll_env_c.Point(0, 0, 0), th=0)
    fw_coll_env_c.fw_dynamics(dt=1, ac=ac, x=x)
    check_state(x, 0, 0, -1, 0)


def test_fw_dynamics_alt_rate() -> None:
    ac = FwSingleAction(v=0, w=0, dz=1)
    x = FwSingleState(p=fw_coll_env_c.Point(0, 0, 0), th=0)
    fw_coll_env_c.fw_dynamics(dt=1, ac=ac, x=x)
    check_state(x, 0, 0, 0, 1)

    ac = FwSingleAction(v=0, w=0, dz=-1)
    x = FwSingleState(p=fw_coll_env_c.Point(0, 0, 0), th=0)
    fw_coll_env_c.fw_dynamics(dt=1, ac=ac, x=x)
    check_state(x, 0, 0, 0, -1)


def test_rho() -> None:
    x1 = FwSingleState(p=fw_coll_env_c.Point(0, 0, 0), th=0)
    x2 = FwSingleState(p=fw_coll_env_c.Point(5, 0, 0), th=0)
    x = FwState(x1, x2)

    rho = fw_coll_env_c.Rho(safety_dist=5, max_val=25)
    assert rho(x) == 0

    x.x2.p.x = 300
    assert rho(x) == 25

    rho_unpickled = pickle.loads(pickle.dumps(rho))
    assert rho.safety_dist == rho_unpickled.safety_dist
    assert rho.max_val == rho_unpickled.max_val


def test_copy() -> None:
    p = Point(0, 1, 2)
    p2 = copy.deepcopy(p)
    p2.x = 5
    assert p2.x == 5 and p.x == 0

    single_ac1 = FwSingleAction(1, 2, 3)
    single_ac2 = copy.deepcopy(single_ac1)
    single_ac2.v = 5
    assert single_ac2.v == 5 and single_ac1.v == 1

    ac = FwAction(single_ac1, single_ac2)
    ac2 = copy.deepcopy(ac)
    ac2.a2 = single_ac1
    assert ac2.a2 == single_ac1 and ac.a2 == single_ac2

    single_x1 = FwSingleState(p, 0)
    single_x2 = copy.deepcopy(single_x1)
    single_x2.p = p2
    single_x2.th = 1
    assert single_x2.p == p2 and single_x1.p == p and \
        single_x2.th == 1 and single_x1.th == 0

    x3 = copy.deepcopy(single_x1)
    fw_dynamics(1, single_ac1, x3)
    assert single_x1 != x3

    x = FwState(single_x1, single_x2)
    x2 = copy.deepcopy(x)
    x2.x2 = single_x1
    assert x2.x2 == single_x1 and x.x2 == single_x2
