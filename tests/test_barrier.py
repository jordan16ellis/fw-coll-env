import pickle
from typing import Tuple

import numpy as np

import fw_coll_env_c
from fw_coll_env_c import FwAvailActions, BarrierGammaTurn, FwSingleState, \
    FwState, Point, FwSingleAction, FwAction, FwCollisionEnv


DT = 0.1
V = 15
W = 12
MAX_VAL = 300
SAFETY_DIST = 5
GOAL1 = Point(200, 0, 0)
GOAL2 = Point(-200, 0, 0)


def make_barrier_func() -> Tuple[FwAvailActions, BarrierGammaTurn]:
    avail_v = [15, 20, 25]
    avail_w = [-W, 0, W]
    avail_dz = [0]
    avail = FwAvailActions(v=avail_v, w=avail_w, dz=avail_dz)
    bf = BarrierGammaTurn(
        dt=DT, max_val=MAX_VAL, v=V, w_deg_per_sec=W, safety_dist=SAFETY_DIST,
        avail_actions=avail)
    return avail, bf


def test_bf_max_val() -> None:

    avail, bf = make_barrier_func()
    action_index = fw_coll_env_c.FwActionIndex(avail)

    x1 = FwSingleState(Point(5000, 0, 0), 0)
    x2 = FwSingleState(Point(-5000, 0, 0), 0)
    x = FwState(x1, x2)
    assert bf.calc_h(x) == MAX_VAL

    for ac1 in avail.get_all_actions():
        for ac2 in avail.get_all_actions():
            ac = FwAction(ac1, ac2)
            assert bf.calc_dh(x, ac) == 0

            ac_idx = np.array(
                [action_index.action_to_idx(ac)])
            x_npy = np.asarray(x)[np.newaxis, :]
            assert bf.choose_u(x_npy, ac_idx) == ac_idx


def calc_bf_constraint(dh: float, h: float) -> float:
    lmbda = 0.99
    return dh + lmbda * h  # type: ignore


def test_bf_override() -> None:
    avail, bf = make_barrier_func()

    x1 = FwSingleState(Point(21, -1, 0), np.pi)
    x2 = FwSingleState(Point(-21, 1, 0), 0)
    x = FwState(x1, x2)

    h = bf.calc_h(x)

    assert h < MAX_VAL

    correct_single_ac = FwSingleAction(V, W, 0)
    correct_ac = FwAction(correct_single_ac, correct_single_ac)

    max_evasive_bf_constraint = \
        calc_bf_constraint(bf.calc_dh(x, correct_ac), bf.calc_h(x))
    assert max_evasive_bf_constraint >= 0
    action_index = fw_coll_env_c.FwActionIndex(avail)

    for ac1 in avail.get_all_actions():
        for ac2 in avail.get_all_actions():
            ac = FwAction(ac1, ac2)
            bf_constraint = calc_bf_constraint(
                bf.calc_dh(x, ac), bf.calc_h(x))
            assert max_evasive_bf_constraint >= bf_constraint

            x_npy = np.asarray(x)[np.newaxis, :]
            ac_idx = np.array(
                [action_index.action_to_idx(ac)])
            safe_u_idx = bf.choose_u(x_npy, ac_idx)
            safe_u = action_index.idx_to_action(safe_u_idx)

            assert calc_bf_constraint(
                bf.calc_dh(x, safe_u), bf.calc_h(x)) >= 0


def _new_state() -> FwSingleState:
    point = Point(
        *np.random.uniform(low=(-200, -200, -0), high=(200, 200, 0)))
    return FwSingleState(point, th=np.random.uniform(-np.pi, np.pi))


def test_pred_dh() -> None:
    avail, bf = make_barrier_func()
    all_actions = avail.get_all_actions()

    num_scenarios = 100

    env = FwCollisionEnv(
        dt=DT, max_sim_time=10, done_dist=10,
        safety_dist=SAFETY_DIST, goal1=GOAL1, goal2=GOAL2,
        time_warp=-1)

    for _ in range(num_scenarios):
        x = FwState(_new_state(), _new_state())
        ac1 = all_actions[np.random.randint(len(all_actions))]
        ac2 = all_actions[np.random.randint(len(all_actions))]
        ac = FwAction(ac1, ac2)

        pred_dh = bf.calc_dh(x, ac)

        h = bf.calc_h(x)

        env.reset(x.x1, x.x2, 0.0)
        env.step(ac.a1, ac.a2)

        x1 = FwState(x1=env.x1, x2=env.x2)
        h1 = bf.calc_h(x1)

        assert pred_dh == h1 - h


def test_calc_h() -> None:
    bf = make_barrier_func()[1]
    x1 = FwSingleState(Point(-50, 0, 0), th=0)
    x2 = FwSingleState(Point(50, 0, 0), th=np.pi)
    x = FwState(x1, x2)
    h = bf.calc_h(x)
    assert h < x1.p.dist(x2.p)


def test_barrier_gamma_pickle() -> None:
    bf = make_barrier_func()[1]
    bf_unpickle = pickle.loads(pickle.dumps(bf))

    assert bf.dt == bf_unpickle.dt
    assert bf.max_val == bf_unpickle.max_val
    assert bf.v == bf_unpickle.v
    assert bf.w_rad_per_sec == bf_unpickle.w_rad_per_sec
    assert bf.safety_dist == bf_unpickle.safety_dist


def test_fw_action_index() -> None:
    avail = FwAvailActions([20], [-W, 0, W], [0])
    fw_action_idx = fw_coll_env_c.FwActionIndex(avail)
    actions = avail.get_all_actions()

    for ac1 in actions:
        for ac2 in actions:
            ac = fw_coll_env_c.FwAction(ac1, ac2)
            idx = fw_action_idx.action_to_idx(ac)
            assert ac == fw_action_idx.idx_to_action(idx)

    for idx in range(len(actions)**2):
        ac = fw_action_idx.idx_to_action(idx)
        assert idx == fw_action_idx.action_to_idx(ac)
