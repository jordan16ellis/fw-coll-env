import pickle
from typing import Tuple

import numpy as np

from fw_coll_env_c import FwCollisionEnv, Point, FwSingleState, \
    FwAvailActions, Uhat

DT = 0.1
DONE_DIST = 75
MAX_SIM_TIME = 100
GOAL1 = Point(200, 0, 0)
GOAL2 = Point(-200, 0, 0)


def make_base_env(safety_dist: float) -> FwCollisionEnv:
    return FwCollisionEnv(
        dt=DT, max_sim_time=MAX_SIM_TIME, done_dist=DONE_DIST,
        safety_dist=safety_dist, goal1=GOAL1, goal2=GOAL2,
        time_warp=-1)


def make_uhat(avail_v: list) -> Tuple[Uhat, Uhat, FwAvailActions]:
    avail_w = [-13, 0, 13]
    avail_dz = [0]
    avail = FwAvailActions(v=avail_v, w=avail_w, dz=avail_dz)
    uhat1 = Uhat(goal=GOAL1, dt=DT, avail_actions=avail)
    uhat2 = Uhat(goal=GOAL2, dt=DT, avail_actions=avail)

    return uhat1, uhat2, avail


def test_state_getter() -> None:
    env = make_base_env(0)
    x1 = FwSingleState(Point(50, 0, 0), 0)
    x2 = FwSingleState(Point(-50, 0, 0), 0)
    env.reset(x1, x2, 0.0)

    assert env.t == 0
    assert env.x1 == x1
    assert env.x2 == x2


def test_pickle() -> None:
    env = make_base_env(safety_dist=-1)
    env.x1.p = Point(1, 2, 3)
    env.x1.th = 4
    env.x2.p = env.x1.p
    env.x2.th = env.x1.th

    orig_x1 = env.x1
    env_pickle = pickle.dumps(env)
    env2 = pickle.loads(env_pickle)
    assert orig_x1 == env2.x1


def test_reaches_goal() -> None:
    env = make_base_env(safety_dist=-1)
    x1 = FwSingleState(Point(0, 50, 0), np.pi / 2)
    x2 = FwSingleState(Point(0, -50, 0), -np.pi / 2)
    env.reset(x1, x2, 0.0)

    uhat1, uhat2 = make_uhat([20])[:2]

    while True:
        a1 = uhat1.calc(env.x1)
        a2 = uhat2.calc(env.x2)
        done = env.step(a1, a2)
        if done:
            assert not env.stats.done_time
            assert env.stats.done_goal
            assert not env.stats.done_collision
            assert env.stats.dist_to_goal1 < DONE_DIST or \
                env.stats.dist_to_goal2 < DONE_DIST
            assert env.stats.dist_to_veh > 100
            break


def test_max_time() -> None:
    env = make_base_env(safety_dist=-1)
    x1 = FwSingleState(Point(0, 50, 0), np.pi / 2)
    x2 = FwSingleState(Point(0, -50, 0), -np.pi / 2)
    env.reset(x1, x2, 0.0)

    uhat1, uhat2 = make_uhat([0])[:2]

    while True:
        a1 = uhat1.calc(env.x1)
        a2 = uhat2.calc(env.x2)
        done = env.step(a1, a2)
        if done:
            assert env.stats.done_time
            assert not env.stats.done_goal
            assert not env.stats.done_collision
            break


def test_collision() -> None:
    env = make_base_env(safety_dist=5)
    x1 = FwSingleState(Point(0, 1, 0), np.pi / 2)
    x2 = FwSingleState(Point(0, -1, 0), -np.pi / 2)
    env.reset(x1, x2, 0.0)

    uhat1, uhat2 = make_uhat([0])[:2]
    a1 = uhat1.calc(env.x1)
    a2 = uhat2.calc(env.x2)
    env.step(a1, a2)
    assert env.stats.done_collision
    assert env.stats.dist_to_veh == 2
