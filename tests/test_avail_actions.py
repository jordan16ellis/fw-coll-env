import pickle
from typing import List

import fw_coll_env_c


def test_avail_actions() -> None:
    avail_v = [-1, 0, 1]
    avail_w = [-1, 0, 1]
    avail_dz = [-1, 0, 1]
    avail = fw_coll_env_c.FwAvailActions(v=avail_v, w=avail_w, dz=avail_dz)

    ac = avail.idx_to_action(0)
    assert ac.v == -1
    assert ac.w != -1, "should convert to radians"
    assert ac.dz == -1

    num = len(avail_v) * len(avail_w) * len(avail_dz)
    assert len(avail.get_all_actions()) == num

    all_actions: List[fw_coll_env_c.FwAction] = []
    for i in range(num):
        ac = avail.idx_to_action(i)
        assert avail.action_to_idx(ac) == i

        for prev_ac in all_actions:
            assert ac != prev_ac

        all_actions.append(ac)

    # ensure there are no duplicates
    assert len(all_actions) == num


def test_avail_actions_pickle() -> None:
    avail_v = [0, 1, 2]
    avail_w = [3, 4, 5]
    avail_dz = [6, 7, 8]
    avail = fw_coll_env_c.FwAvailActions(v=avail_v, w=avail_w, dz=avail_dz)

    avail_unpickled = pickle.loads(pickle.dumps(avail))

    assert avail_unpickled.v == avail_v
    assert avail_unpickled.w_deg_per_sec == avail_w
    assert avail_unpickled.dz == avail_dz
