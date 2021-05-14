import pickle
import numpy as np

import fw_coll_env_c


def test_uhat() -> None:
    avail_v = [1, 2, 3]
    avail_w = [-1, 0, 1]
    avail_dz = [-1, 0, 1]
    avail = fw_coll_env_c.FwAvailActions(v=avail_v, w=avail_w, dz=avail_dz)

    uhat = fw_coll_env_c.Uhat(
        goal=fw_coll_env_c.Point(x=200, y=0, z=0),
        dt=0.01, avail_actions=avail)

    x0 = fw_coll_env_c.FwSingleState(
        p=fw_coll_env_c.Point(0, 0, 0), th=0)

    # pointing to the left of goal, should turn right
    x0.th = np.pi / 2
    ac = uhat.calc(x0)
    assert ac.w < 0
    assert ac.dz == 0

    # pointing to the right of goal, should turn right
    x0.th = -np.pi / 2
    ac = uhat.calc(x0)
    assert ac.w > 0
    assert ac.dz == 0

    # pickling
    pickle.loads(pickle.dumps(uhat))
