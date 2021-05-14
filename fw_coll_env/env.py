from typing import Tuple, Optional

import numpy as np
import gym
import fw_coll_env_c

from .viewer import Viewer


class FwCollisionGymEnv(gym.Env):
    def __init__(
            self,
            env: fw_coll_env_c.FwCollisionEnv,
            veh1_reset_lims: np.ndarray,
            veh2_reset_lims: np.ndarray,
            goal1_reset_lims: np.ndarray,
            goal2_reset_lims: np.ndarray,
            avail_actions: fw_coll_env_c.FwAvailActions):
        self.env = env
        self.veh1_reset_lims = veh1_reset_lims
        self.veh2_reset_lims = veh2_reset_lims
        self.goal1_reset_lims = goal1_reset_lims
        self.goal2_reset_lims = goal2_reset_lims
        self.uhat = \
            fw_coll_env_c.Uhat(self.env.goal2, self.env.dt, avail_actions)
        self.avail_actions = avail_actions
        self.action_index = fw_coll_env_c.FwActionIndex(avail_actions)
        self.viewer: Optional[Viewer] = None

        self.action_space = gym.spaces.MultiDiscrete(
            [len(avail_actions.v), len(avail_actions.w_deg_per_sec),
             len(avail_actions.dz)])

        n = 11
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (n,))

    def step(self, action: np.ndarray) \
            -> Tuple[np.ndarray, float, bool, dict]:

        v = self.avail_actions.v[action[0]]
        w = self.avail_actions.w_rad_per_sec[action[1]]
        dz = self.avail_actions.dz[action[1]]

        ac1 = fw_coll_env_c.FwSingleAction(v, w, dz)
        ac2 = self.uhat.calc(self.state.x2)

        self.env.step(ac1, ac2)

        obs = self._get_obs()
        reward = float(
            self.env.stats.dist_to_goal1 < self.env.stats.dist_to_goal1)

        return obs, reward, self.env.done, {}

    def _get_obs(self) -> np.ndarray:
        self.state = fw_coll_env_c.FwState(self.env.x1, self.env.x2)

        veh1_pos = self._normalize_pos(self.env.x1.p, self.veh1_reset_lims)
        veh2_pos = self._normalize_pos(self.env.x2.p, self.veh2_reset_lims)
        goal1_nrm = self._normalize_pos(self.env.goal1, self.veh1_reset_lims)
        goal2_nrm = self._normalize_pos(self.env.goal2, self.veh2_reset_lims)

        th1 = self.env.x1.th
        th2 = self.env.x2.th
        obs = \
            veh1_pos.tolist() + [np.sin(th1), np.sin(th1)] + \
            veh2_pos.tolist() + [np.sin(th2), np.sin(th1)] + \
            goal1_nrm.tolist() + goal2_nrm.tolist()

        return obs

    def _normalize_pos(
            self, p: fw_coll_env_c.Point, lims: np.ndarray) -> np.ndarray:
        low_pos_lims = lims[0][[0, 1, 3]]
        high_pos_lims = lims[1][[0, 1, 3]]
        return (np.asarray(p) - low_pos_lims) / \
            np.maximum((high_pos_lims - low_pos_lims), 0.1)

    def reset(self) -> None:

        def _sample(_lims: np.ndarray) -> np.ndarray:
            return np.random.uniform(_lims[0], _lims[1])

        veh1_pose = _sample(self.veh1_reset_lims)
        veh2_pose = _sample(self.veh2_reset_lims)
        self.state = np.hstack((veh1_pose, veh2_pose))

        self.goal1 = _sample(self.goal1_reset_lims)
        self.goal2 = _sample(self.goal2_reset_lims)

        self.uhat.goal = fw_coll_env_c.Point.from_numpy(self.goal2)

        self.env.reset(fw_coll_env_c.FwSingleState.from_numpy(veh1_pose),
                       fw_coll_env_c.FwSingleState.from_numpy(veh2_pose), 0.0)

        return self._get_obs()

    def render(self) -> None:
        if self.viewer is None:
            self.viewer = Viewer(self.env)

        assert self.viewer is not None
        self.viewer.render(self.env)

    def close(self) -> None:
        if self.viewer is not None:
            self.viewer.viewer.close()
            self.viewer = None
