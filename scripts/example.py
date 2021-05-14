import numpy as np
import fw_coll_env
import fw_coll_env_c


def main():
    env_c = fw_coll_env_c.FwCollisionEnv(
        dt=0.1, max_sim_time=30, done_dist=25, safety_dist=25,
        goal1=fw_coll_env_c.Point(200, 0, 0),
        goal2=fw_coll_env_c.Point(-200, 0, 0),
        time_warp=-1)

    avail_actions = fw_coll_env_c.FwAvailActions([15], [-12, 0, 12], [0])

    res_lims = np.array([[-200, -200, -np.pi, 0], [200, 200, np.pi, 0]])
    goal1_lims = np.array([[200, 0, 0], [200, 0, 0]])
    goal2_lims = np.array([[-200, 0, 0], [-200, 0, 0]])
    env = fw_coll_env.env.FwCollisionGymEnv(
        env_c, res_lims, res_lims, goal1_lims, goal2_lims, avail_actions)
    env.reset()

    while True:
        ac = np.array([0, 0, 0])
        obs, rew, done, info = env.step(ac)
        env.render()

        if done:
            env.reset()


if __name__ == '__main__':
    main()
