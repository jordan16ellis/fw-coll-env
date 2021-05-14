from pathlib import Path
from typing import Tuple

import numpy as np

import pyglet
from gym.envs.classic_control import rendering

import mfcbf_c


class Viewer:
    def __init__(self, env: mfcbf_c.FwCollisionEnv):
        self.viewer = rendering.Viewer(1000, 1000)
        bound = 1.25
        # bound = 0.5
        self.viewer.set_bounds(-bound, bound, -bound, bound)

        self.uav1, self.uav1_xform = self._make_uav('blue')
        self.uav2, self.uav2_xform = self._make_uav('red')

        self.goal1 = self._make_goal(
            np.array([0, 0]), env.safety_dist, 'blue', True)
        self.goal1_xform = rendering.Transform()
        self.goal1.add_attr(self.goal1_xform)

        self.goal2 = self._make_goal(
            np.array([0, 0]), env.safety_dist, 'red', True)
        self.goal2_xform = rendering.Transform()
        self.goal2.add_attr(self.goal2_xform)


        self.label = pyglet.text.Label(
            "", x=0, y=0,
            font_name='Bitstream Vera Sans Mono',
            color=(0, 0, 0, 255),
            anchor_y='bottom', width=150, multiline=True)

        self.collision_circle1 = self._make_goal(
            np.array([0, 0]), env.safety_dist, 'gray', False)
        self.collision_circle1_xform = rendering.Transform()
        self.collision_circle1.add_attr(self.collision_circle1_xform)

        self.collision_circle2 = self._make_goal(
            np.array([0, 0]), env.safety_dist, 'gray', False)
        self.collision_circle2_xform = rendering.Transform()
        self.collision_circle2.add_attr(self.collision_circle2_xform)

    def render(self, env: mfcbf_c.FwCollisionEnv) -> bool:
        self.viewer.add_onetime(self.uav1)
        self.uav1_xform.set_translation(
            *self._scale_pos(np.asarray(env.x1.p)[:2]))
        self.uav1_xform.set_rotation(env.x1.th)

        self.viewer.add_onetime(self.uav2)
        self.uav2_xform.set_translation(
            *self._scale_pos(np.asarray(env.x2.p)[:2]))
        self.uav2_xform.set_rotation(env.x2.th)

        self.viewer.add_onetime(self.collision_circle1)
        self.viewer.add_onetime(self.collision_circle2)

        self.collision_circle1_xform.set_translation(
            *self._scale_pos(np.asarray(env.x1.p)[:2]))
        self.collision_circle2_xform.set_translation(
            *self._scale_pos(np.asarray(env.x2.p)[:2]))

        self.viewer.add_onetime(self.goal1)
        self.viewer.add_onetime(self.goal2)

        self.goal1_xform.set_translation(
            *self._scale_pos(np.asarray(env.goal1)[:2]))
        self.goal2_xform.set_translation(
            *self._scale_pos(np.asarray(env.goal2)[:2]))

        self.label.text = \
            (f't:  {env.t: >5.1f}\n'
             f'd1: {env.stats.dist_to_goal1: >3.1f}\n'
             f'd2: {env.stats.dist_to_goal2: >3.1f}\n'
             f'dv: {env.stats.dist_to_veh: >3.1f}\n')
        self._viewer_render_override()
        return self.viewer.isopen

    def _make_uav(
            self, color: str) -> Tuple[rendering.Image, rendering.Transform]:
        fname = str(Path(__file__).parent / f"aircraft_{color}.png")
        uav = rendering.Image(fname, 0.05, 0.1)
        xform = rendering.Transform()
        uav.add_attr(xform)
        return uav, xform

    def _make_goal(
            self, pos: np.ndarray, done_dist: float,
            color: str, filled: bool) -> rendering.PolyLine:
        goal = rendering.make_circle(
            self._scale_pos(done_dist), filled=filled)
        goal._color = rendering.Color((0.47, 0.13, 0.13, 1)) \
            if color == 'red' else rendering.Color((0.086, 0.176, 0.314, 1))
        goal.attrs = [goal._color]
        goal_xform = rendering.Transform()
        goal.add_attr(goal_xform)
        goal_xform.set_translation(*self._scale_pos(np.asarray(pos)[:2]))
        self.viewer.add_geom(goal)
        return goal

    def _scale_pos(self, pos: np.ndarray) -> np.ndarray:
        high_lims = 200
        low_lims = -200
        pct = (pos - low_lims) / (high_lims - low_lims)
        return 2 * (pct - 0.5)

    def _viewer_render_override(self) -> None:
        # not calling render loop so text can be rendered
        # https://stackoverflow.com/a/63928928
        pyglet.gl.glClearColor(1, 1, 1, 1)
        self.viewer.window.clear()
        self.viewer.window.switch_to()
        self.viewer.window.dispatch_events()
        self.viewer.transform.enable()
        for geom in self.viewer.geoms:
            geom.render()
        for geom in self.viewer.onetime_geoms:
            geom.render()
        self.viewer.transform.disable()

        self.label.draw()
        self.viewer.window.flip()
        self.viewer.onetime_geoms = []
