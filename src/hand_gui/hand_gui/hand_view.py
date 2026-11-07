"""Robot hand view: a MuJoCo copy of the AP002 that mirrors the pipeline and locates the forces.

Pose source (same rule as gripper_camera):
  /hand_sim/full_joint_states fresh -> copy all joints (identical to the simulation)
  else /hand_hw/state               -> actuator targets, own physics follows (real hand)

render() returns the image and, per tactile region, where to draw its force on that image.
The image is turned 90 deg counter-clockwise (palm facing the viewer, thumb up, fingers left):
MuJoCo renders a portrait frame that is rotated, and overlay points / mouse drags are mapped.
The GUI draws the forces in 2D on top (never hidden by the fingers or the object):
  circle : normal force, size and colour relative to the auto-scaled maximum
  arrow  : shear force, length = tangential, direction = reported angle in the region frame
           (sim: fingertip link x/y axes; real force3d: sensor x/y axes, mapping not verified)
"""
import math
import time

import mujoco
import numpy as np

from finger_node.common import TACTILE_REGIONS, default_model_xml
from finger_node.sim_backend import ACTUATORS, BODY_PREFIX_REGION, disable_gravity, load_model

FOLLOW_TIMEOUT = 0.5   # s; full joint states older than this fall back to /hand_hw/state
ARROW_LENGTH = 0.04    # m at full-scale shear
SCALE_FLOOR = {'force3d': 0.2, 'dot_matrix': 50.0}  # smallest full-scale value per tactile kind
SCALE_DECAY = 0.995    # per render: the auto scale slowly shrinks back after a peak
OBJECT_ALPHA = 0.45    # test object drawn see-through so the fingertips stay visible


class HandView:
    def __init__(self, width, height, test_object='none', hand_xml=''):
        # output is width x height; MuJoCo renders the portrait frame height x width before rotation
        self.width, self.height = height, width
        self.model = load_model(hand_xml or default_model_xml(), test_object)
        self.model.vis.global_.offwidth = max(self.model.vis.global_.offwidth, self.width)
        self.model.vis.global_.offheight = max(self.model.vis.global_.offheight, self.height)
        disable_gravity(self.model)
        for g in range(self.model.ngeom):
            if self.model.geom(g).name == 'test_object':
                self.model.geom_rgba[g, 3] = OBJECT_ALPHA
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self.renderer = mujoco.Renderer(self.model, self.height, self.width)
        self.start = time.monotonic()

        self.qpos_addr = {self.model.joint(j).name: int(self.model.jnt_qposadr[j])
                          for j in range(self.model.njnt) if self.model.joint(j).name}
        self.actuator_id = {f: self.model.actuator(name).id for f, name in ACTUATORS.items()}

        # region -> frame body (fingertip distal link / palm base) and the geoms marking its pad
        self.region_body = {}
        hand_bodies = []
        for b in range(self.model.nbody):
            name = self.model.body(b).name
            for prefix, region in BODY_PREFIX_REGION:
                if name.startswith(prefix):
                    hand_bodies.append(b)
                    if region not in self.region_body or 'distal' in name:
                        self.region_body[region] = b
        self.region_geoms = {r: [g for g in range(self.model.ngeom) if self.model.geom_bodyid[g] == b]
                             for r, b in self.region_body.items()}

        self.camera = mujoco.MjvCamera()
        self.camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.home = self.data.xpos[hand_bodies].mean(axis=0)
        self.reset_camera()
        self.scale = dict(SCALE_FLOOR)

    def reset_camera(self):
        self.camera.lookat[:] = self.home
        self.camera.distance = 0.30
        self.camera.azimuth = 0.0     # palm side, fingers pointing up
        self.camera.elevation = 60.0

    # ---- pose ------------------------------------------------------------------
    def update_pose(self, snap):
        now = time.monotonic()
        positions, stamp = snap['full_state']
        if positions is not None and now - stamp < FOLLOW_TIMEOUT:
            for name, value in positions.items():
                if name in self.qpos_addr:
                    self.data.qpos[self.qpos_addr[name]] = value
            mujoco.mj_forward(self.model, self.data)
            return
        targets, _ = snap['hw_state']
        if targets is None:
            return
        for finger, value in targets.items():
            if finger in self.actuator_id:
                a = self.actuator_id[finger]
                lo, hi = self.model.actuator_ctrlrange[a]
                self.data.ctrl[a] = min(max(value, lo), hi)
        target = min(now - self.start, self.data.time + 0.1)
        while self.data.time < target:
            mujoco.mj_step(self.model, self.data)

    # ---- camera control (mouse) ------------------------------------------------
    def rotate(self, dx, dy):
        """Mouse drag in the displayed (rotated) image."""
        dx, dy = -dy, dx  # back to the unrotated render frame
        self.camera.azimuth -= dx * 0.4
        self.camera.elevation = min(89.0, max(-89.0, self.camera.elevation + dy * 0.4))

    def zoom(self, steps):
        self.camera.distance = min(1.5, max(0.1, self.camera.distance * (0.9 ** steps)))

    # ---- render ----------------------------------------------------------------
    def projector(self):
        """World point -> image (u, v) for the camera of the last update_scene."""
        cam = mujoco.mjv_averageCamera(self.renderer.scene.camera[0], self.renderer.scene.camera[1])
        forward, up = np.array(cam.forward), np.array(cam.up)
        right = np.cross(forward, up)
        pos = np.array(cam.pos)
        half = 0.5 * (self.width / self.height) * (cam.frustum_top - cam.frustum_bottom)
        left = cam.frustum_center - half

        def project(point):
            v = np.asarray(point) - pos
            depth = float(v @ forward)
            if depth <= 1e-6:
                return None
            x = float(v @ right) * cam.frustum_near / depth
            y = float(v @ up) * cam.frustum_near / depth
            return ((x - left) / (2 * half) * self.width,
                    (cam.frustum_top - y) / (cam.frustum_top - cam.frustum_bottom) * self.height)
        return project

    def overlays(self, tactile):
        """Per region: dict(region, u, v, fraction, arrow=(u, v) or None, normal, tangential)."""
        for kind in self.scale:
            self.scale[kind] = max(SCALE_FLOOR[kind], self.scale[kind] * SCALE_DECAY)
        project = self.projector()
        items = []
        for region in TACTILE_REGIONS:
            entry = tactile[region]
            if region not in self.region_body or not np.isfinite(entry['normal']):
                continue
            kind = entry['kind'] if entry['kind'] in self.scale else 'force3d'
            shear = entry['tangential'] if np.isfinite(entry['tangential']) else 0.0
            self.scale[kind] = max(self.scale[kind], entry['normal'], shear)
            geoms = self.region_geoms[region]
            point = self.data.geom_xpos[geoms].mean(axis=0) if geoms else self.data.xpos[self.region_body[region]]
            uv = project(point)
            if uv is None:
                continue
            arrow = None
            if shear > 0:
                rot = self.data.xmat[self.region_body[region]].reshape(3, 3)
                angle = math.radians(entry['direction'])
                direction = rot @ np.array([math.cos(angle), math.sin(angle), 0.0])
                arrow = project(point + direction * ARROW_LENGTH * min(shear / self.scale[kind], 1.0))
            items.append(dict(region=region, u=uv[0], v=uv[1], arrow=arrow,
                              fraction=min(entry['normal'] / self.scale[kind], 1.0),
                              normal=entry['normal'], tangential=entry['tangential']))
        return items

    def turn(self, u, v):
        """Point in the portrait render -> point in the image turned 90 deg counter-clockwise."""
        return v, (self.width - 1) - u

    def render(self, tactile):
        """(RGB uint8 image, overlay items) for the current pose, turned counter-clockwise."""
        self.renderer.update_scene(self.data, camera=self.camera)
        image = np.ascontiguousarray(np.rot90(self.renderer.render(), 1))
        items = self.overlays(tactile)
        for item in items:
            item['u'], item['v'] = self.turn(item['u'], item['v'])
            if item['arrow'] is not None:
                item['arrow'] = self.turn(*item['arrow'])
        return image, items

    def close(self):
        self.renderer.close()
