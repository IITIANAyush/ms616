#!/usr/bin/env python3
"""
ms616_kinematic_sim.py

Quick 2D kinematic simulation of the ms616 differential-drive robot,
using the exact forward-kinematics equations from the kinematic model:

    v = (r/2)(wR + wL)
    w = (r/2L)(wR - wL)

    xdot = v cos(theta)
    ydot = v sin(theta)
    thetadot = w

Controls (continuous, hold-to-move):
    W / S     : increase / decrease linear velocity target
    A / D     : increase / decrease angular velocity target
    SPACE     : emergency stop (zero both velocities instantly)
    R         : reset pose to origin
    ESC / Q   : quit

Robot dimensions (from mechanical drawing):
    Chassis footprint : 242.8 mm x 149.78 mm  (drawn as a rectangle)
    Wheel radius r     : 0.065 m  (130 mm dia, matches the kinematic doc)
    Track width 2L     : 0.210 m  (matches wheel_L = 0.105 m in the URDF)

This is a pure kinematic simulation (no dynamics, no slip, no motor lag
beyond the acceleration limiter below) -- intended to sanity-check the
forward-kinematics equations visually before touching Gazebo.
"""

import sys
import math
import pygame


# ============================================================
# ROBOT / SIM PARAMETERS
# ============================================================
WHEEL_RADIUS = 0.065        # r, meters (130 mm dia wheel)
TRACK_HALF_WIDTH = 0.105    # L, meters (2L = 210 mm track width)

CHASSIS_LENGTH = 0.2428     # meters (242.8 mm, from drawing)
CHASSIS_WIDTH = 0.14978     # meters (149.78 mm, from drawing)

MAX_LINEAR_VEL = 0.6        # m/s, cap on v
MAX_ANGULAR_VEL = 3.0       # rad/s, cap on w
LINEAR_ACCEL = 1.2          # m/s^2, how fast v ramps toward target when a key is held
ANGULAR_ACCEL = 6.0         # rad/s^2, how fast w ramps toward target
VEL_DECAY = 3.0             # how fast v/w decay toward 0 when no key held (per second)

PIXELS_PER_METER = 220      # zoom level for drawing
TRAIL_MAX_POINTS = 2000     # breadcrumb trail length before oldest points drop off
TRAIL_FADE_TIME = 8.0       # seconds before a trail point fully fades out

WINDOW_W, WINDOW_H = 1000, 750
FPS = 60

BG_COLOR = (245, 245, 248)
GRID_COLOR = (222, 222, 228)
GRID_COLOR_MAJOR = (200, 200, 208)
CHASSIS_COLOR = (60, 90, 200)
CHASSIS_OUTLINE = (30, 45, 110)
WHEEL_COLOR = (25, 25, 28)
HEADING_COLOR = (230, 60, 60)
TRAIL_COLOR = (60, 160, 120)
TEXT_COLOR = (30, 30, 35)
HUD_BG = (255, 255, 255, 210)


class RobotState:
    """Pose + velocity state, integrated with the diff-drive forward kinematics."""

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0   # radians, 0 = facing +X (screen right)
        self.v = 0.0        # current linear velocity, m/s
        self.w = 0.0        # current angular velocity, rad/s
        self.v_target = 0.0
        self.w_target = 0.0
        self.trail = []     # list of (x, y, timestamp)

    def reset(self):
        self.__init__()

    def wheel_speeds(self):
        """Inverse kinematics: (v, w) -> (wL, wR) in rad/s, matches Eq. 13 of the doc."""
        r = WHEEL_RADIUS
        L = TRACK_HALF_WIDTH
        wR = (self.v + L * self.w) / r
        wL = (self.v - L * self.w) / r
        return wL, wR

    def step(self, dt, forward_input, turn_input, sim_time):
        # Ramp v_target / w_target from held keys (-1, 0, +1 style inputs)
        self.v_target = forward_input * MAX_LINEAR_VEL
        self.w_target = turn_input * MAX_ANGULAR_VEL

        # Accelerate v toward target, or decay toward 0 if no input
        if forward_input != 0:
            self._ramp_toward("v", self.v_target, LINEAR_ACCEL, dt)
        else:
            self._decay_toward_zero("v", dt)

        if turn_input != 0:
            self._ramp_toward("w", self.w_target, ANGULAR_ACCEL, dt)
        else:
            self._decay_toward_zero("w", dt)

        self.v = max(-MAX_LINEAR_VEL, min(MAX_LINEAR_VEL, self.v))
        self.w = max(-MAX_ANGULAR_VEL, min(MAX_ANGULAR_VEL, self.w))

        # Forward kinematics (Eq. 8 / 10-12 of the kinematic model doc)
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.w * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))  # wrap to [-pi, pi]

        # Trail bookkeeping
        self.trail.append((self.x, self.y, sim_time))
        if len(self.trail) > TRAIL_MAX_POINTS:
            self.trail.pop(0)
        # drop points older than TRAIL_FADE_TIME
        cutoff = sim_time - TRAIL_FADE_TIME
        while self.trail and self.trail[0][2] < cutoff:
            self.trail.pop(0)

    def _ramp_toward(self, attr, target, accel, dt):
        cur = getattr(self, attr)
        if cur < target:
            cur = min(cur + accel * dt, target)
        elif cur > target:
            cur = max(cur - accel * dt, target)
        setattr(self, attr, cur)

    def _decay_toward_zero(self, attr, dt):
        cur = getattr(self, attr)
        if cur > 0:
            cur = max(0.0, cur - VEL_DECAY * dt)
        elif cur < 0:
            cur = min(0.0, cur + VEL_DECAY * dt)
        setattr(self, attr, cur)

    def stop(self):
        self.v = 0.0
        self.w = 0.0
        self.v_target = 0.0
        self.w_target = 0.0


# ============================================================
# DRAWING HELPERS
# ============================================================

def world_to_screen(x, y, cam_x, cam_y):
    """World meters -> screen pixels. Screen Y is flipped (pygame Y grows down)."""
    sx = WINDOW_W / 2 + (x - cam_x) * PIXELS_PER_METER
    sy = WINDOW_H / 2 - (y - cam_y) * PIXELS_PER_METER
    return sx, sy


def draw_grid(surface, cam_x, cam_y):
    spacing_m = 0.5  # grid line every 0.5 m
    major_every = 2   # every 2nd line (1 m) drawn darker

    left_m = cam_x - WINDOW_W / 2 / PIXELS_PER_METER
    right_m = cam_x + WINDOW_W / 2 / PIXELS_PER_METER
    bottom_m = cam_y - WINDOW_H / 2 / PIXELS_PER_METER
    top_m = cam_y + WINDOW_H / 2 / PIXELS_PER_METER

    start_i = int(math.floor(left_m / spacing_m))
    end_i = int(math.ceil(right_m / spacing_m))
    for i in range(start_i, end_i + 1):
        x = i * spacing_m
        sx, _ = world_to_screen(x, 0, cam_x, cam_y)
        color = GRID_COLOR_MAJOR if i % major_every == 0 else GRID_COLOR
        pygame.draw.line(surface, color, (sx, 0), (sx, WINDOW_H), 1)

    start_j = int(math.floor(bottom_m / spacing_m))
    end_j = int(math.ceil(top_m / spacing_m))
    for j in range(start_j, end_j + 1):
        y = j * spacing_m
        _, sy = world_to_screen(0, y, cam_x, cam_y)
        color = GRID_COLOR_MAJOR if j % major_every == 0 else GRID_COLOR
        pygame.draw.line(surface, color, (0, sy), (WINDOW_W, sy), 1)

    # world origin marker
    ox, oy = world_to_screen(0, 0, cam_x, cam_y)
    pygame.draw.circle(surface, (150, 150, 160), (int(ox), int(oy)), 4)


def draw_trail(surface, trail, cam_x, cam_y, sim_time):
    if len(trail) < 2:
        return
    for i in range(len(trail) - 1):
        x0, y0, t0 = trail[i]
        x1, y1, t1 = trail[i + 1]
        age = sim_time - t1
        alpha_frac = max(0.0, 1.0 - age / TRAIL_FADE_TIME)
        if alpha_frac <= 0:
            continue
        p0 = world_to_screen(x0, y0, cam_x, cam_y)
        p1 = world_to_screen(x1, y1, cam_x, cam_y)
        color = tuple(int(c * alpha_frac + bg * (1 - alpha_frac))
                      for c, bg in zip(TRAIL_COLOR, BG_COLOR))
        pygame.draw.line(surface, color, p0, p1, 3)


def rotate_point(px, py, theta):
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    return (px * cos_t - py * sin_t, px * sin_t + py * cos_t)


def draw_robot(surface, state, cam_x, cam_y):
    half_len = CHASSIS_LENGTH / 2
    half_wid = CHASSIS_WIDTH / 2

    # chassis rectangle corners in robot-local frame (x forward, y left)
    corners_local = [
        (half_len, half_wid), (half_len, -half_wid),
        (-half_len, -half_wid), (-half_len, half_wid),
    ]
    corners_world = [
        (state.x + rotate_point(lx, ly, state.theta)[0],
         state.y + rotate_point(lx, ly, state.theta)[1])
        for lx, ly in corners_local
    ]
    corners_screen = [world_to_screen(wx, wy, cam_x, cam_y) for wx, wy in corners_world]
    pygame.draw.polygon(surface, CHASSIS_COLOR, corners_screen)
    pygame.draw.polygon(surface, CHASSIS_OUTLINE, corners_screen, 2)

    # wheel marks: small rectangles at +-L on the Y axis, at the axle (x=0 local)
    wheel_len = 0.05   # visual length along rolling direction
    wheel_thick = 0.02
    for side in (1, -1):
        wx_local = 0.0
        wy_local = side * TRACK_HALF_WIDTH
        w_corners_local = [
            (wheel_len / 2, wy_local + wheel_thick / 2),
            (wheel_len / 2, wy_local - wheel_thick / 2),
            (-wheel_len / 2, wy_local - wheel_thick / 2),
            (-wheel_len / 2, wy_local + wheel_thick / 2),
        ]
        w_corners_world = [
            (state.x + rotate_point(lx, ly, state.theta)[0],
             state.y + rotate_point(lx, ly, state.theta)[1])
            for lx, ly in w_corners_local
        ]
        w_corners_screen = [world_to_screen(wx, wy, cam_x, cam_y) for wx, wy in w_corners_world]
        pygame.draw.polygon(surface, WHEEL_COLOR, w_corners_screen)

    # heading indicator: line from center to front edge
    center_screen = world_to_screen(state.x, state.y, cam_x, cam_y)
    front_local = (half_len + 0.03, 0)
    front_world = (state.x + rotate_point(*front_local, state.theta)[0],
                   state.y + rotate_point(*front_local, state.theta)[1])
    front_screen = world_to_screen(*front_world, cam_x, cam_y)
    pygame.draw.line(surface, HEADING_COLOR, center_screen, front_screen, 3)
    pygame.draw.circle(surface, HEADING_COLOR, (int(front_screen[0]), int(front_screen[1])), 4)


def draw_hud(surface, font, state):
    wL, wR = state.wheel_speeds()
    lines = [
        f"x = {state.x:+.3f} m    y = {state.y:+.3f} m    theta = {math.degrees(state.theta):+.1f} deg",
        f"v = {state.v:+.3f} m/s    w = {state.w:+.3f} rad/s",
        f"wheel_L = {wL:+.2f} rad/s    wheel_R = {wR:+.2f} rad/s",
        "",
        "W/S: linear vel   A/D: angular vel   SPACE: stop   R: reset   ESC/Q: quit",
    ]
    hud_surf = pygame.Surface((560, 20 + 22 * len(lines)), pygame.SRCALPHA)
    hud_surf.fill(HUD_BG)
    surface.blit(hud_surf, (10, 10))
    for i, line in enumerate(lines):
        text_surf = font.render(line, True, TEXT_COLOR)
        surface.blit(text_surf, (20, 16 + 22 * i))


# ============================================================
# MAIN LOOP
# ============================================================

def main():
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption("ms616 — 2D Kinematic Simulation (WASD)")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas", 16)
    if font is None:
        font = pygame.font.Font(None, 18)

    state = RobotState()
    sim_time = 0.0
    running = True

    while running:
        dt = clock.tick(FPS) / 1000.0
        sim_time += dt

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE, pygame.K_q):
                    running = False
                elif event.key == pygame.K_r:
                    state.reset()
                elif event.key == pygame.K_SPACE:
                    state.stop()

        keys = pygame.key.get_pressed()
        forward_input = 0
        if keys[pygame.K_w]:
            forward_input += 1
        if keys[pygame.K_s]:
            forward_input -= 1

        turn_input = 0
        if keys[pygame.K_a]:
            turn_input += 1
        if keys[pygame.K_d]:
            turn_input -= 1

        state.step(dt, forward_input, turn_input, sim_time)

        # camera follows the robot, centered
        cam_x, cam_y = state.x, state.y

        screen.fill(BG_COLOR)
        draw_grid(screen, cam_x, cam_y)
        draw_trail(screen, state.trail, cam_x, cam_y, sim_time)
        draw_robot(screen, state, cam_x, cam_y)
        draw_hud(screen, font, state)

        pygame.display.flip()

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()