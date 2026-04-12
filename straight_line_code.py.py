import pygame
import numpy as np
import sys
from visual_kinematics.RobotSerial import *
from visual_kinematics.Frame import Frame

# ----------------------------
# PYGAME SETUP
# ----------------------------
pygame.init()

WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Full Workspace IK - No Constraints")

clock = pygame.time.Clock()

# ----------------------------
# DISPLAY SCALE
# ----------------------------
scale = 100
L1 = 1.0 * scale
L2 = 0.8 * scale

base_x = WIDTH // 2
base_y = HEIGHT // 2

# ----------------------------
# ROBOT SETUP
# ----------------------------
dh_params = np.array([
    [0.0, L1, 0.0, 0.0],
    [0.0, L2, 0.0, 0.0]
])

robot = RobotSerial(dh_params)

# ----------------------------
# TARGET POINTS (FULL WORKSPACE SAFE LOOP)
# ----------------------------

target_xyz = [
    np.array([[60], [40], [0]]),     # Point A (lower-right)
    np.array([[100], [100], [0]]),   # Point B (upper-right)
    np.array([[160], [80], [0]]),    # Point C (right-top)
    np.array([[120], [20], [0]]),    # Point D (lower)
    np.array([[60], [40], [0]])      # Close loop
]


steps = 150
trajectory = []

print("Computing trajectory...")

# ----------------------------
# TRAJECTORY GENERATION
# ----------------------------
for j in range(1, len(target_xyz)):

    start_xyz = target_xyz[j-1]
    end_xyz = target_xyz[j]

    for i in range(steps):

        r = i / steps
        interp_xyz = start_xyz + (end_xyz - start_xyz) * r

        end = Frame.from_euler_3([0, 0, 0], interp_xyz)

        robot.inverse(end)

        if robot.is_reachable_inverse:

            theta1, theta2 = robot.axis_values

            # Normalize angles to [-pi, pi]
            theta1 = np.arctan2(np.sin(theta1), np.cos(theta1))
            theta2 = np.arctan2(np.sin(theta2), np.cos(theta2))

            trajectory.append([theta1, theta2])

        else:
            print("Unreachable point during trajectory")
            sys.exit()

print("Trajectory ready. Starting animation...")

# ----------------------------
# ANIMATION LOOP
# ----------------------------
running = True
step = 0
trail_points = []

while running:

    clock.tick(60)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    if step < len(trajectory):
        theta1, theta2 = trajectory[step]
        step += 1
    else:
        theta1, theta2 = trajectory[-1]

    # Forward kinematics
    x1 = base_x + L1 * np.cos(theta1)
    y1 = base_y - L1 * np.sin(theta1)

    x2 = x1 + L2 * np.cos(theta1 + theta2)
    y2 = y1 - L2 * np.sin(theta1 + theta2)

    trail_points.append((x2, y2))

    # ----------------------------
    # DRAW
    # ----------------------------
    screen.fill((40, 40, 40))

    if len(trail_points) > 1:
        pygame.draw.lines(screen, (0, 255, 0), False, trail_points, 2)

    pygame.draw.circle(screen, (255, 255, 255), (base_x, base_y), 8)

    pygame.draw.line(screen, (0, 200, 255), (base_x, base_y), (x1, y1), 6)
    pygame.draw.line(screen, (255, 100, 100), (x1, y1), (x2, y2), 6)

    pygame.draw.circle(screen, (255, 255, 255), (int(x1), int(y1)), 6)
    pygame.draw.circle(screen, (255, 255, 0), (int(x2), int(y2)), 6)

    pygame.display.flip()

pygame.quit()
sys.exit()