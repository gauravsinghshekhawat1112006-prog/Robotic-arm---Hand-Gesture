import pygame
import numpy as np
import sys
import random
from visual_kinematics.RobotSerial import *
from visual_kinematics.Frame import Frame

# ----------------------------
# PYGAME SETUP
# ----------------------------
pygame.init()

WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Online IK with Random Replanning")

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

# Initial joint configuration
theta1, theta2 = 0.0, 0.0

# ----------------------------
# TARGET POINTS (pixels)
# ----------------------------
targets = [
    np.array([[175], [0], [0]]),
    np.array([[0], [175], [0]]),
    np.array([[100], [0], [0]]),
    np.array([[50], [0], [0]])
]

current_target_index = 0
current_target = targets[current_target_index]

trail_points = []

# ----------------------------
# MAIN LOOP
# ----------------------------
running = True

while running:
    clock.tick(60)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # ----------------------------
    # FORWARD KINEMATICS (current pose)
    # ----------------------------
    x1 = L1 * np.cos(theta1)
    y1 = L1 * np.sin(theta1)

    x2 = x1 + L2 * np.cos(theta1 + theta2)
    y2 = y1 + L2 * np.sin(theta1 + theta2)

    current_xyz = np.array([[x2], [y2], [0]])

    # ----------------------------
    # RANDOM INTERRUPTION
    # ----------------------------
    if random.random() < 0.01:  # 1% chance per frame
        current_target_index = (current_target_index + 1) % len(targets)
        current_target = targets[current_target_index]

    # ----------------------------
    # MOVE TOWARD CURRENT TARGET
    # ----------------------------
    direction = current_target - current_xyz
    distance = np.linalg.norm(direction)

    if distance > 2:
        step_size = 3
        step_vector = direction / distance * step_size
        new_xyz = current_xyz + step_vector
        
    else:
        # Target reached → go to next
        trail_points.clear()
        current_target_index = (current_target_index + 1) % len(targets)
        current_target = targets[current_target_index]
        new_xyz = current_xyz

    # ----------------------------
    # INVERSE KINEMATICS
    # ----------------------------
    end = Frame.from_euler_3([0, 0, 0], new_xyz)
    robot.inverse(end)

    if robot.is_reachable_inverse:
        theta1, theta2 = robot.axis_values

    # ----------------------------
    # STORE TRAIL (persistent)
    # ----------------------------
    trail_points.append((base_x + x2, base_y + y2))

    # ----------------------------
    # DRAW
    # ----------------------------
    screen.fill((40, 40, 40))

    # Draw trail
    if len(trail_points) > 1:
        pygame.draw.lines(screen, (0, 255, 0), False, trail_points, 2)

    # Draw base
    pygame.draw.circle(screen, (255, 255, 255), (base_x, base_y), 8)

    # Draw links
    pygame.draw.line(screen, (0, 200, 255),
                     (base_x, base_y),
                     (base_x + x1, base_y + y1), 6)

    pygame.draw.line(screen, (255, 100, 100),
                     (base_x + x1, base_y + y1),
                     (base_x + x2, base_y + y2), 6)

    # Draw joints
    pygame.draw.circle(screen, (255, 255, 255),
                       (int(base_x + x1), int(base_y + y1)), 6)

    pygame.draw.circle(screen, (255, 255, 0),
                       (int(base_x + x2), int(base_y + y2)), 6)

    pygame.display.flip()

pygame.quit()
sys.exit()