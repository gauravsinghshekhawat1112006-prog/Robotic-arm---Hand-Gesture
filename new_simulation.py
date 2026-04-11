import pygame
import math
import sys
import numpy as np
import time

Line_1 = 0.54
Line_2 = 0.54

L1 = 216
L2 = 216
WIDTH, HEIGHT = 800, 600
base_x = 200
base_y = HEIGHT - 20
scale = L1 / Line_1
num_points = 200


def calculate_traj(start_xyz, target_xyz):
    t = np.linspace(0, 1, num_points)
    start = start_xyz.reshape(3, 1)
    end = target_xyz.reshape(3, 1)
    path = start + (end - start) * t.reshape(1, num_points)
    
    x = path[0, :]
    y = path[1, :]
    
    r_sq = x**2 + y**2
    cos_theta2 = (r_sq - Line_1**2 - Line_2**2) / (2 * Line_1 * Line_2)
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)
    
    sin_theta2 = -np.sqrt(1 - cos_theta2**2)
    theta2 = np.arctan2(sin_theta2, cos_theta2)
    
    k1 = Line_1 + Line_2 * cos_theta2
    k2 = Line_2 * sin_theta2
    theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)
    
    return np.column_stack((theta1, theta2))

def get_joint_positions(th1, th2):
    x1 = base_x + L1 * math.cos(th1)
    y1 = base_y - L1 * math.sin(th1) 
    x2 = x1 + L2 * math.cos(th1 + th2)
    y2 = y1 - L2 * math.sin(th1 + th2)
    return (x1, y1), (x2, y2)

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Optimized Real-time Tracking (Analytical IK)")
clock = pygame.time.Clock()

current_xyz = np.array([1.08, 0.0, 0.0]) 
traj_index = 0
path_points = []
target_points = []
current_target_pixel = None

last_target_time = time.time()
target_interval = 1.5

targets = [
    [0.9, 0.2, 0],
    [0.11, 0.2, 0],
]
target_id = 0

running = True

theta_traj = calculate_traj(current_xyz, current_xyz)

theta1, theta2 = 0.0, 0.0 
if len(theta_traj) > 0:
    theta1, theta2 = theta_traj[0]

while running:
    screen.fill((30, 30, 30))

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # target
    if time.time() - last_target_time > target_interval:
        last_target_time = time.time()
        
        target_xyz = np.array(targets[target_id])
        target_id = (target_id + 1) % len(targets)

        tx = base_x + target_xyz[0] * scale
        ty = base_y - target_xyz[1] * scale
        current_target_pixel = (int(tx), int(ty))
        target_points.append(current_target_pixel)
        
        theta_traj = calculate_traj(current_xyz, target_xyz)
        
        traj_index = 0
        path_points = [] 

    # only runs while moving
    if traj_index < len(theta_traj):
        theta1, theta2 = theta_traj[traj_index]
        traj_index += 1
        
        current_xyz = np.array([
            (Line_1 * math.cos(theta1)) + (Line_2 * math.cos(theta1 + theta2)),
            (Line_1 * math.sin(theta1)) + (Line_2 * math.sin(theta1 + theta2)),
            0.0
        ])

    
    joint1, end_eff = get_joint_positions(theta1, theta2)
    path_points.append((int(end_eff[0]), int(end_eff[1])))

    if len(path_points) > 1:
        pygame.draw.lines(screen, (0, 255, 0), False, path_points, 2)

    pygame.draw.line(screen, (0, 200, 255), (base_x, base_y), joint1, 5)
    pygame.draw.line(screen, (255, 200, 0), joint1, end_eff, 5)

    pygame.draw.circle(screen, (255, 255, 255), (base_x, base_y), 6)
    pygame.draw.circle(screen, (255, 255, 255), (int(joint1[0]), int(joint1[1])), 6)
    pygame.draw.circle(screen, (255, 0, 0), (int(end_eff[0]), int(end_eff[1])), 6)

    for pt in target_points:
        pygame.draw.circle(screen, (255, 200, 0), pt, 4)
    if current_target_pixel:
        pygame.draw.circle(screen, (255, 0, 0), current_target_pixel, 7)

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
sys.exit()