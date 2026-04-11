import pygame
import math
import sys
import numpy as np
from math import pi
from visual_kinematics.RobotSerial import RobotSerial
from visual_kinematics.Frame import Frame

Line_1 = 12
Line_2 = 7

dh_params = np.array([
    [0., Line_1, 0., 0.],
    [0., Line_2, 0., 0.]
])

robot = RobotSerial(dh_params)

P1 = np.array([7, -1, 0.0])
P2 = np.array([13, -1, 0.0])

points = [P1, P2, P1]

abc = np.array([0.5 * pi, 0., pi])
num_points_per_edge = 100

theta_traj = []
all_theta = []

for i in range(len(points) - 1):
    start = points[i]
    end = points[i + 1]

    for t in np.linspace(0, 1, num_points_per_edge):
        xyz = start * (1 - t) + end * t
        frame = Frame.from_euler_3(abc, xyz.reshape(3, 1))
        robot.inverse(frame)
        theta_traj.append(robot.axis_values.copy())
        all_theta.append(robot.axis_values.copy()*180/pi)

L1 = 216
L2 = 126

pygame.init()
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Rectangle Trajectory")

clock = pygame.time.Clock()

base_x = 200
base_y = HEIGHT-20 

def get_joint_positions(th1, th2):
    x1 = base_x + L1 * math.cos(th1)
    y1 = base_y - L1 * math.sin(th1)

    x2 = x1 + L2 * math.cos(th1 + th2)
    y2 = y1 - L2 * math.sin(th1 + th2)

    return (x1, y1), (x2, y2)


running = True
index = 0
path_points = []
# print(all_theta)
while running:
    screen.fill((30, 30, 30))

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    index = (index + 1) % len(theta_traj)
    theta1, theta2 = theta_traj[index]
    print(theta1*180/pi, theta2*180/pi)
    if index == 0:
        path_points = []

    joint1, end_eff = get_joint_positions(theta1, theta2)

    path_points.append((int(end_eff[0]), int(end_eff[1])))

    if len(path_points) > 1:
        pygame.draw.lines(screen, (0, 255, 0), False, path_points, 2)

    pygame.draw.line(screen, (0, 200, 255), (base_x, base_y), joint1, 5)
    pygame.draw.line(screen, (255, 200, 0), joint1, end_eff, 5)

    pygame.draw.circle(screen, (255, 255, 255), (base_x, base_y), 6)
    pygame.draw.circle(screen, (255, 255, 255), (int(joint1[0]), int(joint1[1])), 6)
    pygame.draw.circle(screen, (255, 0, 0), (int(end_eff[0]), int(end_eff[1])), 6)

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
sys.exit()
