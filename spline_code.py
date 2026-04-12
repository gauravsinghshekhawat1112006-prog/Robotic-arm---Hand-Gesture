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
pygame.display.set_caption("Spline IK Motion with Target Markers")

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
# TARGET POINTS (Complex Forward Curve)
# ----------------------------
target_xyz = [
    np.array([[120], [0], [0]]),
    np.array([[100], [-60], [0]]),
    np.array([[60], [-40], [0]]),
    np.array([[90], [40], [0]]),
    np.array([[150], [0], [0]]),
       
]

# ----------------------------
# NATURAL CUBIC SPLINE FUNCTION
# ----------------------------
def natural_cubic_spline(t, y):
    n = len(t)
    h = np.diff(t)

    A = np.zeros((n, n))
    b = np.zeros(n)

    A[0, 0] = 1
    A[n-1, n-1] = 1

    for i in range(1, n-1):
        A[i, i-1] = h[i-1]
        A[i, i]   = 2 * (h[i-1] + h[i])
        A[i, i+1] = h[i]
        b[i] = 3 * ((y[i+1] - y[i]) / h[i] -
                    (y[i] - y[i-1]) / h[i-1])

    c = np.linalg.solve(A, b)

    a = y[:-1]
    b_coeff = np.zeros(n-1)
    d = np.zeros(n-1)

    for i in range(n-1):
        b_coeff[i] = ((y[i+1] - y[i]) / h[i] -
                      h[i] * (2*c[i] + c[i+1]) / 3)
        d[i] = (c[i+1] - c[i]) / (3*h[i])

    return a, b_coeff, c[:-1], d

# ----------------------------
# COMPUTE SPLINE TRAJECTORY
# ----------------------------
print("Computing spline trajectory...")

n_points = len(target_xyz)
t = np.linspace(0, 1, n_points)

x_points = np.array([p[0,0] for p in target_xyz])
y_points = np.array([p[1,0] for p in target_xyz])

ax, bx, cx, dx = natural_cubic_spline(t, x_points)
ay, by, cy, dy = natural_cubic_spline(t, y_points)

trajectory = []
segment_boundaries = []
samples_per_segment = 80

for i in range(n_points - 1):

    # Skip first sample to avoid duplication
    t_local = np.linspace(t[i], t[i+1], samples_per_segment)[1:]
    dt = t_local - t[i]

    for dti in dt:
        x = ax[i] + bx[i]*dti + cx[i]*dti**2 + dx[i]*dti**3
        y = ay[i] + by[i]*dti + cy[i]*dti**2 + dy[i]*dti**3

        interp_xyz = np.array([[x], [y], [0]])
        end = Frame.from_euler_3([0,0,0], interp_xyz)

        robot.inverse(end)

        if robot.is_reachable_inverse:
            trajectory.append(robot.axis_values.copy())
        else:
            print("Unreachable spline point")
            sys.exit()

    segment_boundaries.append(len(trajectory)-1)

print("Spline ready.")

# ----------------------------
# INITIALIZE AT FIRST POSE
# ----------------------------
theta1, theta2 = trajectory[0]

x1 = base_x + L1 * np.cos(theta1)
y1 = base_y + L1 * np.sin(theta1)

x2 = x1 + L2 * np.cos(theta1 + theta2)
y2 = y1 + L2 * np.sin(theta1 + theta2)

trail_points = [(x2, y2)]
reached_targets = []
step = 1  # start from next point

# ----------------------------
# ANIMATION LOOP
# ----------------------------
running = True

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
    y1 = base_y + L1 * np.sin(theta1)

    x2 = x1 + L2 * np.cos(theta1 + theta2)
    y2 = y1 + L2 * np.sin(theta1 + theta2)

    trail_points.append((x2, y2))

    # Mark target reached
    if step-1 in segment_boundaries:
        reached_targets.append((x2, y2))

    # ----------------------------
    # DRAW
    # ----------------------------
    screen.fill((40, 40, 40))

    # Green trail
    if len(trail_points) > 1:
        pygame.draw.lines(screen, (0, 255, 0), False, trail_points, 2)

    # Red dots at reached targets
    for pt in reached_targets:
        pygame.draw.circle(screen, (255, 0, 0),
                           (int(pt[0]), int(pt[1])), 6)

    # Base
    pygame.draw.circle(screen, (255, 255, 255),
                       (base_x, base_y), 8)

    # Links
    pygame.draw.line(screen, (0, 200, 255),
                     (base_x, base_y), (x1, y1), 6)
    pygame.draw.line(screen, (255, 100, 100),
                     (x1, y1), (x2, y2), 6)

    # Joints
    pygame.draw.circle(screen, (255, 255, 255),
                       (int(x1), int(y1)), 6)
    pygame.draw.circle(screen, (255, 255, 0),
                       (int(x2), int(y2)), 6)

    pygame.display.flip()

pygame.quit()
sys.exit()