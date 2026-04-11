import numpy as np 
import math
from math import pi 
from visual_kinematics.RobotSerial import RobotSerial
from visual_kinematics.Frame import Frame


L1 = 0.54
L2 = 0.54
# d,a, alpha, theta
dh_params = np.array([[0., L1, 0., 135*pi/180],
                      [0., L2, 0., -95 * pi/180]])

robot = RobotSerial(dh_params)

def compute_forward_kinematics(theta1, theta2):
    theta1_rad = math.radians(theta1)
    theta2_rad = math.radians(theta2)

    joint_angles = np.array([theta1_rad, theta2_rad])
    robot.forward(joint_angles)
    T_Matrix_end = robot.end_frame

    return T_Matrix_end

print(compute_forward_kinematics(0,0))

# xyz = np.array([[0.54], [0.30], [0.]])
# abc = np.array([0.5 * pi, 0., pi])
# # ([-0.5 * pi, 0.5 * pi, 0.])
# end = Frame.from_euler_3(abc, xyz)
# robot.inverse(end)
# angles=robot.axis_values


# print(-1*angles[0]*180/pi,-1*angles[1]*180/pi)
# robot.show()
