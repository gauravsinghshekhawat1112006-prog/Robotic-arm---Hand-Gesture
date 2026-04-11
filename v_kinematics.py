import numpy as np 
import math
from math import pi 
from visual_kinematics.RobotSerial import RobotSerial
from visual_kinematics.Frame import Frame


L1 = 0.54
L2 = 0.54
# d,a, alpha, theta
dh_params = np.array([[0., L1, 0., math.radians(90)],
                      [0., L2, 0., math.radians(-90)]])

robot = RobotSerial(dh_params)
# robot.show()
xyz = np.array([[0.34], [-0.2], [0.0]])
abc = np.array([0.5 * pi, 0., pi])
# ([-0.5 * pi, 0.5 * pi, 0.])
end = Frame.from_euler_3(abc, xyz)
robot.inverse(end)
angles=robot.axis_values
print(angles[0]*180/pi, angles[1]*180/pi)
robot.show()

# import numpy as np
# import math
# from math import pi
# from visual_kinematics.RobotSerial import RobotSerial
# from visual_kinematics.Frame import Frame
# from visual_kinematics.RobotTrajectory import RobotTrajectory

# # Link lengths
# L1 = 0.54
# L2 = 0.54

# # DH parameters: [d, a, alpha, theta]
# dh_params = np.array([
#     [0., L1, 0., 0.]])

# # Create robot
# robot = RobotSerial(dh_params)

# # -------- Circle Parameters --------
# radius = 0.54                 # Must be <= L1 + L2
# center_x = 0.3
# center_y = 0.0
# z_const = 0.0

# # Constant orientation
# abc = np.array([0.5 * pi, 0., pi])

# # Generate circular frames
# theta_path = np.linspace(0, 2*pi, 80)

# frames = []
# for t in theta_path:
#     x = center_x + radius * math.cos(t)
#     y = center_y + radius * math.sin(t)
#     z = z_const

#     xyz = np.array([[x], [y], [z]])
#     frame = Frame.from_euler_3(abc, xyz)
#     frames.append(frame)

# # -------- Trajectory --------
# trajectory = RobotTrajectory(robot, frames)

# # Motion types:
# # "p2p"  -> joint interpolation
# # "lin"  -> Cartesian interpolation (better for circles)
# trajectory.show(motion="lin")

# import numpy as np
# import math
# from math import pi
# from visual_kinematics.RobotSerial import RobotSerial
# from visual_kinematics.Frame import Frame
# import time

# # -------- Robot Parameters --------
# L1 = 0.54
# L2 = 0.54

# dh_params = np.array([
#     [0., L1, 0., 0.],
#     [0., L2, 0., 0.]
# ])

# robot = RobotSerial(dh_params)

# # -------- Start and End Points (Cartesian) --------
# start_xyz = np.array([0.6, 0.0, 0.0])
# end_xyz   = np.array([0.2, 0.4, 0.0])

# # Constant orientation
# abc = np.array([0.5 * pi, 0., pi])

# # -------- Generate Straight Line --------
# num_points = 60

# for t in np.linspace(0, 1, num_points):
    
#     # Linear interpolation in Cartesian space
#     xyz = start_xyz * (1 - t) + end_xyz * t
    
#     # Create frame
#     frame = Frame.from_euler_3(abc, xyz.reshape(3, 1))
    
#     # Solve inverse kinematics
#     robot.inverse(frame)
    
#     # Get joint angles
#     theta = np.array(robot.axis_values)
    
#     # -------- Forward using theta input --------
#     f = robot.forward(theta)
    
#     # Show robot
#     robot.show()
    
#     time.sleep(0.05)


# import numpy as np
# import math
# from math import pi
# from visual_kinematics.RobotSerial import RobotSerial
# from visual_kinematics.Frame import Frame
# from visual_kinematics.RobotTrajectory import RobotTrajectory

# # -------- Robot Parameters --------
# L1 = 0.54
# L2 = 0.54

# # DH: [d, a, alpha, theta]
# dh_params = np.array([
#     [0., L1, 0., 0.],
#     [0., L2, 0., 0.]
# ])

# robot = RobotSerial(dh_params)

# # -------- Start and End Points --------
# start_xyz = np.array([0.5, 0.0, 0.0])
# end_xyz   = np.array([0.2, 0.5, 0.0])

# # Keep orientation constant
# abc = np.array([0.5 * pi, 0., pi])

# # -------- Generate Straight Line --------
# num_points = 50
# frames = []

# for t in np.linspace(0, 1, num_points):
#     # Linear interpolation
#     xyz = start_xyz * (1 - t) + end_xyz * t
    
#     frame = Frame.from_euler_3(
#         abc,
#         xyz.reshape(3, 1)
#     )
#     frames.append(frame)

# # -------- Trajectory --------
# trajectory = RobotTrajectory(robot, frames)

# # "lin" ensures straight Cartesian motion
# trajectory.show(motion="lin")



