from visual_kinematics.RobotSerial import *
from visual_kinematics.Frame import Frame
import numpy as np

def main():
    

    dh_params = np.array([
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.8, 0.0, 0.0]
    ])

    robot = RobotSerial(dh_params)

    xyz = np.array([[1.2], [1], [0.0]])
    abc = np.array([0.0, 0.0, 0.0])

    end = Frame.from_euler_3(abc, xyz)          

    robot.inverse(end)

    if robot.is_reachable_inverse:
        print("reachable")
        print("Joint 1 Angle:", np.degrees(robot.axis_values[0]))
        print("Joint 2 Angle:", np.degrees(robot.axis_values[1]))
        robot.show()
    else:
        print("Target is too far away!")

if __name__ == "__main__":
    main()