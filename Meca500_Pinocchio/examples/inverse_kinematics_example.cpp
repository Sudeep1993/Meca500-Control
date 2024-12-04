#include <iostream>

#include "kinematics/kinematics.h"
#include "kinematics/config.h"

int main()
{
    Kinematics kinematics;

    Eigen::Matrix<double, DOF, 1> pose;
    Eigen::Matrix<double, NUM_OF_JOINTS, 1> joint_angles;

    pose << 0.13, 0.012, 0.6, 0, 0, 0;

    kinematics.setRobotUrdf("../urdfs/assy_arm_r12/Assy-Arm-R12/urdf/Assy-Arm-R12.urdf");
    bool success = kinematics.inverse(pose, joint_angles, true, true);
    std::cout << joint_angles;

	return 0;
}
