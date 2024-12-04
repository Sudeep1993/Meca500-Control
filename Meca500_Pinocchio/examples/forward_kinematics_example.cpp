#include <iostream>

#include "kinematics/kinematics.h"
#include "kinematics/config.h"

int main()
{
    Kinematics kinematics;

    Eigen::Matrix<double, DOF, 1> pose;
    Eigen::Matrix<double, NUM_OF_JOINTS, 1> joint_angles;

    for (int i = 0; i <= NUM_OF_JOINTS; i++)
    {
        joint_angles[i] = 0;
    }

    kinematics.setRobotUrdf("../urdfs/assy_arm_r12/Assy-Arm-R12/urdf/Assy-Arm-R12.urdf");
    kinematics.forward(joint_angles, pose, true);
    std::cout << pose << std::endl;

	return 0;
}
