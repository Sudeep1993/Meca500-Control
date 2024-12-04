#include <iostream>

#include "kinematics/kinematics.h"
#include "kinematics/config.h"

int main(int argc, char** argv)
{
    Kinematics kinematics;
    kinematics.setRobotUrdf("../urdfs/assy_arm_r12/Assy-Arm-R12/urdf/Assy-Arm-R12.urdf", "../urdfs/assy_arm_r12/");

    Eigen::Matrix<double, DOF, 1> pose;
    Eigen::Matrix<double, NUM_OF_JOINTS, 1> joint_angles;
    const double PI = 3.14159265358979323846264338327950288;

    // Joint Angles in Degrees - Input Args
    double iJ0 = strtod(argv[1], NULL);
    double iJ1 = strtod(argv[2], NULL);
    double iJ2 = strtod(argv[3], NULL);
    double iJ3 = strtod(argv[4], NULL);
    double iJ4 = strtod(argv[5], NULL);
    double iJ5 = strtod(argv[6], NULL);

    double j0 = iJ0 * PI/180;
    double j1 = iJ1 * PI/180;
    double j2 = iJ2 * PI/180;
    double j3 = iJ3 * PI/180;
    double j4 = iJ4 * PI/180;
    double j5 = iJ5 * PI/180;
    
    joint_angles << j0, j1, j2, j3, j4, j5;
    std::cout << "*** FORWARD KINEMATICS ***" << std::endl;
    std::cout << "\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~\nJoint Angles:\n" << joint_angles << std::endl;
    kinematics.forward(joint_angles, pose, false);
    std::cout << "\n~~~~~~~~~~~~~~~\nPose:\n" << pose << "\n~~~~~~~~~~~~~~~\n" << std::endl;


    //for the pose: x, y, z, r, p, y - xyz in meters, rpy in radians

    bool success = kinematics.inverse(pose, joint_angles, false, true, true);
    
    if (success)
    {
        std::cout << "*** INVERSE KINEMATICS ***" << std::endl;
        std::cout << "\n~~~~~~~~~~~~~~~\nJoint Angles (Radians):\n" << joint_angles << "\n~~~~~~~~~~~~~~~\n" << std::endl;
        std::cout << "Joint Angles (Degrees):" << std::endl;
        std::cout << joint_angles[0]*180/PI << std::endl;
        std::cout << joint_angles[1]*180/PI << std::endl;
        std::cout << joint_angles[2]*180/PI << std::endl;
        std::cout << joint_angles[3]*180/PI << std::endl;
        std::cout << joint_angles[4]*180/PI << std::endl;
        std::cout << joint_angles[5]*180/PI << std::endl;
        std::cout << "\n~~~~~~~~~~~~~~~\n" << std::endl;

        std::cout << "*** FORWARD KINEMATICS ***" << std::endl;
        kinematics.forward(joint_angles, pose, false);
        std::cout << "\n~~~~~~~~~~~~~~~\nPose:\n" << pose << "\n~~~~~~~~~~~~~~~\n" << std::endl;
    }
    else
    {
        std::cout << "Collision detected or joint limits exceeded" << std::endl;
    }

    return 0;
}
