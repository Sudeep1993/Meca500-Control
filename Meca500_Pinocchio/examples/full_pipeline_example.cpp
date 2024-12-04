#include <iostream>

#include "kinematics/kinematics.h"
#include "kinematics/motion_generator.h"
#include "kinematics/utils.h"

int main(int argc, char** argv)
{
    Kinematics kinematics;
    Utils utils;
    MotionGenerator motion_generator;

    kinematics.setRobotUrdf("../urdfs/assy_arm_r12/Assy-Arm-R12/urdf/Assy-Arm-R12.urdf");

    Eigen::Matrix<double, NUM_OF_JOINTS, 1> joint_angles;
    Eigen::Matrix<double, DOF, 1> pose;
    std::vector<Eigen::Matrix<double, NUM_OF_JOINTS, 1>> linear_movement_joint_angles;
    std::vector<Eigen::Matrix<double, DOF, 1>> linear_movement_pose;
    double distance = -0.3;
    int steps = 100;
    bool success = true;

    pose << 0.13, 0.012, 0.23, 0, 0, 0;
    linear_movement_pose = motion_generator.moveAlongEEAxis(pose, distance, steps);

    success = kinematics.inverse(pose, joint_angles, true, false);

    for (int i = 0; i < steps; i++)
    {
        success = kinematics.inverse(linear_movement_pose[i], joint_angles, true, false);
     
        bool null = utils.checkNull(joint_angles);
     
        if (null)
        {
            std::cout << "failed at " << i << std::endl;
            break;
        }

        linear_movement_joint_angles.push_back(joint_angles);

        utils.printPose(linear_movement_pose[i]);
        utils.printJointAngles(linear_movement_joint_angles[i]);
        std::cout << std::endl;
    }    
    
    return 0;
}
