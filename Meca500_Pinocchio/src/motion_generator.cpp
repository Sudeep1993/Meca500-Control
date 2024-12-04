#include <iostream>
#include "kinematics/motion_generator.h"

MotionGenerator::MotionGenerator()
{
    relative_pose_ << 1.0, 0.0, 0.0, 0.0,
                      0.0, 1.0, 0.0, 0.0,
                      0.0, 0.0, 1.0, 0.0,
                      0.0, 0.0, 0.0, 1.0;
}

MotionGenerator::~MotionGenerator()
{

}

std::vector<Eigen::Matrix<double, DOF, 1>>  MotionGenerator::moveAlongEEAxis(Eigen::Matrix<double, DOF, 1> pose, double distance, int lerp_steps)
{
    Eigen::Vector3d rpy = {pose[3], pose[4], pose[5]};
    Eigen::Matrix3d orn = utils.RPYToMat(rpy);

    std::vector<Eigen::Matrix<double, DOF, 1>> pose_table;
    Eigen::Matrix<double, DOF, 1> pose_step;

    relative_pose_(2, 3) = distance;

    current_pose_ << orn(0, 0), orn(0, 1), orn(0, 2), pose[0],
                     orn(1, 0), orn(1, 1), orn(1, 2), pose[1],
                     orn(2, 0), orn(2, 1), orn(2, 2), pose[2],
                     0.0,    0.0,    0.0,    1.0;

    target_pose_ = current_pose_ * relative_pose_;

    double step_size_x = (target_pose_(0, 3) - pose[0]) / lerp_steps;
    double step_size_y = (target_pose_(1, 3) - pose[1]) / lerp_steps;
    double step_size_z = (target_pose_(2, 3) - pose[2]) / lerp_steps;

    for (int i = 1; i < lerp_steps + 1; i++)
    {
        pose_step << pose[0] + step_size_x * i, pose[1] + step_size_y * i, pose[2] + step_size_z * i, pose[3], pose[4], pose[5];
        pose_table.push_back(pose_step);
    }

    return pose_table;
}

std::vector<Eigen::Matrix<double, DOF, 1>> MotionGenerator::linearInterpolate(Eigen::Matrix<double, DOF, 1> pose1, Eigen::Matrix<double, DOF, 1> pose2, int lerp_steps_rot, int lerp_steps_trn)
{
    std::vector<Eigen::Matrix<double, DOF, 1>> pose_table;
    Eigen::Matrix<double, DOF, 1> pose_step;

    Eigen::Vector3d trn_step = {(pose2[0] - pose1[0])/lerp_steps_trn, (pose2[1] - pose1[1])/lerp_steps_trn, (pose2[2] - pose1[2])/lerp_steps_trn};
    Eigen::Vector3d rpy_step = {(pose2[3] - pose1[3])/lerp_steps_rot, (pose2[4] - pose1[4])/lerp_steps_rot, (pose2[5] - pose1[5])/lerp_steps_rot};

    std::cout << "rpy_step\t" << rpy_step << std::endl;

    for (int i = 1; i < lerp_steps_rot + 1; i++)
    {
        pose_step << pose1[0], pose1[1], pose1[2], pose1[3] + rpy_step[0] * i, pose1[4] + rpy_step[1] * i, pose1[5] + rpy_step[2] * i;
        pose_table.push_back(pose_step);
    }

    for (int i = 1; i < lerp_steps_trn + 1; i++)
    {
        pose_step << pose1[0] + trn_step[0] * i, pose1[1] + trn_step[1] * i, pose1[2] + trn_step[2] * i, pose1[3] + rpy_step[0] * lerp_steps_rot, pose1[4] + rpy_step[1] * lerp_steps_rot, pose1[5] + rpy_step[2] * lerp_steps_rot;
        pose_table.push_back(pose_step);
    }


    return  pose_table;
}
