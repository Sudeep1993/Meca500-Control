#include "kinematics/motion_generator.h"
#include <iostream>

int main()
{
    MotionGenerator motion_generator;

    Eigen::Matrix<double, DOF, 1> pose1;
    Eigen::Matrix<double, DOF, 1> pose2;

    pose1 << 10, 5, 12, 30, 30, 30;
    pose2 << 20, 15, 34, 60, 60, 60;
    
    std::vector<Eigen::Matrix<double, DOF, 1>> target = motion_generator.linearInterpolate(pose1, pose2, 30, 100);
    
    for (auto i = target.begin(); i != target.end(); i++)
    {
        std::cout << *i << std::endl;
        std::cout << std::endl;
    }
    
    return 0;
}