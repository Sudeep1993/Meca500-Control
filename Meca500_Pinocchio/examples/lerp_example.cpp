#include "kinematics/motion_generator.h"
#include <iostream>

int main()
{
    MotionGenerator motion_generator;

    Eigen::Matrix<double, DOF, 1> current_pose;
    double distance = 10;

    current_pose << 10, 5, 12, 0, 0, 0;
    
    std::vector<Eigen::Matrix<double, DOF, 1>> target = motion_generator.moveAlongEEAxis(current_pose, distance, 100);
    
    for (auto i = target.begin(); i != target.end(); i++)
    {
        std::cout << *i << std::endl;
        std::cout << std::endl;
    }
    
    return 0;
}