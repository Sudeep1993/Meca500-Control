#include <iostream>
#include "kinematics/dynamics.h"

int main()
{
    Dynamics dynamics("../urdfs/assy_arm_r12/Assy-Arm-R12/urdf/Assy-Arm-R12.urdf");
    std::cout << "Dynamics started" << std::endl;

    Eigen::VectorXd q;
    Eigen::VectorXd v;
    Eigen::VectorXd a;
    
    std::cout << "initializing qva" << std::endl;
    v << 1, 0, 0, 0, 0, 0, 0;
    std::cout << "initialized v" << std::endl;
    a << 1, 0, 0, 0, 0, 0, 0;
    std::cout << "initialized a" << std::endl;
    q << 1, 0, 0, 0, 0, 0, 0;
    std::cout << "initialized q" << std::endl;

    std::cout << "calling getTorques" << std::endl;
    
    Eigen::VectorXd torques = dynamics.getTorques(q, v, a);
    std::cout << "Torques: " << torques << std::endl;
    return 0;
}
