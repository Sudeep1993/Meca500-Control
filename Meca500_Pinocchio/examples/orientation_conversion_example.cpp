#include <iostream>

#include "kinematics/utils.h"


int main()
{
    Utils utils;

    Eigen::Vector3d rpy_vec;
    Eigen::Matrix3d rot_mat;

    rpy_vec << 30, 60, 90;

    std::cout << rpy_vec << std::endl;
    std::cout << std::endl;

    rot_mat = utils.RPYToMat(rpy_vec);
    rpy_vec = utils.MatToRPY(rot_mat);

    std::cout << rot_mat << std::endl;
    std::cout << std::endl;

    std::cout << rpy_vec << std::endl;
    std::cout << std::endl;

	return 0;
}
