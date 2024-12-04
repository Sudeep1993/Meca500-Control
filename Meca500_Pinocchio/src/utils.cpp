#include "kinematics/utils.h"

Utils::Utils()
{

}

Utils::~Utils()
{

}

//function used for conversion of given roll, pitch and yaw 3D vector into Rotational matrix

Eigen::Matrix3d Utils::RPYToMat(Eigen::Vector3d rpy)
{
    Eigen::AngleAxisd rx(rpy[0] * M_PI / 180.0, Eigen::Vector3d::UnitX()); //represents a 3D rotation angle around X-axis
    Eigen::AngleAxisd ry(rpy[1] * M_PI / 180.0, Eigen::Vector3d::UnitY()); //represents a 3D rotation angle around Y-axis
    Eigen::AngleAxisd rz(rpy[2] * M_PI / 180.0, Eigen::Vector3d::UnitZ()); //represents a 3D rotation angle around Z-axis

    Eigen::Quaternion<double> q = rx * ry * rz; //3D rotation is represented using Quaternion form (w+xi+yj+zk)

    return q.matrix(); //returning the rotation matrix which is obtained using given roll, pitch and yaw angles
}

//function used to convert given 3D rotational matrix into 3D vector consisting of roll, pitch and yaw angles

Eigen::Vector3d Utils::MatToRPY(Eigen::Matrix3d mat)
{
    return mat.eulerAngles(0, 1, 2) * 180.0 / M_PI; //returns a rotation matrix into vector consisting of three euler angles 
}


double Utils::degToRad(double deg)
{
    return deg * PI / 180.0; //converts degrees to radians
}

double Utils::radToDeg(double rad)
{
    return rad * 180.0 / PI; //converts radians to degrees
}

//

bool Utils::checkNull(Eigen::Matrix<double, NUM_OF_JOINTS, 1> joint_angles)
{
    for (int i = 0; i < NUM_OF_JOINTS; i++)
    {
        if (joint_angles[i] != -1)
        {
            return false;
        }
    }
    return true;
}

//prints out all joint angles 

void Utils::printJointAngles(Eigen::Matrix<double, NUM_OF_JOINTS, 1> joint_angles)
{
    std::cout << "Joint angles:\t";
    for (int i = 0; i < NUM_OF_JOINTS; i++)
    {
        std::cout << joint_angles[i] << ",\t";
    }
    std::cout << std::endl;
}

//prints out the pose of the end effector

void Utils::printPose(Eigen::Matrix<double, DOF, 1> pose)
{
    std::cout << "Pose:\t";
    for (int i = 0; i < DOF; i++)
    {
        std::cout << pose[i] << ",\t";
    }
    std::cout << std::endl;
}
