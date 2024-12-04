#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <Eigen/Dense>
#include "config.h"

class Utils
{
public:
    Utils();
    ~Utils();
    
    Eigen::Matrix3d RPYToMat(Eigen::Vector3d rpy);
    Eigen::Vector3d MatToRPY(Eigen::Matrix3d mat);

    double degToRad(double deg);
    double radToDeg(double rad);

    bool checkNull(Eigen::Matrix<double, NUM_OF_JOINTS, 1> joint_angles);

    void printJointAngles(Eigen::Matrix<double, NUM_OF_JOINTS, 1> joint_angles);
    void printPose(Eigen::Matrix<double, DOF, 1> pose);

private:
    double PI = 3.14159265358979323846264338327950288;
};

#endif // UTILS_H