#ifndef MOTIONGENERATOR_H
#define MOTIONGENERATOR_H

#include <Eigen/Dense>
#include <vector>

#include "config.h"
#include "utils.h"

class MotionGenerator
{
public:
    MotionGenerator();
    ~MotionGenerator();

    
    std::vector<Eigen::Matrix<double, DOF, 1>> moveAlongEEAxis(Eigen::Matrix<double, DOF, 1> pose, double distance, int lerp_steps);
    std::vector<Eigen::Matrix<double, DOF, 1>> linearInterpolate(Eigen::Matrix<double, DOF, 1> pose1, Eigen::Matrix<double, DOF, 1> pose2, int lerp_steps_rot, int lerp_steps_trn);

private:
    Utils utils;

    Eigen::Matrix4d current_pose_;
    Eigen::Matrix4d target_pose_;
    Eigen::Matrix4d relative_pose_;
    
};

#endif // MOTIONGENERATOR_H

