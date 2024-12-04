#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <pinocchio/parsers/sample-models.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include "pinocchio/algorithm/geometry.hpp"

#include "config.h"
#include "utils.h"

class Kinematics
{
public:
    Kinematics();
    ~Kinematics();

    // Set path to URDF
    void setRobotUrdf(const char* urdf_path);
    void setRobotUrdf(const char* urdf_path, const char* model_path);
    
    // Kinematics functions
    void forward(Eigen::Matrix<double, NUM_OF_JOINTS, 1> joint_angles, Eigen::Matrix<double, DOF, 1>& pose, bool track_ee);
    bool inverse(Eigen::Matrix<double, DOF, 1> pose, Eigen::Matrix<double, NUM_OF_JOINTS, 1>& joint_angles, bool track_ee, bool debug, bool check_collision=true);

private:
    Utils utils;
    bool success_ = false;

    pinocchio::Model model_;
    pinocchio::GeometryModel geom_model_;

    Eigen::VectorXd q_;
    Eigen::VectorXd temp_q_;
    pinocchio::Data::Matrix6x J_;
    Eigen::Matrix<double, DOF, 1> err_;
    Eigen::VectorXd v_;

    Eigen::Vector3d pos_;
    Eigen::Matrix3d orn_;
};

#endif // KINEMATICS_H

