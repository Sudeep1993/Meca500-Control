#include "kinematics/dynamics.h"

Dynamics::Dynamics(const std::string & model_file)
{
    pinocchio::urdf::buildModel(model_file, pinocchio::JointModelFreeFlyer(), model);
    data = pinocchio::Data(model);
}

Dynamics::~Dynamics()
{
    
}

Eigen::VectorXd Dynamics::getTorques(const Eigen::VectorXd & q, const Eigen::VectorXd & v, const Eigen::VectorXd & a)
{
    pinocchio::rnea(model, data, q, v, a);
    return data.tau.transpose();
}