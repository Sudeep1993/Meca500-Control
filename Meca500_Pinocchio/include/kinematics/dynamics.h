#ifndef DYNAMICS_H
#define DYNAMICS_H

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/sample-models.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include "pinocchio/algorithm/geometry.hpp"


class Dynamics
{
public:
    Dynamics(const std::string & model_file);
    ~Dynamics();

    Eigen::VectorXd getTorques(const Eigen::VectorXd & q, const Eigen::VectorXd & v, const Eigen::VectorXd & a);

private:
    pinocchio::Model model;
    pinocchio::Data data;
};



#endif // DYNAMICS_H