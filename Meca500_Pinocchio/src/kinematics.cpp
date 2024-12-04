#include <iostream>
#include "kinematics/kinematics.h"


Kinematics::Kinematics()
{

}

Kinematics::~Kinematics()
{

}

void Kinematics::setRobotUrdf(const char* urdf_path)
{
    pinocchio::urdf::buildModel(urdf_path, model_);

    J_.resize(NUM_OF_JOINTS, model_.nv);
    v_.resize(model_.nv);
    q_ = pinocchio::neutral(model_);
}

void Kinematics::setRobotUrdf(const char* urdf_path, const char* model_path)
{
    pinocchio::urdf::buildModel(urdf_path, model_);
    pinocchio::urdf::buildGeom(model_, urdf_path, pinocchio::COLLISION, geom_model_, model_path);
    geom_model_.addAllCollisionPairs();

    J_.resize(NUM_OF_JOINTS, model_.nv);
    v_.resize(model_.nv);
    q_ = pinocchio::neutral(model_);
}

void Kinematics::forward(Eigen::Matrix<double, NUM_OF_JOINTS, 1> joint_angles, Eigen::Matrix<double, DOF, 1>& pose, bool track_ee)
{
    Eigen::VectorXd joint_angle_vec(NUM_OF_JOINTS);

    for (int i = 0; i < NUM_OF_JOINTS; i++)
    {
        joint_angle_vec[i] = joint_angles[i];
    }

    pinocchio::Data data(model_);
    pinocchio::forwardKinematics(model_, data, joint_angle_vec);
    pinocchio::updateFramePlacements(model_, data);

    if (track_ee)
    {
        Eigen::Vector3d rpy = utils.MatToRPY(data.oMf[model_.getFrameId("Needle tip")].rotation());
        pose << data.oMf[model_.getFrameId("Needle tip")].translation().transpose()[0], data.oMf[model_.getFrameId("Needle tip")].translation().transpose()[1], data.oMf[model_.getFrameId("Needle tip")].translation().transpose()[2], rpy[0], rpy[1], rpy[2];
    }
    else
    {
        Eigen::Vector3d rpy = utils.MatToRPY(data.oMi[DOF].rotation());
        pose << data.oMi[DOF].translation().transpose()[0], data.oMi[DOF].translation().transpose()[1], data.oMi[DOF].translation().transpose()[2], rpy[0], rpy[1], rpy[2];
    }
}

bool Kinematics::inverse(Eigen::Matrix<double, DOF, 1> pose, Eigen::Matrix<double, NUM_OF_JOINTS, 1>& joint_angles, bool track_ee, bool debug, bool check_collision)
{
    pos_ << pose[0], pose[1], pose[2];
    Eigen::Vector3d rpy = {pose[3], pose[4], pose[5]};
    orn_ = utils.RPYToMat(rpy);

    const pinocchio::SE3 oMdes(orn_, pos_);
    pinocchio::SE3 dMi;

    pinocchio::Data data(model_);
    pinocchio::GeometryData geom_data(geom_model_); 

    J_.setZero();
    success_ = false;

    for (int i = 0; ; i++)
    {
        pinocchio::forwardKinematics(model_, data, q_);
        pinocchio::updateFramePlacements(model_, data);

        if (track_ee)
        {
            dMi = oMdes.actInv(data.oMf[model_.getFrameId("Needle tip")]);
        }
        else
        {
            dMi = oMdes.actInv(data.oMi[NUM_OF_JOINTS]);
        }
        
        err_ = pinocchio::log6(dMi).toVector();
        if (err_.norm() < EPS)
        {
            success_ = true;
            break;
        }
        if (i >= IT_MAX)
        {
            success_ = false;
            break;
        }
        pinocchio::computeJointJacobian(model_, data, q_, NUM_OF_JOINTS, J_);
        pinocchio::Data::Matrix6 JJt;
        JJt.noalias() = J_ * J_.transpose();
        JJt.diagonal().array() += DAMP;
        v_.noalias() = -J_.transpose() * JJt.ldlt().solve(err_);
        temp_q_ = pinocchio::integrate(model_, q_, v_ * DT);
        

        if (check_collision)
        {
            pinocchio::updateGeometryPlacements(model_, data, geom_model_, geom_data, temp_q_);
            pinocchio::computeCollisions(model_, data, geom_model_, geom_data, temp_q_);

            for(int  i = 0; i < geom_model_.collisionPairs.size(); i++)
            {
                const hpp::fcl::CollisionResult &cr = geom_data.collisionResults[i];
                if (cr.isCollision())
                {
                    if (debug)
                    {
                        std::cout << "Collision detected, taking one gradient step back" << std::endl;
                    }
                    for (int j = 0; j < NUM_OF_JOINTS; j++)
                    {
                        temp_q_[j] = q_[j];
                    }
                    break;
                }
            }
        }

        for (int i = 0; i < NUM_OF_JOINTS; i++)
        {
            if ((temp_q_[i] < model_.lowerPositionLimit[i]) || (temp_q_[i] > model_.upperPositionLimit[i]))
            {
                if (debug)
                {
                    std::cout << "Joint angles exceeded, for joint " << i << " taking one gradient step back" << std::endl;
                }
                temp_q_[i] = q_[i];
            }
        }

        q_ = temp_q_;

        if(!(i % 10))
        {
            if (debug)
            {
                std::cout << i << ": error = " << err_.transpose() << std::endl;
            }
        }
    }

	
    std::cout << "Velocity: "<< v_ << std::endl;

    for (int i = 0; i < NUM_OF_JOINTS; i++)
    {
        joint_angles[i] = q_.transpose()[i];
    }

    if (success_)
    {
        if (debug)
        {
                std::cout << "Convergence achieved!" << std::endl;
                std::cout << "Result: " << q_.transpose() << std::endl;
                std::cout << "Final error: " << err_.transpose() << std::endl;
        }
        return success_;
    }

    else
    {
        if (debug)
        {
            std::cout << "Warning: the iterative algorithm has not reached convergence to the desired precision" << std::endl;
        }
        return success_;
    }
}
