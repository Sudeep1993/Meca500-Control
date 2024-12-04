#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/geometry.hpp"
 
#include <iostream>
 

int main()
{
  using namespace pinocchio;
  
  const std::string urdf_filename = "../urdfs/assy_arm_r11/Assy-Arm-R11/urdf/Assy-Arm-R11.urdf";
  
  Model model;
  pinocchio::urdf::buildModel(urdf_filename,model);
  Data data(model);

  GeometryModel geom_model;
  pinocchio::urdf::buildGeom(model, urdf_filename, pinocchio::COLLISION, geom_model, "../urdfs/test_description/");
  GeometryData geom_data(geom_model); 
  
  geom_model.addAllCollisionPairs();
  
  Eigen::VectorXd q = pinocchio::neutral(model);
  updateGeometryPlacements(model, data, geom_model, geom_data, q);
  
  computeCollisions(model,data,geom_model,geom_data,q);
  
  for(size_t k = 0; k < geom_model.collisionPairs.size(); ++k)
  {
    const CollisionPair & cp = geom_model.collisionPairs[k];
    const hpp::fcl::CollisionResult & cr = geom_data.collisionResults[k];
    
    std::cout << "collision pair: " << cp.first << " , " << cp.second << " - collision: ";
    std::cout << (cr.isCollision() ? "yes" : "no") << std::endl;
  }
  
  return 0;
}
