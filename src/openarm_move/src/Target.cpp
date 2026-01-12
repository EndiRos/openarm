#include "Target.hpp"

Target::Target(): pos_({0.0,0.0,0.0}) , rot_({0.0,0.0,0.0,1.0}), frame_ (DEFAULT_FRAME){}



Target::Target(std::vector<double> pos, std::vector<double> rot, std::string frame) {
  if (pos.size() != 3) {
    RCLCPP_ERROR(LOGGER, "Position must have x y z format");
    return;
  }
  if (rot.size() > 4 && rot.size() < 3)
    RCLCPP_ERROR(LOGGER, "Rotation must be in quaternion or y format");
  return;
  point_.affine();
  point_.translation().x()= pos[0];
  point_.translation().y()= pos[1];
  point_.translation().z()= pos[2];
  if (rot.size() == 4){
    

  }
  rot_ = rot;
  
}

}

geometry_msgs::msg::Pose Target::Pos() { return geometry_msgs::msg::Pose(); }
