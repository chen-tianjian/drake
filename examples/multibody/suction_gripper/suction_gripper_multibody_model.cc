#include "drake/examples/multibody/suction_gripper/suction_gripper_multibody_model.h"

namespace drake::examples::multibody::suction_gripper {

int SuctionGripperMultibodyModel::count = 0;

drake::multibody::ModelInstanceIndex
SuctionGripperMultibodyModel::get_gripper_model_instance() const {
  if (!gripper_model_instance_.is_valid()) {
    throw std::runtime_error("Suction gripper model instance not initialized.");
  } else {
    return gripper_model_instance_;
  }
}

const std::vector<drake::multibody::BodyIndex>&
SuctionGripperMultibodyModel::get_suction_cup_base_body_id_vec() const {
  if (suction_cup_base_body_idx_vec_.empty()) {
    throw std::runtime_error(
        "Suction cup base body indices not defined.");
  } else {
    return suction_cup_base_body_idx_vec_;
  }
}

drake::geometry::GeometryId SuctionGripperMultibodyModel::get_base_body_geom_id() const {
  return base_body_geom_id_;
}

drake::geometry::GeometryId SuctionGripperMultibodyModel::get_cup_body_geom_id() const {
  return cup_body_geom_id_;
}

}  // namespace drake::examples::multibody::suction_gripper
