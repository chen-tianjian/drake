#pragma once
#include <unordered_map>
#include <vector>

#include "drake/geometry/geometry_ids.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake::examples::multibody::suction_gripper {

/// @brief Base class of the suction gripper multibody model. Any suction
/// gripper multibody model should inherit from this class and construct the
/// gripper geometries additionally.
class SuctionGripperMultibodyModel {
 public:
  SuctionGripperMultibodyModel(
      drake::multibody::MultibodyPlant<double>* plant_ptr,
      const drake::multibody::Body<double>& wrist_body,
      double cup_modulus)
      : plant_ptr_{plant_ptr}, wrist_body_{wrist_body}, cup_modulus_{cup_modulus} {
    gripper_model_instance_ =
        plant_ptr_->AddModelInstance("SuctionGripper" + std::to_string(count));
    SuctionGripperMultibodyModel::count++;
  }

  virtual ~SuctionGripperMultibodyModel() = default;

  drake::geometry::GeometryId get_base_body_geom_id() const;

  drake::geometry::GeometryId get_cup_body_geom_id() const;

  const std::vector<drake::multibody::BodyIndex>&
  get_suction_cup_base_body_id_vec() const;

  drake::multibody::ModelInstanceIndex get_gripper_model_instance() const;
  virtual double CalcCupArea() const = 0;

  static int count;

 protected:
  drake::multibody::MultibodyPlant<double>* plant_ptr_{nullptr};
  const drake::multibody::Body<double>& wrist_body_;
  drake::multibody::ModelInstanceIndex gripper_model_instance_;

  std::vector<drake::multibody::BodyIndex> suction_cup_base_body_idx_vec_;

  drake::geometry::GeometryId base_body_geom_id_;
  drake::geometry::GeometryId cup_body_geom_id_;
  const double cup_modulus_; 

};

}  // namespace drake::examples::multibody::suction_gripper
