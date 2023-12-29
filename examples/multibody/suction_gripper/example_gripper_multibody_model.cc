#include "drake/examples/multibody/suction_gripper/example_gripper_multibody_model.h"

#include <cmath>
#include <vector>

#include "drake/geometry/shape_specification.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/prismatic_spring.h"
#include "drake/geometry/proximity_properties.h"


namespace drake::examples::multibody::suction_gripper {

ExampleGripperMultibodyModel::ExampleGripperMultibodyModel(
    drake::multibody::MultibodyPlant<double>* plant_ptr,
    const drake::multibody::Body<double>& wrist_body, 
    double cup_modulus)
    : SuctionGripperMultibodyModel(plant_ptr, wrist_body, cup_modulus) {
  // add the base
  auto& base_body = plant_ptr_->AddRigidBody(
      /*name*/ "base_body",
      /*model_instance*/ gripper_model_instance_,
      /*inertia*/
      drake::multibody::SpatialInertia<double>::MakeFromCentralInertia(
          /*m*/ kBaseMass,
          /*p_PScm_E*/ Eigen::Vector3d(0., 0., -kBaseHeight / 2),
          /*I_SScm_E*/
          drake::multibody::RotationalInertia<double>(
              kBaseInertiaX, kBaseInertiaY, kBaseInertiaZ)));

  plant_ptr_->AddJoint<drake::multibody::WeldJoint>(
      "wrist_to_gripper_joint", wrist_body,
      drake::math::RigidTransform<double>(), base_body,
      drake::math::RigidTransform<double>(), kWristGripperTransform);

  drake::multibody::CoulombFriction base_friction(
      /*static_friction */ kBaseFriction, /* dynamic_friction*/ kBaseFriction);

  plant_ptr_->RegisterVisualGeometry(
      /*body*/ base_body,
      /*X_BG*/
      drake::math::RigidTransform<double>(
          Eigen::Vector3d(0., 0., -kBaseHeight / 2)),
      /*shape*/ drake::geometry::Box(kBaseWidth, kBaseWidth, kBaseHeight),
      /*name*/ "base_body_visual_geom",
      /*diffuse_color*/ Eigen::Vector4d(0.2, 0.2, 0.2, 1.0)); // dark gray 

  plant_ptr_->RegisterVisualGeometry(
      /*body*/ base_body,
      /*X_BG*/
      drake::math::RigidTransform<double>(
          Eigen::Vector3d(0., 0., -kBaseHeight - kCupFittingHeight / 2)),
      /*shape*/
      drake::geometry::Cylinder(kCupFittingDiameter / 2, kCupFittingHeight),
      /*name*/ "base_body_cup_fitting_visual_geom",
      /*diffuse_color*/ Eigen::Vector4d(0.5, 0.5, 0.5, 1.0)); // light gray 

  // TODO: make this hydroelastic too with metal modulus 
  plant_ptr_->RegisterCollisionGeometry(
      /*body*/ base_body,
      /*X_BG*/
      drake::math::RigidTransform<double>(
          Eigen::Vector3d(0., 0., -kBaseHeight - kCupFittingHeight / 2)),
      /*shape*/
      drake::geometry::Cylinder(kCupFittingDiameter / 2, kCupFittingHeight),
      /*name*/ "base_body_cup_fitting_collision_geom",
      /*coulomb_friction*/ base_friction);

  // TODO: better member variable name to distinguish visual 
  // TODO: add corresponding collision geometry here
  base_body_geom_id_ = plant_ptr_->RegisterVisualGeometry(
      /*body*/ base_body,
      /*X_BG*/
      drake::math::RigidTransform<double>(Eigen::Vector3d(
          0., 0., -kBaseHeight - kCupFittingHeight - kCupHeight / 4)),
      /*shape*/
      drake::geometry::Cylinder(kCupOuterDiameter / 2, kCupHeight / 2),
      /*name*/ "base_body_cup_upper_half_visual_geom",
      /*diffuse_color*/ Eigen::Vector4d(0.1, 0.1, 0.1, 1.0)); // nearly black 

  suction_cup_base_body_idx_vec_.push_back(base_body.index());

  // add the cup
  auto& cup_body = plant_ptr_->AddRigidBody(
      /*name*/ "cup_body",
      /*model_instance*/ gripper_model_instance_,
      /*inertia*/
      drake::multibody::SpatialInertia<double>::MakeFromCentralInertia(
          /*m*/ kCupMass / 2,
          /*p_PScm_E*/ Eigen::Vector3d(0., 0., -kCupHeight / 2),
          /*I_SScm_E*/
          drake::multibody::RotationalInertia<double>(
              kCupInertiaX / 2, kCupInertiaY / 2, kCupInertiaZ / 2)));

//   auto& base_to_cup_body_joint =
//       plant_ptr_->AddJoint<drake::multibody::PrismaticJoint>(
//           /*name*/ "base_to_cup_body_joint",
//           /*parent*/ base_body,
//           /*X_PF*/
//           drake::math::RigidTransform<double>(Eigen::Vector3d(
//               0., 0., -kBaseHeight - kCupFittingHeight - kCupHeight * 3 / 4)),
//           /*child*/ cup_body,
//           /*X_BM*/ drake::math::RigidTransform<double>(),
//           /*axis*/ Eigen::Vector3d::UnitZ(),
//           /*pos_lower_limit*/ 0.,
//           /*pos_upper_limit*/ 0, // object slips through even if this is 0 
//           /*damping*/ kCupDamping);
  plant_ptr_->AddJoint<drake::multibody::WeldJoint>(
          /*name*/ "base_to_cup_body_joint",
          /*parent*/ base_body,
          /*X_PF*/
          drake::math::RigidTransform<double>(),
          /*child*/ cup_body,
          /*X_BM*/ drake::math::RigidTransform<double>(),
          drake::math::RigidTransform<double>(Eigen::Vector3d(
              0., 0., -kBaseHeight - kCupFittingHeight - kCupHeight * 3 / 4))); 

// TODO: explore adding back 
(void) kCupStiffness; 
(void) kCupDamping;
//   plant_ptr_->AddForceElement<drake::multibody::PrismaticSpring>(
//       /*joint*/ base_to_cup_body_joint,
//       /*nominal_position*/ 0.,
//       /*stiffness*/ kCupStiffness);

  plant_ptr_->RegisterVisualGeometry(
      /*body*/ cup_body,
      /*X_BG*/ drake::math::RigidTransform<double>(),
      /*shape*/
      drake::geometry::Cylinder(kCupOuterDiameter / 2, kCupHeight / 2),
      /*name*/ "cup_body_lower_half_visual_geom",
      /*diffuse_color*/ Eigen::Vector4d(0.1, 0.1, 0.1, 1.0));

  // collision geometry for cup_body 
  geometry::ProximityProperties cup_body_props; 
  geometry::AddContactMaterial(0.0, 5e3 /* point stiffness */, base_friction, &cup_body_props);
  const auto cup_hydroelastic_modulus = cup_modulus_;
  const auto mesh_res_hint = (kCupOuterDiameter/2); // 10 you can feel slowdown. 5 is real time but smooth cylinder. 

//   (void) cup_hydroelastic_modulus;
//   (void) mesh_res_hint;
  geometry::AddCompliantHydroelasticProperties(mesh_res_hint, cup_hydroelastic_modulus, &cup_body_props);
  cup_body_geom_id_ = plant_ptr_->RegisterCollisionGeometry(
      /*body*/ cup_body,
      /*X_BG*/ drake::math::RigidTransform<double>(Eigen::Vector3d(
              0., 0., kCupHeight/4)),
      /*shape*/ drake::geometry::Cylinder(kCupOuterDiameter / 2, kCupHeight), // runs into pc body regardless of height 
      /*name*/ "cup_body_collision_geom",
      /*proximity properties*/ cup_body_props);

}

}  // namespace drake::examples::multibody::suction_gripper
