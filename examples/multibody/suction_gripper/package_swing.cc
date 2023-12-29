#include <memory>

#include <gflags/gflags.h>
#include <iostream>

#include "drake/common/proto/call_python.h"
#include "drake/examples/multibody/suction_gripper/example_gripper_multibody_model.h"
#include "drake/examples/multibody/suction_gripper/suction_force_model.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/contact_solvers/sap/sap_solver.h"
#include "drake/multibody/math/spatial_algebra.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/visualization/visualization_config_functions.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/systems/primitives/vector_log_sink.h"
#include "drake/examples/multibody/suction_gripper/abstract_logger.h"

namespace drake::examples::multibody::suction_gripper {

DEFINE_double(max_suction_dist, 0.004, "Maximum distance for suction to run.");

DEFINE_double(cup_modulus, 3e5, "hydroelastic modulus of cup");

DEFINE_double(box_modulus, 3e5, "hydroelastic modulus of box");

DEFINE_double(publish_period, 0.005,
              "How often to publish logger.");

DEFINE_double(simulation_time, 5.0,
              "Duration of the simulation (in seconds).");

DEFINE_double(box_diss, 0.0, "dissipation of box");

DEFINE_double(box_pc_stiffness, 1.0e3, "point stiffness of box");

DEFINE_double(initial_angle, M_PI / 4,
              "Initial angle of the package swing (in radian).");

DEFINE_bool(advance_in_time, true, "Whether to advance sim or not");
DEFINE_double(realtime_rate, 1.0, "Realtime rate.");

DEFINE_bool(peeloff, false, "Whether to simulate peeloff (different initial angle).");

// void SaveLogToCsv(std::string filepath, const drake::systems::VectorLog<double>& log) {
//     std::ofstream file;
//     file.open(filepath);

//     // Write header
//     file << "time";
//     const auto matrix = log.data().transpose();
//     for (int i = 0; i < matrix.cols(); i++) {
//         file << ",value" + std::to_string(i);
//     }
//     file << "\n";

//     for (int i = 0; i < matrix.rows(); i++) {
//         const double t = log.sample_times()[i];
//         std::string time_str = std::to_string(t);
//         file << time_str << ',';
//         for (int j = 0; j < matrix.cols(); j++) {
//             std::string str = std::to_string(matrix(i, j));
//             if (j + 1 == matrix.cols()) {
//                 file << str;
//             } else {
//                 file << str << ',';
//             }
//         }
//         file << '\n';
//     }
// }

int do_main() {
  const double kMultibodyTimeStep = 0.002;
  drake::systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] = drake::multibody::AddMultibodyPlantSceneGraph(
      &builder, std::make_unique<drake::multibody::MultibodyPlant<double>>(
                    kMultibodyTimeStep));

  auto& world_body = plant.world_body();

  // Add an object (cardboard box)
  const double kFrictionCoeff = 0.5; // https://wiki.unece.org/display/TransportSustainableCTUCode/Appendix+2.%09Friction+factors
  drake::multibody::CoulombFriction obj_friction(
      /*static_friction */ kFrictionCoeff,
      /* dynamic_friction*/ kFrictionCoeff);

  const double kPackageMass = 2.73;
  const double kPackageLen = 0.3;
  const double kPackageWidth = 0.15;
  auto& obj_body = plant.AddRigidBody(
      /*name*/ "obj_body",
      /*inertia*/ drake::multibody::SpatialInertia<double>::SolidBoxWithMass(
          kPackageMass, kPackageLen, kPackageWidth, kPackageLen));

  geometry::ProximityProperties box_props; 
  geometry::AddContactMaterial(FLAGS_box_diss /* dissipation */, FLAGS_box_pc_stiffness /* point stiffness */, obj_friction, &box_props); 
  const auto box_hydroelastic_modulus = FLAGS_box_modulus;  
  geometry::AddCompliantHydroelasticProperties({}, box_hydroelastic_modulus, &box_props);  
  auto obj_body_collision_geom = plant.RegisterCollisionGeometry(
      /*body*/ obj_body,
      /*X_BG*/ drake::math::RigidTransform<double>(),
      /*shape*/ drake::geometry::Box(kPackageLen, kPackageWidth, kPackageLen),
      /*name*/ "obj_body_collision_geom",
      /*proximity properties*/ box_props);

  plant.RegisterVisualGeometry(
      /*body*/ obj_body,
      /*X_BG*/ drake::math::RigidTransform<double>(),
      /*shape*/ drake::geometry::Box(kPackageLen, kPackageWidth, kPackageLen),
      /*name*/ "obj_body_visual_geom",
      /*diffuse_color*/ Eigen::Vector4d(0.6, 0.4, 0.1, 1.0));

  // Add a wrist
  const double kWristMass = 1.0;
  const double kWristInertia = 0.01;
  const double kWristHeight = 1.0;
  auto& wrist_body = plant.AddRigidBody(
      /*name*/ "wrist_body",
      /*inertia*/ drake::multibody::SpatialInertia<double>(
          kWristMass, Eigen::Vector3d::Zero(),
          drake::multibody::UnitInertia<double>(kWristInertia, kWristInertia,
                                                kWristInertia)));

  plant.AddJoint<drake::multibody::WeldJoint>(
      "wrist_joint", world_body, drake::math::RigidTransform<double>(),
      wrist_body, drake::math::RigidTransform<double>(),
      drake::math::RigidTransform<double>(
          Eigen::Vector3d(0., 0., kWristHeight)));

  // Add a suction gripper
  ExampleGripperMultibodyModel suction_gripper(&plant, wrist_body, FLAGS_cup_modulus);
  auto suction_cup_base_body_id_vec = suction_gripper.get_suction_cup_base_body_id_vec(); 
  auto base_body_geom_id = suction_gripper.get_base_body_geom_id(); 
  auto cup_body_geom_id = suction_gripper.get_cup_body_geom_id(); 

  plant.set_discrete_contact_solver(
      drake::multibody::DiscreteContactSolver::kSap);
  plant.Finalize();

  const double kPumpPressure = -9e4;
  const double kMaxSuctionDist = FLAGS_max_suction_dist; 
  const int kNumSuctionCup = 1;
  const double kSuctionModelTimeStep = 0.01;

  auto& suction_pressure_source = *builder.AddSystem<CupPressureSource>(
      kPumpPressure, kMaxSuctionDist, kNumSuctionCup);

  std::unordered_map<drake::geometry::GeometryId, drake::multibody::BodyIndex>
      obj_geom_id_to_body_idx_map = {
          {obj_body_collision_geom, obj_body.index()}};
  auto& cup_obj_interface = *builder.AddSystem<CupObjInterface>(
      kSuctionModelTimeStep, kMaxSuctionDist, suction_gripper.CalcCupArea(),
      obj_geom_id_to_body_idx_map,
      suction_cup_base_body_id_vec,
      base_body_geom_id, 
      cup_body_geom_id);

  builder.Connect(suction_pressure_source.GetSuctionCupPressureOutputPort(),
                  cup_obj_interface.GetSuctionCupPressureInputPort());
  builder.Connect(cup_obj_interface.GetCupObjDistOutputPort(),
                  suction_pressure_source.GetCupObjDistInputPort());
  builder.Connect(scene_graph.get_query_output_port(),
                  cup_obj_interface.GetGeomQueryInputPort());
  builder.Connect(cup_obj_interface.GetSuctionForceOutputPort(),
                  plant.get_applied_spatial_force_input_port());

  visualization::AddDefaultVisualization(&builder);

  // Add abstract value logger system
  auto& abstract_value_logger = *builder.AddSystem<abstract_logger::AbstractValueLogger>();
  builder.Connect(plant.get_reaction_forces_output_port(), abstract_value_logger.GetReactionForcesInputPort()); 
  builder.Connect(plant.get_contact_results_output_port(), abstract_value_logger.GetContactResultsInputPort());
  
//   // Add logger
//   auto suction_force_logger = LogVectorOutput(cup_obj_interface.GetSingleSuctionForceOutputPort(), &builder, FLAGS_publish_period);
//   auto reaction_forces_logger = LogVectorOutput(abstract_value_logger.GetVectorReactionForceOutputPort(), &builder, FLAGS_publish_period);
//   auto cup_obj_dist_logger = LogVectorOutput(cup_obj_interface.GetCupObjDistOutputPort(), &builder, FLAGS_publish_period);
//   auto pressure_logger = LogVectorOutput(suction_pressure_source.GetSuctionCupPressureOutputPort(), &builder, FLAGS_publish_period);

  // Build diagram 
  auto diagram_ptr = builder.Build();

  drake::systems::Simulator<double> simulator(*diagram_ptr);
  auto& plant_context =
      plant.GetMyMutableContextFromRoot(&simulator.get_mutable_context());

  const double kCupCenterHeightFromGround = 0.6365; // center of entire cup (top and bottom half)
  const double kCupEngageOffset = 0.0; // 0.001 would cause contact with piston 
  const auto initial_angle = (FLAGS_peeloff) ? 1.0472 : FLAGS_initial_angle;
  plant.SetFreeBodyPose(
      &plant_context, obj_body,
      drake::math::RigidTransform<double>(
          drake::math::RollPitchYaw<double>(0., initial_angle, 0.),
          Eigen::Vector3d(-kPackageLen / 2 * sin(initial_angle), 0,
                          kCupCenterHeightFromGround + kCupEngageOffset -
                              kPackageLen / 2 * cos(initial_angle))));

  auto& suction_pressure_source_context =
      suction_pressure_source.GetMyMutableContextFromRoot(
          &simulator.get_mutable_context());
  suction_pressure_source.GetSuctionCmdInputPort().FixValue(
      &suction_pressure_source_context,
      drake::systems::BasicVector<double>({1}));

  simulator.set_target_realtime_rate(FLAGS_realtime_rate);
  simulator.Initialize();

  const auto simulation_time = (FLAGS_peeloff) ? 0.4 : FLAGS_simulation_time;
  if (FLAGS_advance_in_time) simulator.AdvanceTo(simulation_time);

//   // Print number of joints in this plant
//   std::cout << "Number of joints in this plant: " << plant.num_joints() << std::endl; // 4

  // Iterate over all joints in the plant
//   for (int i = 0; i < plant.num_joints(); i++) {
//     const auto& joint = plant.get_joint(drake::multibody::JointIndex(i));
//     std::cout << "Joint " << i << " name: " << joint.name() << std::endl;
//     std::cout << "\t parent: " << joint.parent_body().name() << std::endl;
//     std::cout << "\t child: " << joint.child_body().name() << std::endl;
//     const auto& frame_on_child = joint.frame_on_child();
//     const auto rotation_matrix = frame_on_child.CalcRotationMatrixInWorld(plant_context); 
//     std::cout << "\t rotation matrix is identity?: " << rotation_matrix.IsExactlyIdentity() << std::endl; // 1 is true
//     // world to wrist is identity. so okay to add spatial contact force to reaction force at joint. 
//   }

  // Query contact results
//   auto contact_results = plant.get_contact_results_output_port().Eval<drake::multibody::ContactResults<double>>(plant_context);
//   std::cout << "There are " << contact_results.num_point_pair_contacts() << " point pair contacts." << std::endl; 
//   std::cout << "There are " << contact_results.num_hydroelastic_contacts() << " hydroelastic contacts." << std::endl;
//   for (int i = 0; i < contact_results.num_point_pair_contacts(); i++){
//     const auto point_contact_info = contact_results.point_pair_contact_info(i);
//     std::cout << "body " << point_contact_info.bodyA_index() << " and " << point_contact_info.bodyB_index() << " have contact." << std::endl;
//     const drake::multibody::Body<double>& bodyA = plant.get_body(point_contact_info.bodyA_index());
//     const drake::multibody::Body<double>& bodyB = plant.get_body(point_contact_info.bodyB_index());
//     std::cout << "bodyA name: " << bodyA.name() << std::endl;
//     std::cout << "bodyB name: " << bodyB.name() << std::endl;
//     const auto contact_force = point_contact_info.contact_force(); 
//     std::cout << "contact force: " << contact_force[0] << ", " << contact_force[1] << ", " << contact_force[2]  << std::endl;
//   }
//   for (int i = 0; i < contact_results.num_hydroelastic_contacts(); i++){
//     const auto hydroelastic_contact_info = contact_results.hydroelastic_contact_info(i);
//     const auto contact_force = hydroelastic_contact_info.F_Ac_W();
//     std::cout << "body " << hydroelastic_contact_info.contact_surface().id_M() << " and " << hydroelastic_contact_info.contact_surface().id_N() << " have hydroelastic contact." << std::endl;
//     std::cout << "contact force: " << contact_force << std::endl;
//   }

  // query pressure
  const auto& cup_obj_dist_vec = suction_pressure_source.GetCupObjDistInputPort().Eval<drake::systems::BasicVector<double>>(suction_pressure_source_context);
  std::cout << "[main.cc] distance: " << cup_obj_dist_vec << std::endl;
  const auto& pressure_output = suction_pressure_source.GetSuctionCupPressureOutputPort().Eval<drake::systems::BasicVector<double>>(suction_pressure_source_context);  
  std::cout << "[main.cc] pressure: " << pressure_output << std::endl;

  // Print log data
//   const std::string suffix = (FLAGS_peeloff) ? "_peeloff" : "";
//   SaveLogToCsv("/Volumes/workplace/suction-cup-drake/reaction_forces_data" + suffix + ".csv", reaction_forces_logger->FindLog(simulator.get_context()));
//   SaveLogToCsv("/Volumes/workplace/suction-cup-drake/cup_obj_dist_data" + suffix + ".csv", cup_obj_dist_logger->FindLog(simulator.get_context()));
//   SaveLogToCsv("/Volumes/workplace/suction-cup-drake/pressure_data" + suffix + ".csv", pressure_logger->FindLog(simulator.get_context()));
//   SaveLogToCsv("/Volumes/workplace/suction-cup-drake/examples/multibody/suction_gripper/suction_force.csv", suction_force_logger->FindLog(simulator.get_context()));

  // Print realtime rate
  std::cout << "Realtime rate: " << simulator.get_actual_realtime_rate() << std::endl;

  return EXIT_SUCCESS;
}
}  // namespace drake::examples::multibody::suction_gripper

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A demo showing the package grabbed by a suction cup swinging if "
      "released from a non-zero initial angle.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::multibody::suction_gripper::do_main();
}
