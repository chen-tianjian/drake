#include <memory>
#include <iostream>

#include <gflags/gflags.h>

#include "drake/examples/multibody/suction_gripper/example_gripper_multibody_model.h"
#include "drake/examples/multibody/suction_gripper/suction_force_model.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/contact_solvers/sap/sap_solver.h"
#include "drake/multibody/math/spatial_algebra.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/visualization/visualization_config_functions.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/multibody/plant/multibody_plant_config.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/visualization/visualization_config_functions.h"
#include "drake/systems/primitives/vector_log_sink.h"
#include "drake/multibody/parsing/parser.h"

#include <chrono>
#include <thread>
#include <array>
#include <cmath>
#include <Eigen/Dense>

DEFINE_double(realtime_rate, 1.0, "realtime rate"); 

DEFINE_double(box_modulus, 3.0e5, "hydroelastic modulus of box");
DEFINE_double(cup_modulus, 3.0e5, "hydroelastic modulus of cup");

DEFINE_string(contact_model, "hydroelastic_with_fallback",
              "Contact model. Options are: 'point', 'hydroelastic', "
              "'hydroelastic_with_fallback'.");
DEFINE_double(mbp_dt, 0.002,
              "The fixed time step period (in seconds) of discrete updates "
              "for the multibody plant modeled as a discrete system. "
              "Strictly positive.");
DEFINE_bool(vis, true, "Visualize frames or not");
DEFINE_bool(advance_in_time, true, "Whether to advance sim or not");
DEFINE_bool(publish_contacts, true, "Whether to publish contacts or not");
DEFINE_int32(num_packages, 0, "Number of dummy packages to add to decrease RTR");

DEFINE_double(publish_period, 0.005,
              "How often to publish logger.");

DEFINE_bool(skew_pick, false, "Whether to try to pick at 30 deg or not");

namespace drake::examples::multibody::suction_gripper {


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

std::vector<drake::multibody::ModelInstanceIndex> AddPackages(drake::multibody::Parser& parser, const int num_packages){
    std::vector<drake::multibody::ModelInstanceIndex> package_indices;
    for (int i = 0; i < num_packages; ++i) {
        auto package_model = parser.AddModels("examples/multibody/suction_gripper/0_18B_hydroelastic.sdf")[0];
        package_indices.push_back(package_model);
    }

    return package_indices; 
}

const std::string MESHCAT_PREFIX = "/drake/proximity";
// path in gemini would be gemini/floating_objects/0_09B#0/base_link/coordinate_frame

std::string DrawTriad(std::shared_ptr<geometry::Meshcat> mc, const std::string& path, double radius, double length, double opacity) {
    const double cone_length = length / 5;
    const double m_line_length = length - cone_length;
    const double line_offset = m_line_length / 2;
    auto line_coords = Eigen::Matrix<double, 3, 3>::Identity() * line_offset;
    auto cone_coords = Eigen::Matrix<double, 3, 3>::Identity() * length;

    // cylinders are drawn z up so the z-line needs no rotation
    const std::vector<Eigen::Vector3d> line_rotation_axes = {Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitX(),
                                                             Eigen::Vector3d::UnitZ()};
    // cones are drawn z down so they all require rotation
    const std::vector<Eigen::Vector3d> cone_rotation_axes = {Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitX(),
                                                             Eigen::Vector3d::UnitY()};
    const std::array<double, 3> cone_angles{{-M_PI_2, M_PI_2, M_PI}};
    Eigen::Array<double, 3, 4> colors; // RGB-XYZ
    colors << 1, 0, 0, opacity, 0, 1, 0, opacity, 0, 0, 1, opacity;
    const std::string frame_name = path + "/coordinate_frame";
    std::array<std::string, 3> names{{frame_name + "/x", frame_name + "/y", frame_name + "/z"}};
    for (int i = 0; i < 3; ++i) {
        const drake::geometry::Rgba color{colors.coeff(i, 0), colors.coeff(i, 1), colors.coeff(i, 2), colors.coeff(i, 3)};
        mc->SetObject(names[i] + "/line", drake::geometry::Cylinder(radius, m_line_length), color);
        mc->SetTransform(names[i] + "/line", {Eigen::AngleAxis{M_PI_2, line_rotation_axes[i]}, line_coords.row(i)});

        mc->SetObject(names[i] + "/arrow", drake::geometry::MeshcatCone{cone_length, radius * 2, radius * 2}, color);
        mc->SetTransform(names[i] + "/arrow", {Eigen::AngleAxis{cone_angles[i], cone_rotation_axes[i]}, cone_coords.row(i)});
    }
    return frame_name;
}

// incremented for every free frame that gets added so they have a unique name
static int free_frame_idx = 0;

template <typename T>
std::string GetFrameNameFromBodyIndex(const drake::multibody::MultibodyPlant<T>& plant, const drake::geometry::SceneGraph<T>& scene_graph,
                                      const drake::multibody::BodyIndex& body_index) {
    const auto& inspector = scene_graph.model_inspector();
    auto frame_id = plant.GetBodyFrameIdOrThrow(body_index);
    if (frame_id == inspector.world_frame_id()) { return ""; }
    return inspector.GetName(frame_id);
}

template <typename T>
std::string GetFrameNameFromGeometryId(const drake::geometry::SceneGraph<T>& scene_graph, const drake::geometry::GeometryId& geometry_id) {
    const auto& inspector = scene_graph.model_inspector();
    auto frame_id = inspector.GetFrameId(geometry_id);
    if (frame_id == inspector.world_frame_id()) { return ""; }
    return inspector.GetName(frame_id);
}

/**
 * @brief Builds a meshcat scene tree name from the scene inspector path inside Drake. This method works on not fully constructed
 *        plants and scene_graphs. I.e. we can use it in Station finalize.
 * @note Path always ends in a concrete name, it never ends with a "/"
 * @tparam T the Drake primitive type for the plant and scenegraph.
 * @param plant A multibody plant containing the body index.
 * @param scene_graph A scene graph to query for bodies parent frames.
 * @param body_index The index of the body from the parent frame.
 * @return
 */
template <typename T>
std::string GetMeshcatPathForParentBody(const drake::multibody::MultibodyPlant<T>& plant, const drake::geometry::SceneGraph<T>& scene_graph,
                                        const drake::multibody::BodyIndex& body_index) {
    auto frame_name = GetFrameNameFromBodyIndex(plant, scene_graph, body_index);
    std::stringstream visualizer_name;
    if (frame_name.empty()) {
        visualizer_name << MESHCAT_PREFIX;
    } else {
        size_t pos;
        while ((pos = frame_name.find("::")) != std::string::npos) { frame_name.replace(pos, 2, "/"); }
        visualizer_name << MESHCAT_PREFIX << "/" << frame_name;
    }
    return visualizer_name.str();
}

/**
 * Draw a cooridnate frame in meshcat visualizer
 * @note drake::geometry::Meshcat is not thread safe so this function must be called from the same thread where the Meshcat class
 * instance was constructed.
 * @param mc pointer to the drake::geometry::Meshcat instance to draw with
 * @param parent @ref drake::geometry::meshcat::meshcat_path path to parent frame e.g. /gemini/arm/link_4. Pass empty string " to
 * draw frame relative to world coordinates
 * @param radius the radius/thickness of the triad components
 * @param length the length of the drawn red green blue triad
 * @param opacity transparency [0-1]
 * @param xform fixed transform from parent body to the coordinate frame marker
 * @throws if length or radius is negative or opacity is out of bounds [0-1]
 * @return The meshcat_path of the resultant frame. Useful for calling drake::geometry::Meshcat::Delete on.
 */
std::string DrawFrame(std::shared_ptr<geometry::Meshcat> mc, const std::string& parent, double radius, double length, double opacity,
                                   const drake::math::RigidTransformd& xform = {}) {
    std::string frame_path;
    // If a frame has a parent, do not append an index which will limit one drawn frame per body
    if (parent != "") {
        frame_path = parent;
    } else {
        std::cout << " frame doesn't have a parent " << std::endl; 
        std::string frame_name = "frame" + std::to_string(free_frame_idx++);
        // frame_path = PathJoin(MESHCAT_PREFIX, frame_name);
    }
    const auto path = DrawTriad(mc, frame_path, radius, length, opacity);
    mc->SetTransform(path, xform);
    return path;
}


int do_main() {
  // All values are in SI units.
  // --------------------------Multibody model--------------------------
  drake::systems::DiagramBuilder<double> builder;

  drake::multibody::MultibodyPlantConfig config;
  // We allow only discrete systems.
  DRAKE_DEMAND(FLAGS_mbp_dt > 0.0);
  config.time_step = FLAGS_mbp_dt;
  config.contact_model = FLAGS_contact_model;
  auto [plant, scene_graph] = drake::multibody::AddMultibodyPlant(config, &builder);

  auto& world_body = plant.world_body();

  // Add two conveyor belts
  const double kConveyorMass = 100.;
  const double kConveyorLen = 5.;
  const double kConveyorWidth = 0.6;
  const double kConveyorHeight = 0.2;
  const double kFrictionCoeff = 0.5; // https://wiki.unece.org/display/TransportSustainableCTUCode/Appendix+2.%09Friction+factors

  auto& conveyor1_body = plant.AddRigidBody(
      /*name*/ "conveyor1_body",
      /*inertia*/ drake::multibody::SpatialInertia<double>(
          /*m*/ kConveyorMass,
          /*p_PScm_E*/ Eigen::Vector3d::Zero(),
          /*G_SP_E*/ drake::multibody::UnitInertia<double>(1., 1., 1.)));

  plant.RegisterVisualGeometry(
      /*body*/ conveyor1_body,
      /*X_BG*/ drake::math::RigidTransform<double>(),
      /*shape*/
      drake::geometry::Box(kConveyorWidth, kConveyorLen, kConveyorHeight),
      /*name*/ "conveyor1_visual_geom",
      /*diffuse_color*/ Eigen::Vector4d(0.2, 0.2, 0.2, 1.));

    geometry::ProximityProperties conveyor1_props;
    geometry::AddContactMaterial({} /* dissipation */, {} /* point stiffness */, drake::multibody::CoulombFriction(
          /*static_friction */ kFrictionCoeff,
          /* dynamic_friction*/ kFrictionCoeff), &conveyor1_props); 
    const auto conveyor_hydroelastic_modulus = 1e6;  
    geometry::AddCompliantHydroelasticProperties({}, conveyor_hydroelastic_modulus, &conveyor1_props);  
    plant.RegisterCollisionGeometry(
      /*body*/ conveyor1_body,
      /*X_BG*/ drake::math::RigidTransform<double>(),
      /*shape*/
      drake::geometry::Box(kConveyorWidth, kConveyorLen, kConveyorHeight),
      /*name*/ "conveyor1_collision_goem",
      /*proximity properties*/ conveyor1_props);

  plant.WeldFrames(world_body.body_frame(), conveyor1_body.body_frame(),
                   /*X_AB*/
                   drake::math::RigidTransform<double>(
                       Eigen::Vector3d(0, 0, -kConveyorHeight / 2)));

  auto& conveyor2_body = plant.AddRigidBody(
      /*name*/ "conveyor2_body",
      /*inertia*/ drake::multibody::SpatialInertia<double>(
          /*m*/ kConveyorMass,
          /*p_PScm_E*/ Eigen::Vector3d::Zero(),
          /*G_SP_E*/ drake::multibody::UnitInertia<double>(1., 1., 1.)));

  plant.RegisterVisualGeometry(
      /*body*/ conveyor2_body,
      /*X_BG*/ drake::math::RigidTransform<double>(),
      /*shape*/
      drake::geometry::Box(kConveyorWidth, kConveyorLen, kConveyorHeight),
      /*name*/ "conveyor2_visual_geom",
      /*diffuse_color*/ Eigen::Vector4d(0.2, 0.2, 0.2, 1.));

    geometry::ProximityProperties conveyor2_props;
    geometry::AddContactMaterial({} /* dissipation */, {} /* point stiffness */, drake::multibody::CoulombFriction(
          /*static_friction */ kFrictionCoeff,
          /* dynamic_friction*/ kFrictionCoeff), &conveyor2_props); 
    geometry::AddCompliantHydroelasticProperties({}, conveyor_hydroelastic_modulus, &conveyor2_props);  
    plant.RegisterCollisionGeometry(
      /*body*/ conveyor2_body,
      /*X_BG*/ drake::math::RigidTransform<double>(),
      /*shape*/
      drake::geometry::Box(kConveyorWidth, kConveyorLen, kConveyorHeight),
      /*name*/ "conveyor2_collision_goem",
      /*proximity properties*/ conveyor2_props);

  const double kConveyorDist = 1.;
  plant.WeldFrames(world_body.body_frame(), conveyor2_body.body_frame(),
                   /*X_AB*/
                   drake::math::RigidTransform<double>(Eigen::Vector3d(
                       kConveyorDist, 0, -kConveyorHeight / 2)));

  // Add an object (cardboard box)
  const double kPackageMass = 2.73;
  const double kPackageLen = 0.3;
  const double kPackageWidth = 0.15;
  drake::multibody::CoulombFriction obj_friction(
      /*static_friction */ kFrictionCoeff,
      /* dynamic_friction*/ kFrictionCoeff);

  auto& obj_body = plant.AddRigidBody(
      /*name*/ "obj_body",
      /*inertia*/ drake::multibody::SpatialInertia<double>::SolidBoxWithMass(
          kPackageMass, kPackageLen, kPackageWidth, kPackageLen));

  geometry::ProximityProperties box_props; 
  geometry::AddContactMaterial(0.0 /* dissipation */, 5e3 /* point stiffness */, obj_friction, &box_props); 
  const auto box_hydroelastic_modulus = FLAGS_box_modulus;  // needs to be at least 1e6
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
  auto& wrist_body1 = plant.AddRigidBody(
      /*name*/ "wrist_body1",
      /*inertia*/ drake::multibody::SpatialInertia<double>(
          kWristMass, Eigen::Vector3d::Zero(),
          drake::multibody::UnitInertia<double>(kWristInertia, kWristInertia,
                                                kWristInertia)));

  const double kWristJointLowerLimit = 0.;
  const double kWristJointUpperLimit = 1.;
  const double kWristJointDamping = 10.;
  auto& wrist_joint1 = plant.AddJoint<drake::multibody::PrismaticJoint>(
      /*name*/ "wrist_joint1",
      /*parent*/ world_body,
      /*X_PF*/ drake::math::RigidTransform<double>(), // frame on parent
      /*child*/ wrist_body1,
      /*X_BM*/ drake::math::RigidTransform<double>(), // frame on child 
      /*axis*/ Eigen::Vector3d::UnitX(), // how the frames move relative to one another 
      /*pos_lower_limit*/ kWristJointLowerLimit,
      /*pos_upper_limit*/ kWristJointUpperLimit,
      /*damping*/ kWristJointDamping);
  plant.AddJointActuator("wrist_joint1_actuator", wrist_joint1);

  auto& wrist_body2 = plant.AddRigidBody(
      /*name*/ "wrist_body2",
      /*inertia*/ drake::multibody::SpatialInertia<double>(
          kWristMass, Eigen::Vector3d::Zero(),
          drake::multibody::UnitInertia<double>(kWristInertia, kWristInertia,
                                                kWristInertia)));

  auto& wrist_joint2 = plant.AddJoint<drake::multibody::PrismaticJoint>(
      /*name*/ "wrist_joint2",
      /*parent*/ wrist_body1,
      /*X_PF*/ drake::math::RigidTransform<double>(),
      /*child*/ wrist_body2,
      /*X_BM*/ drake::math::RigidTransform<double>(),
      /*axis*/ Eigen::Vector3d::UnitZ(),
      /*pos_lower_limit*/ kWristJointLowerLimit,
      /*pos_upper_limit*/ kWristJointUpperLimit,
      /*damping*/ kWristJointDamping);
  plant.AddJointActuator("wrist_joint2_actuator", wrist_joint2);

  // Add a suction gripper
  ExampleGripperMultibodyModel suction_gripper(&plant, wrist_body2, FLAGS_cup_modulus); 
  // Defined in example_gripper_multibody_model.h. Inherits from suction_gripper_multibody_model.h. 
  auto suction_cup_base_body_id_vec = suction_gripper.get_suction_cup_base_body_id_vec(); 
  auto base_body_geom_id = suction_gripper.get_base_body_geom_id(); 
  auto cup_body_geom_id = suction_gripper.get_cup_body_geom_id(); 

  // Add more packages
  drake::multibody::Parser parser(&plant);
  const auto packages= AddPackages(parser, FLAGS_num_packages);

  plant.set_discrete_contact_approximation(
      drake::multibody::DiscreteContactApproximation::kSap);

  // Gravity acting in the -z direction.
  plant.mutable_gravity_field().set_gravity_vector(Eigen::Vector3d{0, 0, -9.81});

  plant.Finalize();

  std::cout << "The contact time scale tc is: " << plant.get_contact_penalty_method_time_scale() << std::endl; // 0.01. want time step < 1/10th of this. 
  // relative velocity -> 0 in this time scale. 

  // --------------------------Motion model--------------------------
  Eigen::VectorXd time_way_pts(7);
  time_way_pts << 0, 2, 4, 6, 10, 12, 14;

  const double kWristToToolTipOffset = 0.357;
  const double kCupEngageOffset = -0.01; // larger penetrates cup further. 0.04 for skew pick. -0.01 normally. 
  const double kPlaceHeight = 0.35;
  Eigen::MatrixXd wrist_xz_traj_way_pts(2, 7);
  wrist_xz_traj_way_pts << kWristJointLowerLimit, kWristJointLowerLimit,
      kWristJointLowerLimit, kWristJointLowerLimit, kWristJointUpperLimit,
      kWristJointUpperLimit, kWristJointUpperLimit, kWristJointUpperLimit,
      kWristJointUpperLimit,
      kPackageLen + kWristToToolTipOffset - kCupEngageOffset,
      kWristJointUpperLimit, kWristJointUpperLimit,
      kPlaceHeight + kWristToToolTipOffset - kCupEngageOffset,
      kWristJointUpperLimit; // x and then z     

  auto wrist_xz_traj =
      drake::trajectories::PiecewisePolynomial<double>::CubicShapePreserving(
          time_way_pts, wrist_xz_traj_way_pts);
  auto& wrist_xz_traj_src =
      *builder.AddSystem<drake::systems::TrajectorySource<double>>(
          wrist_xz_traj, 1);

  auto& pid =
      *builder.AddSystem<drake::systems::controllers::PidController<double>>(
          /*state selection matrix*/ plant.MakeStateSelectorMatrix(
              {wrist_joint1.index(), wrist_joint2.index()}),
          /*Kp*/ Eigen::VectorXd::Constant(2, 1, 1e4),
          /*Ki*/ Eigen::VectorXd::Constant(2, 1, 1e3),
          /*Kd*/ Eigen::VectorXd::Constant(2, 1, 1e2));

  builder.Connect(wrist_xz_traj_src.get_output_port(),
                  pid.get_input_port_desired_state());
  builder.Connect(plant.get_state_output_port(),
                  pid.get_input_port_estimated_state());
  builder.Connect(
      pid.get_output_port_control(),
      plant.get_actuation_input_port(drake::multibody::ModelInstanceIndex(1)));

  // --------------------------Suction model--------------------------
  const double kPumpPressure = -9e4; // does not affect penetration depth either, but does cause pick failure 
  const double kMaxSuctionDist = 0.004; // 4 mm max suction distance 
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

  auto meshcat = std::make_shared<geometry::Meshcat>();
  visualization::ApplyVisualizationConfig(
      visualization::VisualizationConfig{
          .default_proximity_color = geometry::Rgba{1, 0, 0, 0.25},
          .publish_contacts = FLAGS_publish_contacts,
          .delete_on_initialization_event = true, 
          .enable_alpha_sliders = true,
      },
      &builder, nullptr, nullptr, nullptr, meshcat);

  // Add logger
//   auto suction_force_logger = LogVectorOutput(cup_obj_interface.GetSingleSuctionForceOutputPort(), &builder, FLAGS_publish_period);

  // Build diagram
  auto diagram_ptr = builder.Build();

  // Visualize the diagram.
//   const auto graphvizstring = diagram_ptr->GetGraphvizString(); 
//   std::cout << graphvizstring << std::endl;

  // --------------------------Simulation--------------------------
  drake::systems::Simulator<double> simulator(*diagram_ptr);
  auto& plant_context =
      plant.GetMyMutableContextFromRoot(&simulator.get_mutable_context());

  // same pose of main package
  plant.SetFreeBodyPose(&plant_context, obj_body,
                        drake::math::RigidTransform<double>(
                            Eigen::Vector3d(0, 0, kPackageLen / 2)));

  // set pose of additional packages
  uint idx = 0;
  uint odd_idx = 0;
  uint even_idx = 0;
  const auto kBoxHeight = 0.2; 
  for (const auto& i : packages){
    const auto body_idx = plant.GetBodyIndices(i);
    const auto& body = plant.get_body(body_idx[0]);

    if (idx % 2 == 0){
        plant.SetFreeBodyPose(&plant_context, body, 
                            drake::math::RigidTransform<double>(
                                Eigen::Vector3d(0, -0.2, kBoxHeight / 2 + kBoxHeight*even_idx)));
        even_idx++;
    }else{
        plant.SetFreeBodyPose(&plant_context, body, 
                            drake::math::RigidTransform<double>(
                                Eigen::Vector3d(0, 0.2, kBoxHeight / 2 + kBoxHeight*odd_idx)));
        odd_idx++;
    }
    idx++; 
  }

  std::cout << "initial condition wrist joint 1: " << wrist_xz_traj_way_pts(0, 0) << std::endl; // 0 
  std::cout << "initial condition wrist joint 2: " << wrist_xz_traj_way_pts(1, 0) << std::endl; // 1
  wrist_joint1.set_translation(&plant_context, wrist_xz_traj_way_pts(0, 0)); 
  wrist_joint2.set_translation(&plant_context, wrist_xz_traj_way_pts(1, 0)); // row 2 

  auto& suction_pressure_source_context =
      suction_pressure_source.GetMyMutableContextFromRoot(
          &simulator.get_mutable_context());
  suction_pressure_source.GetSuctionCmdInputPort().FixValue(
      &suction_pressure_source_context,
      drake::systems::BasicVector<double>({0}));

  simulator.set_target_realtime_rate(FLAGS_realtime_rate);
  simulator.Initialize();

  // Get initial contact info 
  auto contact_results = plant.get_contact_results_output_port().Eval<drake::multibody::ContactResults<double>>(plant_context);
  std::cout << "\nInitially, there are " << contact_results.num_hydroelastic_contacts() << " hydroelastic contacts." << std::endl;
     for (int i = 0; i < contact_results.num_hydroelastic_contacts(); i++){
        const auto hydroelastic_contact_info = contact_results.hydroelastic_contact_info(i);
        const auto geometry_m = hydroelastic_contact_info.contact_surface().id_M();
        const auto geometry_n = hydroelastic_contact_info.contact_surface().id_N();
        // std::cout << "\tHydroelastic contact between geometry indices " << geometry_m << " and " << geometry_n << std::endl;
        std::cout << "\tHydroelastic contact between: " << GetFrameNameFromGeometryId(scene_graph, geometry_m) << " and " << 
        GetFrameNameFromGeometryId(scene_graph, geometry_n) << std::endl; 
    }
  std::cout << "And there are " << contact_results.num_point_pair_contacts() << " point pair contacts." << std::endl;
    for (int i = 0; i < contact_results.num_point_pair_contacts(); i++){
        const auto point_pair_contact_info = contact_results.point_pair_contact_info(i);
        // std::cout << "\tPoint contact between body indices" << point_pair_contact_info.bodyA_index() << " and " << point_pair_contact_info.bodyB_index() << std::endl; 
        std::cout << "\tPoint contact between: " << GetFrameNameFromBodyIndex(plant, scene_graph, point_pair_contact_info.bodyA_index()) << " and " << 
        GetFrameNameFromBodyIndex(plant, scene_graph, point_pair_contact_info.bodyB_index()) << std::endl; 
    }

  if (FLAGS_advance_in_time){
    simulator.AdvanceTo(time_way_pts(2)); // 4 s. when making contact with object. 

    // Turn on suction to grab the object
    suction_pressure_source.GetSuctionCmdInputPort().FixValue(
        &suction_pressure_source_context,
        drake::systems::BasicVector<double>({1}));

    simulator.AdvanceTo(time_way_pts(3)); // pick retract waypoint. 6 s. 

    if (!FLAGS_skew_pick){
        contact_results = plant.get_contact_results_output_port().Eval<drake::multibody::ContactResults<double>>(plant_context);
        std::cout << "\nAfter reaching pick retract wayoint, there are " << contact_results.num_hydroelastic_contacts() << " hydroelastic contacts." << std::endl;
        for (int i = 0; i < contact_results.num_hydroelastic_contacts(); i++){
            const auto hydroelastic_contact_info = contact_results.hydroelastic_contact_info(i);
            const auto geometry_m = hydroelastic_contact_info.contact_surface().id_M();
            const auto geometry_n = hydroelastic_contact_info.contact_surface().id_N();
            // std::cout << "\tHydroelastic contact between geometry indices " << geometry_m << " and " << geometry_n << std::endl;
            std::cout << "\tHydroelastic contact between: " << GetFrameNameFromGeometryId(scene_graph, geometry_m) << " and " << 
            GetFrameNameFromGeometryId(scene_graph, geometry_n) << std::endl; 
            std::cout << "\tThe hydroelastic force is: " << hydroelastic_contact_info.F_Ac_W() << std::endl;
            std::cout << "\tThe number of faces on the contact surface is: " << hydroelastic_contact_info.contact_surface().num_faces() << std::endl; 
            std::cout << "\tThe hydroelastic contact area is: " << hydroelastic_contact_info.contact_surface().total_area() << std::endl; 
            const double CupOuterDiameter = 0.03;
            const double estimated_contact_area = M_PI * CupOuterDiameter * CupOuterDiameter / 4.0;
            const double desired_pen = 0.0119; 
            const double grad_in_cup = 2*FLAGS_cup_modulus/0.027; 
            const double grad_in_box = 2*FLAGS_box_modulus/kPackageLen; 
            const double combined_grad = grad_in_box*grad_in_cup/(grad_in_box+grad_in_cup);
            std::cout << "\tIn comparison, the cup area is: " << estimated_contact_area << std::endl; 
            std::cout << "\tBack of envelope calc for 0.01 penetration = " << desired_pen*estimated_contact_area*combined_grad << std::endl; 
            // hydroelastic_contact_info.contact_surface().

            // check pressure at contact surface
            const auto representation = hydroelastic_contact_info.contact_surface().representation(); 
            if (representation == geometry::HydroelasticContactRepresentation::kTriangle){
                std::cout << "\tRepresented as triangular mesh" << std::endl; 
                const auto& pressure = hydroelastic_contact_info.contact_surface().tri_e_MN();
                (void) pressure; 

            }else{
                std::cout << "\tRepresented as polygon mesh" << std::endl; 
                const auto& pressure = hydroelastic_contact_info.contact_surface().poly_e_MN();
                (void) pressure; 
            }
        }
        std::cout << "And there are " << contact_results.num_point_pair_contacts() << " point pair contacts." << std::endl;
        for (int i = 0; i < contact_results.num_point_pair_contacts(); i++){
            const auto point_pair_contact_info = contact_results.point_pair_contact_info(i);
            // std::cout << "\tPoint contact between body indices" << point_pair_contact_info.bodyA_index() << " and " << point_pair_contact_info.bodyB_index() << std::endl; 
            std::cout << "\tPoint contact between: " << GetFrameNameFromBodyIndex(plant, scene_graph, point_pair_contact_info.bodyA_index()) << " and " << 
            GetFrameNameFromBodyIndex(plant, scene_graph, point_pair_contact_info.bodyB_index()) << std::endl; 
            std::cout << "\tThe point contact force is: " << point_pair_contact_info.contact_force()[0] << " " << point_pair_contact_info.contact_force()[1] << " " 
            << point_pair_contact_info.contact_force()[2] << std::endl; // f_Bc_W on B at contact point C expressed in the world frame W
            std::cout <<"\tThe slip speed is: " << point_pair_contact_info.slip_speed() << std::endl; // how much slip is allowed. 1.9e-6 m/s
            std::cout << "\tThe separation speed is: " << point_pair_contact_info.separation_speed() << std::endl; // how fast moving apart. mag. of 3.9e-4 m/s 
            std::cout << "\tThe penetration depth is: " << point_pair_contact_info.point_pair().depth << std::endl; // 0.00887451
        }

        // simulator.AdvanceTo(time_way_pts(5));
        // // Turn off suction to release the object
        // suction_pressure_source.GetSuctionCmdInputPort().FixValue(
        //     &suction_pressure_source_context,
        //     drake::systems::BasicVector<double>({0}));

        // simulator.AdvanceTo(time_way_pts(6));
    }
  }else{
    // Wait several seconds
    std::this_thread::sleep_for(std::chrono::seconds(10));
  }

//   SaveLogToCsv("/Volumes/workplace/suction-cup-drake/examples/multibody/suction_gripper/suction_force.csv", suction_force_logger->FindLog(simulator.get_context()));

  return EXIT_SUCCESS;
}
}  // namespace drake::examples::multibody::suction_gripper

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A demo showing pick and place with a suction gripper.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::multibody::suction_gripper::do_main();
}
