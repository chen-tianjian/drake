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
// #include <drake/multibody/tree/linear_spring_damper.h>
#include "drake/geometry/geometry_ids.h"
#include <drake/geometry/shape_specification.h>


#include <chrono>
#include <thread>
#include <array>
#include <cmath>
#include <Eigen/Dense>


Eigen::Vector3d suction_point_in_bag_frame;


namespace drake {
namespace examples {
namespace multibody {
namespace suction_gripper {
namespace {

DEFINE_double(realtime_rate, 1.0, "realtime rate");

DEFINE_double(cup_modulus, 6.0e5, "hydroelastic modulus of cup");

DEFINE_string(contact_model, "hydroelastic",
                            "Contact model. Options are: 'point', 'hydroelastic', "
                            "'hydroelastic_with_fallback'.");
DEFINE_double(mbp_dt, 0.01,
                            "The fixed time step period (in seconds) of discrete updates "
                            "for the multibody plant modeled as a discrete system. "
                            "Strictly positive.");
DEFINE_bool(publish_contacts, false, "Whether to publish contacts or not");


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


    // Add an object
    const std::string sdf_url =
            "package://drake/examples/multibody/suction_gripper/item_in_bag/item_in_bag.sdf";
    drake::multibody::Parser parser(&plant);
    auto obj_model_instance = parser.AddModelsFromUrl(sdf_url)[0];

    auto& obj_body = plant.GetBodyByName("bag", obj_model_instance);
    auto obj_body_collision_geom = plant.GetCollisionGeometriesForBody(obj_body)[0];
    auto obj_frame_id = plant.GetBodyFrameIdOrThrow(obj_body.index());

    const auto& inspector = scene_graph.model_inspector();
    auto obj_viz_geom_id = inspector.GetGeometryIdByName(obj_frame_id, drake::geometry::Role::kIllustration, "item_in_bag::bag_visual");

    auto& item_body = plant.GetBodyByName("item", obj_model_instance);

//   plant.AddForceElement<drake::multibody::LinearSpringDamper>(
//     obj_body, Eigen::Vector3d(0, 0, 0), item_body,
//     Eigen::Vector3d::Zero(), 0.001, 20., 1.);

    plant.set_discrete_contact_approximation(
            drake::multibody::DiscreteContactApproximation::kSap);

    // Gravity acting in the -z direction.
    plant.mutable_gravity_field().set_gravity_vector(Eigen::Vector3d{0, 0, -9.81});

    plant.Finalize();
    auto plant_source_id = plant.get_source_id().value();


    // --------------------------Motion model--------------------------
    Eigen::VectorXd time_way_pts(7);
    time_way_pts << 0, 1, 2, 3, 5, 6, 7;

    const double kWristToToolTipOffset = 0.357;
    const double kCupEngageOffset = 0.0; // larger penetrates cup further. 0.04 for skew pick. -0.01 normally.
    const double kPlaceHeight = 0.35;
    Eigen::MatrixXd wrist_xz_traj_way_pts(2, 7);
    wrist_xz_traj_way_pts << kWristJointLowerLimit, kWristJointLowerLimit,
            kWristJointLowerLimit, kWristJointLowerLimit, kWristJointUpperLimit,
            kWristJointUpperLimit, kWristJointUpperLimit, kWristJointUpperLimit,
            kWristJointUpperLimit,
            0.2 + kWristToToolTipOffset + kCupEngageOffset,
            kWristJointUpperLimit, kWristJointUpperLimit,
            kPlaceHeight + kWristToToolTipOffset + kCupEngageOffset,
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
    const double kPumpPressure = -7e4; // does not affect penetration depth either, but does cause pick failure
    const double kMaxSuctionDist = 0.004; // 4 mm max suction distance
    const int kNumSuctionCup = 1;
    const double kSuctionModelTimeStep = 0.005;

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

    // Build diagram
    auto diagram_ptr = builder.Build();

    // --------------------------Simulation--------------------------
    drake::systems::Simulator<double> simulator(*diagram_ptr);
    auto& plant_context =
            plant.GetMyMutableContextFromRoot(&simulator.get_mutable_context());

    auto& scene_graph_context =
            scene_graph.GetMyMutableContextFromRoot(&simulator.get_mutable_context());

    // same pose of main package
    plant.SetFreeBodyPose(&plant_context, obj_body,
                                                drake::math::RigidTransform<double>(
                                                        Eigen::Vector3d(0, 0, 0.3 / 2)));


    plant.SetFreeBodyPose(&plant_context, item_body,
                                                drake::math::RigidTransform<double>(
                                                        Eigen::Vector3d(0.001, 0.001, 0.3 / 2)));

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

    double time = 0;
    do {
            time = plant_context.get_time();
            simulator.AdvanceTo(time + FLAGS_mbp_dt);
            auto& suction_point_frame = static_cast<const drake::multibody::FixedOffsetFrame<double>&>(plant.GetFrameByName("frameA", obj_model_instance));
            suction_point_frame.SetPoseInParentFrame(&plant_context, drake::math::RigidTransformd(suction_point_in_bag_frame));

    } while (time < time_way_pts(2));


    // Turn on suction to grab the object
    suction_pressure_source.GetSuctionCmdInputPort().FixValue(
            &suction_pressure_source_context,
            drake::systems::BasicVector<double>({1}));

    do {
            time = plant_context.get_time();
            simulator.AdvanceTo(time + FLAGS_mbp_dt);
            auto& suction_point_frame = static_cast<const drake::multibody::FixedOffsetFrame<double>&>(plant.GetFrameByName("frameA", obj_model_instance));
            suction_point_frame.SetPoseInParentFrame(&plant_context, drake::math::RigidTransformd(suction_point_in_bag_frame)); // bag geom frame

            auto bag_pose = plant.EvalBodyPoseInWorld(plant_context, obj_body);
            auto suction_point_in_world = bag_pose * suction_point_in_bag_frame;

            auto item_pos_in_world = plant.EvalBodyPoseInWorld(plant_context, item_body).translation();

            auto suction_point_to_item_vec = item_pos_in_world - suction_point_in_world;

            Eigen::Matrix< double, 3, 4 >  transform_matrix;
            Eigen::Vector3d z_axis = -suction_point_to_item_vec.normalized();
            Eigen::Vector3d x_axis = Eigen::Vector3d(-z_axis(1) / z_axis.head(2).norm(), z_axis(0) / z_axis.head(2).norm(), 0);
            Eigen::Vector3d y_axis = z_axis.cross(x_axis);

            transform_matrix.col(0) = x_axis;
            transform_matrix.col(1) = y_axis;
            transform_matrix.col(2) = z_axis;
            transform_matrix.col(3) = suction_point_in_bag_frame + 0.6 * suction_point_to_item_vec;


            drake::math::RigidTransformd bag_transform (transform_matrix);

            drake::geometry::Cylinder new_cylinder (0.1, suction_point_to_item_vec.norm() * 1.2);
            scene_graph.ChangeShape(&scene_graph_context, plant_source_id, obj_viz_geom_id, new_cylinder, bag_transform);

    } while (time < time_way_pts(3));


    do {
            time = plant_context.get_time();
            simulator.AdvanceTo(time + FLAGS_mbp_dt);
            auto& suction_point_frame = static_cast<const drake::multibody::FixedOffsetFrame<double>&>(plant.GetFrameByName("frameA", obj_model_instance));
            suction_point_frame.SetPoseInParentFrame(&plant_context, drake::math::RigidTransformd(suction_point_in_bag_frame));

            auto bag_pose = plant.EvalBodyPoseInWorld(plant_context, obj_body);
            auto suction_point_in_world = bag_pose * suction_point_in_bag_frame;

            auto item_pos_in_world = plant.EvalBodyPoseInWorld(plant_context, item_body).translation();

            auto suction_point_to_item_vec = item_pos_in_world - suction_point_in_world;

            Eigen::Matrix< double, 3, 4 >  transform_matrix;
            Eigen::Vector3d z_axis = -suction_point_to_item_vec.normalized();
            Eigen::Vector3d x_axis = Eigen::Vector3d(-z_axis(1) / z_axis.head(2).norm(), z_axis(0) / z_axis.head(2).norm(), 0);
            Eigen::Vector3d y_axis = z_axis.cross(x_axis);

            transform_matrix.col(0) = x_axis;
            transform_matrix.col(1) = y_axis;
            transform_matrix.col(2) = z_axis;
            transform_matrix.col(3) = suction_point_in_bag_frame + 0.6 * suction_point_to_item_vec;


            drake::math::RigidTransformd bag_transform (transform_matrix);

            drake::geometry::Cylinder new_cylinder (0.1, suction_point_to_item_vec.norm() * 1.2);
            scene_graph.ChangeShape(&scene_graph_context, plant_source_id, obj_viz_geom_id, new_cylinder, bag_transform);

    } while (time < time_way_pts(5));


    // Turn off suction to release the object
    suction_pressure_source.GetSuctionCmdInputPort().FixValue(
            &suction_pressure_source_context,
            drake::systems::BasicVector<double>({-0.5}));


    do {
            time = plant_context.get_time();
            simulator.AdvanceTo(time + FLAGS_mbp_dt);
            auto& suction_point_frame = static_cast<const drake::multibody::FixedOffsetFrame<double>&>(plant.GetFrameByName("frameA", obj_model_instance));
            suction_point_frame.SetPoseInParentFrame(&plant_context, drake::math::RigidTransformd());
    } while (time < time_way_pts(6));


    return EXIT_SUCCESS;
}

}  // namespace
}  // namespace suction_gripper
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
    gflags::SetUsageMessage(
            "A demo showing pick and place with a suction gripper.");
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    return drake::examples::multibody::suction_gripper::do_main();
}
