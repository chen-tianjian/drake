#include "drake/examples/multibody/suction_gripper/suction_force_model.h"

#include <algorithm>
#include <limits>

#include "drake/geometry/query_object.h"
#include "drake/geometry/query_results/signed_distance_pair.h"
#include <iostream>

namespace drake::examples::multibody::suction_gripper {

// ------------------- ExternallyAppliedSpatialForcePair -------------------

std::pair<drake::multibody::ExternallyAppliedSpatialForce<double>,
          drake::multibody::ExternallyAppliedSpatialForce<double>>
ExternallyAppliedSpatialForcePair::GetAsPair() {
  std::pair<drake::multibody::ExternallyAppliedSpatialForce<double>,
            drake::multibody::ExternallyAppliedSpatialForce<double>>
      pair;

  pair.first.body_index = body_index1_;
  pair.first.p_BoBq_B = p_BoBq_B1_;
  pair.first.F_Bq_W = drake::multibody::SpatialForce<double>(
      /*tau*/ trq_axis_ * tau_mag_, /*f*/ force_axis_ * f_mag_);

  pair.second.body_index = body_index2_;
  pair.second.p_BoBq_B = p_BoBq_B2_;
  pair.second.F_Bq_W = drake::multibody::SpatialForce<double>(
      /*tau*/ -trq_axis_ * tau_mag_, /*f*/ -force_axis_ * f_mag_);
  return pair;
}

// ------------------- CupPressureSource -------------------

CupPressureSource::CupPressureSource(double vacuum_source_pressure,
                                     double max_suction_dist,
                                     int num_suction_cups)
    : vacuum_source_pressure_(vacuum_source_pressure),
      max_suction_dist_(max_suction_dist),
      num_suction_cups_(num_suction_cups) {
  /// ----- Input Ports ----- ///
  suction_cup_obj_dist_input_port_idx_ =
      DeclareVectorInputPort(
          "cup_obj_dists",
          drake::systems::BasicVector<double>(num_suction_cups_))
          .get_index();
  suction_cmd_input_port_idx_ =
      DeclareVectorInputPort(
          "suction_cmds",
          drake::systems::BasicVector<double>(num_suction_cups_))
          .get_index();

  /// ----- Output Ports ----- ///
  suction_cup_pressure_output_port_idx_ =
      DeclareVectorOutputPort(
          "suction_cup_pressures",
          drake::systems::BasicVector<double>(num_suction_cups_),
          &CupPressureSource::CalcSuctionCupPressure,
          {all_input_ports_ticket()})
          .get_index();
}

void CupPressureSource::CalcSuctionCupPressure(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* suction_cup_pressure_ptr) const {
  const auto& cup_obj_dist_vec =
      GetCupObjDistInputPort().Eval<drake::systems::BasicVector<double>>(
          context);
  const auto& suction_cmd_vec =
      GetSuctionCmdInputPort().Eval<drake::systems::BasicVector<double>>(
          context);

  for (int suction_cup_idx = 0; suction_cup_idx < num_suction_cups_;
       suction_cup_idx++) {
    double suction_cmd = suction_cmd_vec[suction_cup_idx];
    // DRAKE_DEMAND(suction_cmd >= 0. && suction_cmd <= 1.);
    double dist = cup_obj_dist_vec[suction_cup_idx];
    double pressure = 0.;
    // use a simple linear pressure-distance model
    if (dist <= 0.) {
      pressure = vacuum_source_pressure_;
    } else if (dist <= max_suction_dist_) {
      pressure = (max_suction_dist_ - dist) * vacuum_source_pressure_ /
                 max_suction_dist_; 
    } else {
      pressure = 0.;
    }
    (*suction_cup_pressure_ptr)[suction_cup_idx] = suction_cmd * pressure;
  }
}

// ------------------- CupObjInterface -------------------
CupObjInterface::CupObjInterface(
    double time_step, double max_suction_dist, double suction_cup_area,
    const std::unordered_map<drake::geometry::GeometryId,
                             drake::multibody::BodyIndex>&
        obj_geom_id_to_body_idx_map,
    const std::vector<drake::multibody::BodyIndex>& suction_cup_base_body_id_vec,
    const drake::geometry::GeometryId base_body_geom_id,
    const drake::geometry::GeometryId cup_body_geom_id)
    : max_suction_dist_(max_suction_dist),
      suction_cup_area_(suction_cup_area),
      obj_geom_id_to_body_idx_map_(obj_geom_id_to_body_idx_map),
      suction_cup_base_body_id_vec_(suction_cup_base_body_id_vec),
      base_body_geom_id_(base_body_geom_id), 
      cup_body_geom_id_(cup_body_geom_id),
      num_suction_cups_(suction_cup_base_body_id_vec.size()) {
  /// ----- Input Ports ----- ///
  geom_query_input_port_idx_ =
      DeclareAbstractInputPort(
          "geom_query", drake::Value<drake::geometry::QueryObject<double>>())
          .get_index();

  suction_cup_pressure_input_port_idx_ =
      DeclareVectorInputPort(
          "suction_cup_pressures",
          drake::systems::BasicVector<double>(num_suction_cups_))
          .get_index();

  /// ----- Output Ports ----- ///
  suction_force_output_port_idx_ =
      DeclareAbstractOutputPort(
          /*name*/ "suction_forces",
          /*alloc_function*/
          std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>(
              2 * num_suction_cups_), // if num_suction_cups_ = 1, then of size 2 
          /*calc_function*/ &CupObjInterface::CalcSuctionForce,
          /*dependency*/
          {xa_ticket(), xd_ticket(),
           input_port_ticket(drake::systems::InputPortIndex(
               suction_cup_pressure_input_port_idx_))})
          .get_index();

  single_suction_force_output_port_idx_ = 
      DeclareVectorOutputPort(
        "suction_force_vector", 
        drake::systems::BasicVector<double>(3), 
        &CupObjInterface::CalcSingleSuctionForce,
          /*dependency*/
          {xa_ticket(), xd_ticket(),
           input_port_ticket(drake::systems::InputPortIndex(
               suction_cup_pressure_input_port_idx_))})
          .get_index();

  suction_cup_obj_dist_output_port_idx_ =
      DeclareVectorOutputPort(
          /*name*/ "cup_obj_dists",
          /*alloc_function*/
          drake::systems::BasicVector<double>(num_suction_cups_),
          /*calc_function*/ &CupObjInterface::OutputCupObjDist,
          /*dependency*/ {xa_ticket(), xd_ticket()})
          .get_index();

  /// ----- States ----- ///
  // each entry is a signed dist pair between the center action point and the
  // closest object
  suction_cup_base_bdy_qry_pt_closest_obj_signed_dist_to_pt_state_idx_ =
      DeclareAbstractState(
          drake::Value<
              std::vector<drake::geometry::SignedDistanceToPoint<double>>>(
              std::vector<drake::geometry::SignedDistanceToPoint<double>>(
                  num_suction_cups_)));

  // each entry is the mean distance between the suction cup edge points to the
  // closest object
  suction_cup_edge_pt_closest_obj_dist_state_idx_ =
      DeclareDiscreteState(num_suction_cups_);

  DeclareInitializationUnrestrictedUpdateEvent(&CupObjInterface::UpdateDists);
  DeclarePeriodicUnrestrictedUpdateEvent(time_step, 0,
                                         &CupObjInterface::UpdateDists);
}

drake::systems::EventStatus CupObjInterface::UpdateDists(
    const drake::systems::Context<double>& context,
    drake::systems::State<double>* state_ptr) const {
  const auto& geom_query =
      GetGeomQueryInputPort().Eval<drake::geometry::QueryObject<double>>(
          context);
  auto& suction_cup_base_bdy_qry_pt_closest_obj_signed_dist_to_pt_state =
      state_ptr->get_mutable_abstract_state()
          .get_mutable_value(
              suction_cup_base_bdy_qry_pt_closest_obj_signed_dist_to_pt_state_idx_)
          .get_mutable_value<
              std::vector<drake::geometry::SignedDistanceToPoint<double>>>();

  auto& suction_cup_edge_pt_closest_obj_dist_state =
      state_ptr->get_mutable_discrete_state(
          suction_cup_edge_pt_closest_obj_dist_state_idx_);

  // for each suction cup 
  for (int suction_cup_idx = 0; suction_cup_idx < num_suction_cups_;
       suction_cup_idx++) {

    drake::geometry::GeometryId closest_obj_geom_id;

    // Distance between base body query point and object
    auto p_WoBo_W = geom_query.GetPoseInWorld(base_body_geom_id_).translation(); // visual geometry. center of base half. 
    auto p_BoQo_W = Eigen::Vector3d(0, 0, -kCupHeight*3/4); // X_BG. translate down to the bottom of the entire cup  
    const auto p_WoQo_W = p_WoBo_W + p_BoQo_W;  
    auto signed_dist_pairs = geom_query.ComputeSignedDistanceToPoint(p_WoQo_W);

    AppendSignedDistanceFromConvexHullToPoint(signed_dist_pairs, p_WoQo_W, geom_query,
                                                  0.1);

    auto min_suction_cup_query_pt_obj_dist = std::numeric_limits<double>::infinity();
    for (const auto& signed_dist_to_pt : signed_dist_pairs) {
        if (obj_geom_id_to_body_idx_map_.find(signed_dist_to_pt.id_G) != obj_geom_id_to_body_idx_map_.end() && signed_dist_to_pt.distance < min_suction_cup_query_pt_obj_dist) {
            min_suction_cup_query_pt_obj_dist = signed_dist_to_pt.distance;
            closest_obj_geom_id = signed_dist_to_pt.id_G; // note: this is where the closest object is computed relative to action point
            suction_cup_base_bdy_qry_pt_closest_obj_signed_dist_to_pt_state.at(suction_cup_idx) = signed_dist_to_pt;
        }
    }
    // TODO: what if no floating bodies present? 


    auto mean_suction_cup_edge_pt_obj_dist = 0.;
    // for each edge point of the suction cup
    const auto kNumEdgePtsPerCup =  4; 
    int obj_located = 0;
    for (uint suction_cup_edge_pt_idx = 0;
       suction_cup_edge_pt_idx < kNumEdgePtsPerCup; suction_cup_edge_pt_idx++) {

       auto angle = 2 * M_PI / kNumEdgePtsPerCup * suction_cup_edge_pt_idx;

       auto p_WoBo_W_edge = geom_query.GetPoseInWorld(cup_body_geom_id_).translation();
       auto p_BoQo_B_edge = Eigen::Vector3d((kCupOuterDiameter / 2) * std::cos(angle),
            (kCupOuterDiameter / 2) * std::sin(angle), -kCupHeight / 2); // X_BG. edges poitns at bottom of entire cup. 
       auto R_WB_edge = geom_query.GetPoseInWorld(cup_body_geom_id_).rotation();
       auto p_BoQo_W_edge = R_WB_edge * p_BoQo_B_edge;
        
       const auto p_WoQo_W_edge = p_WoBo_W_edge + p_BoQo_W_edge; 
    //    const auto threshold = max_suction_dist_ + 100; // 2 mm sufficient for t = 0. can't seem to find sweet spot for holding up to 0.25. 
       // NOTE: this is threshold PER point. individual distances can be > max_suction_dist_ but the average dist can still be < max_suction_dist_. 
       (void) max_suction_dist_;
       // no point in looking further than min_suction_cup_query_pt_obj_dist away + radius
       signed_dist_pairs = geom_query.ComputeSignedDistanceToPoint(p_WoQo_W_edge, min_suction_cup_query_pt_obj_dist+kCupOuterDiameter/2);
       AppendSignedDistanceFromConvexHullToPoint(signed_dist_pairs, p_WoQo_W_edge, geom_query,
                                                  0.1);

       // find pair where pair.id_G = closest_obj_geom_id if it exists in signed_dist_pairs
       const auto& it = std::find_if(signed_dist_pairs.begin(), signed_dist_pairs.end(), [&](const auto& pair){return pair.id_G == closest_obj_geom_id;});

       if (it != signed_dist_pairs.end()){
            mean_suction_cup_edge_pt_obj_dist += std::max(it->distance, 0.); // distance is considered zero if point is inside body 
            obj_located++;
       }else{
            mean_suction_cup_edge_pt_obj_dist = std::numeric_limits<double>::infinity();
       }

    }

    if (obj_located != kNumEdgePtsPerCup) throw std::runtime_error("Not all edge points found object");
    mean_suction_cup_edge_pt_obj_dist /= kNumEdgePtsPerCup;
    suction_cup_edge_pt_closest_obj_dist_state[suction_cup_idx] =
        mean_suction_cup_edge_pt_obj_dist;

  }
  return drake::systems::EventStatus::Succeeded();
}

void CupObjInterface::CalcSuctionForce(
    const drake::systems::Context<double>& context,
    std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>*
        suction_force_vec_ptr) const {
  const auto& pressure_vec =
      GetSuctionCupPressureInputPort()
          .Eval<drake::systems::BasicVector<double>>(context);

  const auto& suction_cup_base_bdy_qry_pt_closest_obj_signed_dist_to_pt_state =
      context.get_abstract_state<
          std::vector<drake::geometry::SignedDistanceToPoint<double>>>(
          suction_cup_base_bdy_qry_pt_closest_obj_signed_dist_to_pt_state_idx_);

  for (int suction_cup_idx = 0; suction_cup_idx < num_suction_cups_;
       suction_cup_idx++) {
    auto signed_dist_to_pt =
        suction_cup_base_bdy_qry_pt_closest_obj_signed_dist_to_pt_state.at(
            suction_cup_idx);

    drake::geometry::GeometryId closest_obj_geom_id = signed_dist_to_pt.id_G;
    Eigen::Vector3d p_GC = signed_dist_to_pt.p_GN;  // geometry to closest point. aka the witness point. 
    Eigen::Vector3d cup_base_qry_pt_obj_vec = -signed_dist_to_pt.grad_W; // force should point in direction of obj 

    // raise exception if this vector ever points upward (meaning action point is below body COM)
    if (cup_base_qry_pt_obj_vec.z() > 0) throw std::runtime_error("Action point is below body COM");

    double f_mag = -pressure_vec[suction_cup_idx] * suction_cup_area_; // suction_cup_area obviously the same 

    // this is actually base body index. cup body is the part that touches the object. 
    auto cup_body_idx = suction_cup_base_body_id_vec_.at(suction_cup_idx); 
    auto obj_body_idx = obj_geom_id_to_body_idx_map_.at(closest_obj_geom_id);

    const auto& scene_graph_inspector =
        GetGeomQueryInputPort()
            .Eval<drake::geometry::QueryObject<double>>(context)
            .inspector();
    const auto& X_BG = scene_graph_inspector.GetPoseInFrame(
        closest_obj_geom_id);  // body to geometry transform 

    ExternallyAppliedSpatialForcePair suction_force_pair(
        /*body_index1*/ cup_body_idx,
        /*body_index2*/ obj_body_idx,
        /*p_BoBq_B1*/ Eigen::Vector3d::Zero(),
        /*p_BoBq_B2*/
        (X_BG * drake::math::RigidTransform<double>(p_GC)).translation(),
        /*force_axis*/ cup_base_qry_pt_obj_vec,
        /*f_mag*/ f_mag,
        /*trq_axis*/ Eigen::Vector3d::UnitZ(),
        /*tau_mag*/ 0);

    auto suction_force_pair_vec = suction_force_pair.GetAsPair();

    suction_force_vec_ptr->at(2 * suction_cup_idx) =
        suction_force_pair_vec.first;
    suction_force_vec_ptr->at(2 * suction_cup_idx + 1) =
        suction_force_pair_vec.second;
  }
}

void CupObjInterface::CalcSingleSuctionForce(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* single_suction_force_vector_ptr) const {
        
  const auto& pressure_vec =
      GetSuctionCupPressureInputPort()
          .Eval<drake::systems::BasicVector<double>>(context);

  const auto& suction_cup_base_bdy_qry_pt_closest_obj_signed_dist_to_pt_state =
      context.get_abstract_state<
          std::vector<drake::geometry::SignedDistanceToPoint<double>>>(
          suction_cup_base_bdy_qry_pt_closest_obj_signed_dist_to_pt_state_idx_);

  for (int suction_cup_idx = 0; suction_cup_idx < num_suction_cups_;
       suction_cup_idx++) {
    auto signed_dist_to_pt =
        suction_cup_base_bdy_qry_pt_closest_obj_signed_dist_to_pt_state.at(
            suction_cup_idx);

    Eigen::Vector3d cup_base_qry_pt_obj_vec = signed_dist_to_pt.grad_W;
    double f_mag = -pressure_vec[suction_cup_idx] * suction_cup_area_;

    Eigen::Vector3d reaction_force{-cup_base_qry_pt_obj_vec*f_mag};
    single_suction_force_vector_ptr->SetFromVector(reaction_force);
  }
}

void CupObjInterface::OutputCupObjDist(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* cup_obj_dist_vec_ptr) const {
  const auto& suction_cup_edge_pt_closest_obj_dist_state =
      context.get_discrete_state(
          suction_cup_edge_pt_closest_obj_dist_state_idx_);
  cup_obj_dist_vec_ptr->SetFromVector(
      suction_cup_edge_pt_closest_obj_dist_state.get_value());
}

void CupObjInterface::AppendSignedDistanceFromConvexHullToPoint(
        std::vector<drake::geometry::SignedDistanceToPoint<double>>& dist_to_pt_vec, Eigen::Vector3d query_pt,
        const drake::geometry::QueryObject<double>& geom_query, double exclude_thresh) const {
    auto& inspector = geom_query.inspector();
    for (const auto& [obj_geom_id, _] : obj_geom_id_to_body_idx_map_) {
        auto obj_geom_shape_name = inspector.GetShape(obj_geom_id).type_name();
        if (obj_geom_shape_name == "Mesh" || obj_geom_shape_name == "Convex") {
            auto* convex_hull_ptr = inspector.GetConvexHull(obj_geom_id);

            auto bbox_center_dim_pair = convex_hull_ptr->CalcBoundingBox(); // axis-aligned bounding box
            double bounding_sphere_radius = bbox_center_dim_pair.second.norm() / 2;

            // skip distance query computation if outside bounding sphere by some margin
            auto world_geom_transform = geom_query.GetPoseInWorld(obj_geom_id);
            if ((world_geom_transform.translation() - query_pt).norm() - bounding_sphere_radius > exclude_thresh) { continue; }

            Eigen::Vector3d query_pt_in_geom_frame = world_geom_transform.inverse() * query_pt;
            auto id_G = obj_geom_id;
            auto [p_GN, distance, grad_G] = CalcApproxSignedDistanceToConvexHull(query_pt_in_geom_frame, *convex_hull_ptr);
            Eigen::Vector3d grad_W = world_geom_transform.rotation() * grad_G;
            drake::geometry::SignedDistanceToPoint<double> dist_to_pt(id_G, p_GN, distance, grad_W);

            dist_to_pt_vec.push_back(dist_to_pt);
        }
    }
}


std::tuple<Eigen::Vector3d, double, Eigen::Vector3d> CalcApproxSignedDistanceToConvexHull(
        const Eigen::Vector3d& queryPoint, const drake::geometry::PolygonSurfaceMesh<double>& convexHull, double smoothFactor) {
    // The idea is taken from https://arxiv.org/pdf/2408.09612 Section IV.A
    // The max signed distance to each face plane can be used as an approximation for signed distance to the mash, with the
    // assumption that the mesh has large enough number of faces. However, using max function may result in discontinuity. Same as
    // the paper, we use LogSumExp for smoothing the signed distances, nearest points, and the gradient vectors.

    // Denote: \phi: signed distnace to mesh, phi_i: signed distnace to i-th face plane, \delta: smooth factor, \lambda: exponent
    // offset for better numerics, \bm{n}_i: normal vector of the i-th face, \bm{q}: query point.

    // Signed distance: \phi = \delta (log(\sum exp(\phi_i / \delta - \lambda)) + \lambda)

    // Gradient: \partial \phi / \partial \bm{q} = (\sum exp(\phi_i/\delta - \lambda) \bm{n_i}) / (\sum exp(\phi_i/\delta -
    // \lambda))

    // Nearest point: \bm{p} = \bm{q} - \phi * (\partial \phi / \partial \bm{q})

    DRAKE_DEMAND(smoothFactor > 0);
    double signedDistExpSum = 0;
    Eigen::Vector3d signedDistExpWeightedNormalVecSum = Eigen::Vector3d::Zero();
    double offset =
            0; // we do a transform to avoid overflow of exp function with large exponent, not affecting the LogSumExp result
    for (int elementIdx = 0; elementIdx < convexHull.num_faces(); elementIdx++) {
        Eigen::Vector3d normal = convexHull.face_normal(elementIdx);
        Eigen::Vector3d centroid = convexHull.element_centroid(elementIdx); // can be any point in the face, using centroid only
                                                                            // because Drake pre-compute this and has a O(1) getter
        Eigen::Vector3d centroidToQueryPointVec = queryPoint - centroid;
        double signedDistaceToPlane = centroidToQueryPointVec.dot(normal);

        // assuming all signedDistaceToPlane are in similar order of magnitude, use the 1st element to get a resonable offset
        if (elementIdx == 0) { offset = signedDistaceToPlane / smoothFactor; }

        double signedDistExp = std::exp(signedDistaceToPlane / smoothFactor - offset);
        signedDistExpSum += signedDistExp;
        signedDistExpWeightedNormalVecSum += signedDistExp * normal;
    }

    double signedDistance = smoothFactor * (offset + std::log(signedDistExpSum));
    Eigen::Vector3d gradient = (signedDistExpWeightedNormalVecSum / signedDistExpSum).stableNormalized();
    Eigen::Vector3d nearestPoint = queryPoint - signedDistance * gradient;
    return {nearestPoint, signedDistance, gradient};
}

}  // namespace drake::examples::multibody::suction_gripper
