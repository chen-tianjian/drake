#include "drake/examples/multibody/suction_gripper/abstract_logger.h"
#include <iostream>

namespace drake::examples::multibody::abstract_logger {

// AbstractValueLogger::AbstractValueLogger(double publish_period_seconds)
//     : publish_period_seconds_(publish_period_seconds) {

AbstractValueLogger::AbstractValueLogger(){

  /// ----- Input Ports ----- ///
  reaction_forces_input_port_idx_ = DeclareAbstractInputPort("reaction_forces", Value<std::vector<drake::multibody::SpatialForce<double>>>()).get_index(); 
  contact_results_input_port_idx_ = DeclareAbstractInputPort("contact_results", Value<drake::multibody::ContactResults<double>>()).get_index();

  /// ----- Output Ports ----- ///
  vector_reaction_force_output_port_idx_ = 
    DeclareVectorOutputPort(
        "vector_reaction_force", 
        drake::systems::BasicVector<double>(3),
        &AbstractValueLogger::CalcVectorReactionForce,
        {xa_ticket(), xd_ticket(), input_port_ticket(drake::systems::InputPortIndex(reaction_forces_input_port_idx_))}).get_index(); 

}

void AbstractValueLogger::CalcVectorReactionForce(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* reaction_force_vector_ptr) const {

    const auto& reaction_forces = GetReactionForcesInputPort().Eval<std::vector<drake::multibody::SpatialForce<double>>>(context);
    const auto world_to_wrist_reaction_wrench = reaction_forces[0];

    const auto& contact_results = GetContactResultsInputPort().Eval<drake::multibody::ContactResults<double>>(context);

    // std::cout << world_to_wrist_reaction_wrench[3:5] << std::endl; 

    // const auto first_elem = world_to_wrist_reaction_wrench.end() - 3;
    // const auto last_elem = world_to_wrist_reaction_wrench.end();
    // std::vector<double> reaction_forces_vec = {world_to_wrist_reaction_wrench[3], world_to_wrist_reaction_wrench[4], world_to_wrist_reaction_wrench[5]};
    // std::cout << reaction_forces_vec << std::endl;

    // for (const auto& force : reaction_forces){
    //     std::cout << force << std::endl;
    // }

    Eigen::Vector3d reaction_forces_vec(3);
    double eoat_weight = (1.0 + 1.0 + 0.02)*9.81; // wrist mass + base mass + cup mass
    reaction_forces_vec << world_to_wrist_reaction_wrench[3], world_to_wrist_reaction_wrench[4], world_to_wrist_reaction_wrench[5] - eoat_weight;
    // std::cout << reaction_forces_vec << std::endl;

    // should only be package and cup in hydroelastic contact
    for (int i = 0; i < contact_results.num_hydroelastic_contacts(); i++){
      const auto hydroelastic_contact_info = contact_results.hydroelastic_contact_info(i);
      const auto contact_force = hydroelastic_contact_info.F_Ac_W();
      for (int j = 0; j < 3; j++){
        reaction_forces_vec(j) += contact_force[j+3]; // forces start at index 3
      }
    }
    reaction_force_vector_ptr->SetFromVector(reaction_forces_vec);
}
}
