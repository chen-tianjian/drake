#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/math/spatial_algebra.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/multibody/plant/multibody_plant.h"


namespace drake::examples::multibody::abstract_logger {

class AbstractValueLogger : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AbstractValueLogger)
  /// @brief Constructor of the AbstractValueLogger class.
  /// @param publish_period_seconds 

    //   AbstractValueLogger(double publish_period_seconds);
  AbstractValueLogger();


  const drake::systems::InputPort<double>& GetReactionForcesInputPort()
      const {
    return get_input_port(reaction_forces_input_port_idx_);
  }

  const drake::systems::InputPort<double>& GetContactResultsInputPort()
      const {
    return get_input_port(contact_results_input_port_idx_);
  }

  const drake::systems::OutputPort<double>& GetVectorReactionForceOutputPort() const {
    return get_output_port(vector_reaction_force_output_port_idx_);
  }

 private:
  void CalcVectorReactionForce(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* reaction_force_vector_ptr) const;

//   double publish_period_seconds_;

  int reaction_forces_input_port_idx_{-1};
  int contact_results_input_port_idx_{-1};
  int vector_reaction_force_output_port_idx_{-1}; 
};
}