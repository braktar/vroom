#ifndef CVRP_OR_OPT_H
#define CVRP_OR_OPT_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "algorithms/local_search/operator.h"

namespace vroom::cvrp {

class OrOpt : public ls::Operator {
protected:
  bool _gain_upper_bound_computed{false};
  Eval _normal_t_gain;
  Eval _reversed_t_gain;
  bool reverse_s_edge{false};
  bool is_normal_valid{false};
  bool is_reverse_valid{false};
  const Amount edge_delivery;

  void compute_gain() override;

  // Pick edge orientation and set stored_gain; gain_upper_bound() and is_valid()
  // must already have run (VRPTW edge-swap LS calls this after TW checks).
  void select_stored_gain();

public:
  OrOpt(const Input& input,
        const utils::SolutionState& sol_state,
        RawRoute& s_route,
        Index s_vehicle,
        Index s_rank,
        RawRoute& t_route,
        Index t_vehicle,
        Index t_rank);

  // Compute and store all possible cost depending on whether edges
  // are reversed or not. Return only an upper bound for gain as
  // precise gain requires validity information.
  Eval gain_upper_bound();

  bool is_valid() override;

  void apply() override;

  std::vector<Index> addition_candidates() const override;

  std::vector<Index> update_candidates() const override;
};

} // namespace vroom::cvrp

#endif
