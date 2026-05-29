#ifndef VRPTW_PD_SHIFT_H
#define VRPTW_PD_SHIFT_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/pd_shift.h"

namespace vroom::vrptw {

class PDShift : public cvrp::PDShift {
private:
  TWRoute& _tw_s_route;
  TWRoute& _tw_t_route;
  Amount _best_t_delivery;
  std::vector<Index> _wait_s_new;
  std::vector<Index> _wait_t_new;

  void compute_gain() override;

  void apply_wait_gain_adjustment() override;

public:
  PDShift(const Input& input,
          const utils::SolutionState& sol_state,
          TWRoute& tw_s_route,
          Index s_vehicle,
          Index s_p_rank,
          Index s_d_rank,
          TWRoute& tw_t_route,
          Index t_vehicle,
          const Eval& gain_threshold);

  bool prunable_by_travel_upper_bound(const Eval& current_best) override;

  void log_route(const std::vector<Index>& route) const;

  void apply() override;
};

} // namespace vroom::vrptw

#endif
