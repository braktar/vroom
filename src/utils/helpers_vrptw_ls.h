#ifndef HELPERS_VRPTW_LS_H
#define HELPERS_VRPTW_LS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "utils/helpers.h"

namespace vroom::utils::vrptw_ls {

// Run cvrp::compute_gain between wait bound setup and wait-adjusted stored_gain.
template <typename SetUb, typename CvrpComputeGain, typename AdjustGain>
void run_compute_gain(SetUb&& set_ub,
                    CvrpComputeGain&& cvrp_compute_gain,
                    AdjustGain&& adjust_gain) {
  set_ub();
  cvrp_compute_gain();
  adjust_gain();
}

// TW feasibility then max_duration on post-move job sequences (if bounded).
template <typename TwCheck, typename DurationCheck>
bool is_valid(const Input& input,
              TwCheck&& tw_ok,
              DurationCheck&& duration_ok) {
  if (!tw_ok()) {
    return false;
  }
  return max_duration_feasible_for_ls(input,
                                      std::forward<DurationCheck>(duration_ok));
}

} // namespace vroom::utils::vrptw_ls

#endif
