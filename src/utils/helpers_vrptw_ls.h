#ifndef HELPERS_VRPTW_LS_H
#define HELPERS_VRPTW_LS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "utils/helpers.h"

namespace vroom::utils::vrptw_ls {

// Travel gain only; wait adjustment runs in apply_wait_gain_adjustment().
template <typename CvrpComputeGain>
void run_travel_compute_gain(CvrpComputeGain&& cvrp_compute_gain) {
  cvrp_compute_gain();
}

// True when travel_ub (and optional wait_ub) cannot beat current_best.
template <typename WaitUbFn>
bool prunable_by_travel_upper_bound(const Input& input,
                                    const Eval& current_best,
                                    const Eval& travel_ub,
                                    WaitUbFn&& wait_ub_fn) {
  if (current_best < travel_ub) {
    return false;
  }
  if (!input.has_nonzero_per_wait_hour()) {
    return true;
  }
  return skip_ls_wait_pruning(travel_ub, wait_ub_fn(), current_best);
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

// Capacity + TW + max_duration, then CVRP travel gain (edge-swap operators).
template <typename CvrpValid,
          typename TwCheck,
          typename DurationCheck,
          typename CvrpComputeGain>
void run_edge_swap_compute_gain(Eval& stored_gain,
                                bool& gain_computed,
                                const Input& input,
                                CvrpValid&& cvrp_valid,
                                TwCheck&& tw_ok,
                                DurationCheck&& duration_ok,
                                CvrpComputeGain&& cvrp_compute_gain) {
  if (!cvrp_valid()) {
    stored_gain = NO_GAIN;
    gain_computed = true;
    return;
  }
  if (!tw_ok()) {
    stored_gain = NO_GAIN;
    gain_computed = true;
    return;
  }
  if (!max_duration_feasible_for_ls(input,
                                    std::forward<DurationCheck>(duration_ok))) {
    stored_gain = NO_GAIN;
    gain_computed = true;
    return;
  }
  cvrp_compute_gain();
}

Eval relocate_travel_upper_bound(const Input& input,
                                 const utils::SolutionState& sol_state,
                                 const std::vector<Index>& s_route,
                                 Index s_vehicle,
                                 Index s_rank,
                                 const std::vector<Index>& t_route,
                                 Index t_vehicle,
                                 Index t_rank);

Eval two_opt_travel_upper_bound(const Input& input,
                                const utils::SolutionState& sol_state,
                                RawRoute& source,
                                Index s_vehicle,
                                Index s_rank,
                                RawRoute& target,
                                Index t_vehicle,
                                Index t_rank);

Eval reverse_two_opt_travel_upper_bound(const Input& input,
                                        const utils::SolutionState& sol_state,
                                        RawRoute& source,
                                        Index s_vehicle,
                                        Index s_rank,
                                        RawRoute& target,
                                        Index t_vehicle,
                                        Index t_rank);

Eval route_exchange_travel_upper_bound(const Input& input,
                                       const utils::SolutionState& sol_state,
                                       RawRoute& source,
                                       Index s_vehicle,
                                       RawRoute& target,
                                       Index t_vehicle);

Eval intra_relocate_travel_upper_bound(const Input& input,
                                       const utils::SolutionState& sol_state,
                                       const std::vector<Index>& s_route,
                                       Index s_vehicle,
                                       Index s_rank,
                                       const std::vector<Index>& t_route,
                                       Index t_rank);

Eval intra_exchange_travel_upper_bound(const Input& input,
                                       const utils::SolutionState& sol_state,
                                       const std::vector<Index>& s_route,
                                       Index s_vehicle,
                                       Index s_rank,
                                       Index t_rank);

Eval intra_two_opt_travel_upper_bound(const Input& input,
                                      const utils::SolutionState& sol_state,
                                      RawRoute& source,
                                      Index s_rank,
                                      Index t_rank);

} // namespace vroom::utils::vrptw_ls

#endif
