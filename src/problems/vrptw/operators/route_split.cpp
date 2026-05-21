/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/route_split.h"
#include "utils/helpers.h"

namespace vroom::vrptw {

RouteSplit::RouteSplit(const Input& input,
                       const utils::SolutionState& sol_state,
                       TWRoute& tw_s_route,
                       Index s_vehicle,
                       const std::vector<Index>& empty_route_ranks,
                       std::vector<TWRoute>& sol,
                       const Eval& best_known_gain)
  : cvrp::RouteSplit(input,
                     sol_state,
                     static_cast<RawRoute&>(tw_s_route),
                     s_vehicle,
                     empty_route_ranks,
                     dummy_sol,
                     best_known_gain),
    _tw_s_route(tw_s_route),
    _tw_sol(sol) {
}

void RouteSplit::compute_gain() {
  // Similar to cvrp::RouteSplit::compute_gain but makes sure to
  // trigger ls::compute_best_route_split_choice<TWRoute>.
  choice = ls::compute_best_route_split_choice(_input,
                                               _sol_state,
                                               s_vehicle,
                                               _tw_s_route,
                                               _empty_route_ranks,
                                               _best_known_gain);
  if (choice.gain.cost > 0) {
    stored_gain = choice.gain;

    // Ranks in choice are relative to _empty_route_ranks so we go
    // back to initial vehicle ranks in _sol.
    _begin_route_rank = _empty_route_ranks[choice.v_begin];
    _end_route_rank = _empty_route_ranks[choice.v_end];

    const Index v_begin = _begin_route_rank;
    const Index v_end = _end_route_rank;
    std::vector<Index> prefix(s_route.begin(),
                              s_route.begin() +
                                static_cast<std::ptrdiff_t>(choice.split_rank));
    std::vector<Index> suffix(
      s_route.begin() + static_cast<std::ptrdiff_t>(choice.split_rank),
      s_route.end());

    if (_input.has_bounded_max_duration() &&
        (!utils::route_jobs_within_max_duration_for_ls(_input, v_begin, prefix) ||
         !utils::route_jobs_within_max_duration_for_ls(_input, v_end, suffix))) {
      stored_gain = NO_GAIN;
    } else {
      const auto& v_s = _input.vehicles[s_vehicle];
      const auto& v_b = _input.vehicles[v_begin];
      const auto& v_e = _input.vehicles[v_end];
      if ((v_s.costs.per_wait_hour != 0 || v_b.costs.per_wait_hour != 0 ||
           v_e.costs.per_wait_hour != 0) &&
          v_s.breaks.empty() && v_b.breaks.empty() && v_e.breaks.empty()) {
        const auto w_full = utils::wait_cost_for_job_sequence(
          _input, s_vehicle, s_route, &_tw_s_route);
        const auto w_prefix =
          utils::wait_cost_for_job_sequence(_input, v_begin, prefix, nullptr);
        const auto w_suffix =
          utils::wait_cost_for_job_sequence(_input, v_end, suffix, nullptr);
        if (w_full.has_value() && w_prefix.has_value() && w_suffix.has_value()) {
          stored_gain.cost += *w_full - *w_prefix - *w_suffix;
        }
      }
    }
  }
  gain_computed = true;
}

void RouteSplit::apply() {
  assert(choice.gain != NO_GAIN);

  // Empty route holding the end of the split.
  auto& end_route = _tw_sol[_end_route_rank];
  assert(end_route.empty());

  const auto end_delivery =
    _tw_s_route.delivery_in_range(choice.split_rank, _tw_s_route.size());

  end_route.replace(_input,
                    end_delivery,
                    s_route.begin() + choice.split_rank,
                    s_route.end(),
                    0,
                    0);
  assert(end_route.max_load() ==
         _tw_s_route.sub_route_max_load_after(choice.split_rank));

  // Empty route holding the beginning of the split.
  auto& begin_route = _tw_sol[_begin_route_rank];
  assert(begin_route.empty());

  const auto begin_delivery =
    _tw_s_route.delivery_in_range(0, choice.split_rank);

  begin_route.replace(_input,
                      begin_delivery,
                      s_route.begin(),
                      s_route.begin() + choice.split_rank,
                      0,
                      0);
  assert(begin_route.max_load() ==
         _tw_s_route.sub_route_max_load_before(choice.split_rank));

  _tw_s_route.remove(_input, 0, s_route.size());
}

} // namespace vroom::vrptw
