/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/relocate.h"
#include "utils/helpers_vrptw_ls.h"

namespace vroom::vrptw {

Relocate::Relocate(const Input& input,
                   const utils::SolutionState& sol_state,
                   TWRoute& tw_s_route,
                   Index s_vehicle,
                   Index s_rank,
                   TWRoute& tw_t_route,
                   Index t_vehicle,
                   Index t_rank)
  : cvrp::Relocate(input,
                   sol_state,
                   static_cast<RawRoute&>(tw_s_route),
                   s_vehicle,
                   s_rank,
                   static_cast<RawRoute&>(tw_t_route),
                   t_vehicle,
                   t_rank),
    _tw_s_route(tw_s_route),
    _tw_t_route(tw_t_route) {
}

bool Relocate::prunable_by_travel_upper_bound(const Eval& current_best) {
  const Eval travel_ub = utils::vrptw_ls::relocate_travel_upper_bound(_input,
                                                                      _sol_state,
                                                                      s_route,
                                                                      s_vehicle,
                                                                      s_rank,
                                                                      t_route,
                                                                      t_vehicle,
                                                                      t_rank);
  return utils::vrptw_ls::prunable_by_travel_upper_bound(
    _input,
    current_best,
    travel_ub,
    [&] {
      return utils::wait_gain_upper_bound_from_routes(_input,
                                                      s_vehicle,
                                                      s_route,
                                                      &_tw_s_route,
                                                      t_vehicle,
                                                      t_route,
                                                      &_tw_t_route);
    });
}

void Relocate::compute_gain() {
  utils::vrptw_ls::run_travel_compute_gain([&] { cvrp::Relocate::compute_gain(); });
}

void Relocate::apply_wait_gain_adjustment() {
  if (wait_gain_adjusted || !gain_computed) {
    return;
  }
  utils::adjust_relocate_wait_gain(_input,
                                   stored_gain,
                                   s_vehicle,
                                   s_route,
                                   s_rank,
                                   t_vehicle,
                                   t_route,
                                   t_rank,
                                   &_tw_s_route,
                                   &_tw_t_route,
                                   best_known_threshold);
  wait_gain_adjusted = true;
}

bool Relocate::is_valid() {
  const auto tw_ok = [&] {
    return cvrp::Relocate::is_valid() &&
           _tw_t_route.is_valid_addition_for_tw(_input, s_route[s_rank], t_rank) &&
           _tw_s_route.is_valid_removal(_input, s_rank, 1);
  };

  if (!_input.has_bounded_max_duration()) {
    return tw_ok();
  }

  return utils::vrptw_ls::is_valid(
    _input,
    tw_ok,
    [&] {
      std::vector<Index> source_after;
      std::vector<Index> target_after;
      utils::build_relocate_post_routes(s_route,
                                        s_rank,
                                        t_route,
                                        t_rank,
                                        source_after,
                                        target_after);
      return utils::routes_within_max_duration_for_ls(_input,
                                                      s_vehicle,
                                                      source_after,
                                                      t_vehicle,
                                                      target_after,
                                                      &_tw_s_route,
                                                      &_tw_t_route);
    });
}

void Relocate::apply() {
  auto relocate_job_rank = s_route[s_rank];

  _tw_s_route.remove(_input, s_rank, 1);
  _tw_t_route.add(_input, relocate_job_rank, t_rank);
}

} // namespace vroom::vrptw
