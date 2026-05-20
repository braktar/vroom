/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/relocate.h"
#include "utils/helpers.h"

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

void Relocate::compute_gain() {
  set_wait_gain_upper_bound(
    utils::wait_gain_upper_bound_from_routes(_input,
                                             s_vehicle,
                                             s_route,
                                             &_tw_s_route,
                                             t_vehicle,
                                             t_route,
                                             &_tw_t_route));

  cvrp::Relocate::compute_gain();

  const auto moved_job = s_route[s_rank];
  auto ns = s_route;
  ns.erase(ns.begin() + static_cast<std::ptrdiff_t>(s_rank));
  auto nt = t_route;
  nt.insert(nt.begin() + static_cast<std::ptrdiff_t>(t_rank), moved_job);

  utils::adjust_stored_gain_for_wait_approx_two_routes(_input,
                                                       stored_gain,
                                                       s_vehicle,
                                                       s_route,
                                                       ns,
                                                       t_vehicle,
                                                       t_route,
                                                       nt,
                                                       &_tw_s_route,
                                                       &_tw_t_route,
                                                       best_known_threshold);
}

bool Relocate::is_valid() {
  if (!cvrp::Relocate::is_valid() ||
      !_tw_t_route.is_valid_addition_for_tw(_input, s_route[s_rank], t_rank) ||
      !_tw_s_route.is_valid_removal(_input, s_rank, 1)) {
    return false;
  }

  if (_input.vehicles[s_vehicle].max_duration == DEFAULT_MAX_DURATION &&
      _input.vehicles[t_vehicle].max_duration == DEFAULT_MAX_DURATION) {
    return true;
  }

  auto source_after = s_route;
  source_after.erase(source_after.begin() + static_cast<std::ptrdiff_t>(s_rank));
  auto target_after = t_route;
  target_after.insert(target_after.begin() + static_cast<std::ptrdiff_t>(t_rank),
                      s_route[s_rank]);

  return utils::route_jobs_within_max_duration(_input, s_vehicle, source_after) &&
         utils::route_jobs_within_max_duration(_input, t_vehicle, target_after);
}

void Relocate::apply() {
  auto relocate_job_rank = s_route[s_rank];

  _tw_s_route.remove(_input, s_rank, 1);
  _tw_t_route.add(_input, relocate_job_rank, t_rank);
}

} // namespace vroom::vrptw
