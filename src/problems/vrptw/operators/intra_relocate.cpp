/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/intra_relocate.h"
#include "utils/helpers.h"

namespace vroom::vrptw {

IntraRelocate::IntraRelocate(const Input& input,
                             const utils::SolutionState& sol_state,
                             TWRoute& tw_s_route,
                             Index s_vehicle,
                             Index s_rank,
                             Index t_rank)
  : cvrp::IntraRelocate(input,
                        sol_state,
                        static_cast<RawRoute&>(tw_s_route),
                        s_vehicle,
                        s_rank,
                        t_rank),
    _tw_s_route(tw_s_route) {
}

void IntraRelocate::compute_gain() {
  set_wait_gain_upper_bound(utils::wait_gain_upper_bound_from_route(
    _input, s_vehicle, s_route, &_tw_s_route));

  cvrp::IntraRelocate::compute_gain();

  auto nr = s_route;
  const auto moved = nr[s_rank];
  nr.erase(nr.begin() + static_cast<std::ptrdiff_t>(s_rank));
  nr.insert(nr.begin() + static_cast<std::ptrdiff_t>(t_rank), moved);

  utils::adjust_stored_gain_for_wait_approx_one_route(_input,
                                                      stored_gain,
                                                      s_vehicle,
                                                      s_route,
                                                      nr,
                                                      &_tw_s_route,
                                                      best_known_threshold);
}

bool IntraRelocate::is_valid() {
  if (!cvrp::IntraRelocate::is_valid() ||
      !_tw_s_route.is_valid_addition_for_tw(_input,
                                            _delivery,
                                            _moved_jobs.begin(),
                                            _moved_jobs.end(),
                                            _first_rank,
                                            _last_rank)) {
    return false;
  }

  if (_input.vehicles[s_vehicle].max_duration == DEFAULT_MAX_DURATION) {
    return true;
  }

  auto route_after = s_route;
  const auto moved = route_after[s_rank];
  route_after.erase(route_after.begin() + static_cast<std::ptrdiff_t>(s_rank));
  route_after.insert(route_after.begin() + static_cast<std::ptrdiff_t>(t_rank),
                     moved);

  return utils::route_jobs_within_max_duration(_input, s_vehicle, route_after);
}

void IntraRelocate::apply() {
  _tw_s_route.replace(_input,
                      _delivery,
                      _moved_jobs.begin(),
                      _moved_jobs.end(),
                      _first_rank,
                      _last_rank);
}

std::vector<Index> IntraRelocate::addition_candidates() const {
  return {s_vehicle};
}

} // namespace vroom::vrptw
