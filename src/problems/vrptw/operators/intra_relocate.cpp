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

  std::vector<Index> nr;
  utils::build_intra_relocate_post_route(s_route, s_rank, t_rank, nr);
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

  return utils::max_duration_feasible_for_ls(_input,
                                             stored_gain,
                                             get_wait_gain_upper_bound(),
                                             best_known_threshold,
                                             [&] {
                                               std::vector<Index> route_after;
                                               utils::build_intra_relocate_post_route(
                                                 s_route, s_rank, t_rank, route_after);
                                               return utils::route_jobs_within_max_duration_for_ls(
                                                 _input, s_vehicle, route_after);
                                             });
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
