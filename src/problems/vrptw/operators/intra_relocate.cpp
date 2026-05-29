/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/intra_relocate.h"
#include "utils/helpers_vrptw_ls.h"

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

bool IntraRelocate::prunable_by_travel_upper_bound(const Eval& current_best) {
  const Eval travel_ub = utils::vrptw_ls::intra_relocate_travel_upper_bound(
    _input, _sol_state, s_route, s_vehicle, s_rank, t_route, t_rank);
  return utils::vrptw_ls::prunable_by_travel_upper_bound(
    _input,
    current_best,
    travel_ub,
    [&] {
      return utils::wait_gain_upper_bound_from_route(_input,
                                                     s_vehicle,
                                                     s_route,
                                                     &_tw_s_route);
    });
}

void IntraRelocate::compute_gain() {
  utils::vrptw_ls::run_travel_compute_gain(
    [&] { cvrp::IntraRelocate::compute_gain(); });
}
void IntraRelocate::apply_wait_gain_adjustment() {
  if (wait_gain_adjusted || !gain_computed) {
    return;
  }

      std::vector<Index> nr;
      utils::build_intra_relocate_post_route(s_route, s_rank, t_rank, nr);
      utils::adjust_stored_gain_for_wait_approx_one_route(_input,
                                                          stored_gain,
                                                          s_vehicle,
                                                          s_route,
                                                          nr,
                                                          &_tw_s_route,
                                                          best_known_threshold);
      wait_gain_adjusted = true;
}


bool IntraRelocate::is_valid() {
  return utils::vrptw_ls::is_valid(
    _input,
    [&] {
      return cvrp::IntraRelocate::is_valid() &&
             _tw_s_route.is_valid_addition_for_tw(_input,
                                                  _delivery,
                                                  _moved_jobs.begin(),
                                                  _moved_jobs.end(),
                                                  _first_rank,
                                                  _last_rank);
    },
    [&] {
      std::vector<Index> route_after;
      utils::build_intra_relocate_post_route(s_route, s_rank, t_rank, route_after);
      return utils::route_jobs_within_max_duration_for_ls(_input,
                                                          s_vehicle,
                                                          route_after,
                                                          &_tw_s_route);
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
