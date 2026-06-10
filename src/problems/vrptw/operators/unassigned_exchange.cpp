/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/unassigned_exchange.h"
#include "utils/helpers_vrptw_ls.h"

namespace vroom::vrptw {

UnassignedExchange::UnassignedExchange(const Input& input,
                                       const utils::SolutionState& sol_state,
                                       std::unordered_set<Index>& unassigned,
                                       TWRoute& tw_s_route,
                                       Index s_vehicle,
                                       Index s_rank,
                                       Index t_rank,
                                       Index u)
  : cvrp::UnassignedExchange(input,
                             sol_state,
                             unassigned,
                             static_cast<RawRoute&>(tw_s_route),
                             s_vehicle,
                             s_rank,
                             t_rank,
                             u),
    _tw_s_route(tw_s_route) {
}

bool UnassignedExchange::prunable_by_travel_upper_bound(const Eval& current_best) {
  if (utils::vrptw_ls::ls_simple_eval(_input)) {
    return false;
  }

  Eval travel_ub;
  if (t_rank == s_rank) {
    travel_ub = utils::addition_eval_delta(_input,
                                           _sol_state,
                                           source,
                                           s_rank,
                                           s_rank + 1,
                                           _u);
  } else {
    const auto& v = _input.vehicles[s_vehicle];
    travel_ub = _sol_state.node_gains[s_vehicle][s_rank] -
                utils::addition_eval(_input, _u, v, s_route, t_rank);
  }
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

void UnassignedExchange::compute_gain() {
  utils::vrptw_ls::run_travel_compute_gain(
    [&] { cvrp::UnassignedExchange::compute_gain(); });
}
void UnassignedExchange::apply_wait_gain_adjustment() {
  if (wait_gain_adjusted || !gain_computed) {
    return;
  }

      utils::adjust_one_route_moved_jobs_wait_gain(_input,
                                                   stored_gain,
                                                   s_vehicle,
                                                   s_route,
                                                   _first_rank,
                                                   _moved_jobs,
                                                   &_tw_s_route,
                                                   best_known_threshold);
      wait_gain_adjusted = true;
}


bool UnassignedExchange::is_valid() {
  return utils::vrptw_ls::is_valid(
    _input,
    [&] {
      return cvrp::UnassignedExchange::is_valid() &&
             _tw_s_route.is_valid_addition_for_tw(_input,
                                                  _delivery,
                                                  _moved_jobs.begin(),
                                                  _moved_jobs.end(),
                                                  _first_rank,
                                                  _last_rank);
    },
    [&] {
      std::vector<Index> route_after;
      utils::build_one_route_after_moved_jobs(s_route,
                                              _first_rank,
                                              _moved_jobs,
                                              route_after);
      return utils::route_jobs_within_max_duration_for_ls(_input,
                                                          s_vehicle,
                                                          route_after,
                                                          &_tw_s_route);
    });
}

void UnassignedExchange::apply() {
  _tw_s_route.replace(_input,
                      _delivery,
                      _moved_jobs.begin(),
                      _moved_jobs.end(),
                      _first_rank,
                      _last_rank);

  assert(_unassigned.find(_u) != _unassigned.end());
  _unassigned.erase(_u);
  assert(_unassigned.find(_removed) == _unassigned.end());
  _unassigned.insert(_removed);
}

} // namespace vroom::vrptw
