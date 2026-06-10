/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/intra_exchange.h"
#include "utils/helpers_vrptw_ls.h"

namespace vroom::vrptw {

IntraExchange::IntraExchange(const Input& input,
                             const utils::SolutionState& sol_state,
                             TWRoute& tw_s_route,
                             Index s_vehicle,
                             Index s_rank,
                             Index t_rank)
  : cvrp::IntraExchange(input,
                        sol_state,
                        static_cast<RawRoute&>(tw_s_route),
                        s_vehicle,
                        s_rank,
                        t_rank),
    _tw_s_route(tw_s_route) {
}

bool IntraExchange::prunable_by_travel_upper_bound(const Eval& current_best) {
  if (utils::vrptw_ls::ls_simple_eval(_input)) {
    return false;
  }

  const Eval travel_ub = utils::vrptw_ls::intra_exchange_travel_upper_bound(
    _input, _sol_state, s_route, s_vehicle, s_rank, t_rank);
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

void IntraExchange::compute_gain() {
  utils::vrptw_ls::run_travel_compute_gain(
    [&] { cvrp::IntraExchange::compute_gain(); });
}
void IntraExchange::apply_wait_gain_adjustment() {
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


bool IntraExchange::is_valid() {
  const auto tw_ok = [&] {
    return cvrp::IntraExchange::is_valid() &&
           _tw_s_route.is_valid_addition_for_tw(_input,
                                                _delivery,
                                                _moved_jobs.begin(),
                                                _moved_jobs.end(),
                                                _first_rank,
                                                _last_rank);
  };

  if (!_input.has_bounded_max_duration()) {
    return tw_ok();
  }

  return utils::vrptw_ls::is_valid(
    _input,
    tw_ok,
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

void IntraExchange::apply() {
  _tw_s_route.replace(_input,
                      _delivery,
                      _moved_jobs.begin(),
                      _moved_jobs.end(),
                      _first_rank,
                      _last_rank);
}

std::vector<Index> IntraExchange::addition_candidates() const {
  return {s_vehicle};
}

} // namespace vroom::vrptw
