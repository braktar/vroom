/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/intra_exchange.h"
#include <algorithm>

#include "utils/helpers.h"

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

void IntraExchange::compute_gain() {
  set_wait_gain_upper_bound(utils::wait_gain_upper_bound_from_route(
    _input, s_vehicle, s_route, &_tw_s_route));

  cvrp::IntraExchange::compute_gain();

  utils::adjust_one_route_moved_jobs_wait_gain(_input,
                                               stored_gain,
                                               s_vehicle,
                                               s_route,
                                               _first_rank,
                                               _moved_jobs,
                                               &_tw_s_route,
                                               best_known_threshold);
}

bool IntraExchange::is_valid() {
  if (!cvrp::IntraExchange::is_valid() ||
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
                                               utils::build_one_route_after_moved_jobs(
                                                 s_route,
                                                 _first_rank,
                                                 _moved_jobs,
                                                 route_after);
                                               return utils::route_jobs_within_max_duration_for_ls(
                                                 _input, s_vehicle, route_after, &_tw_s_route);
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
