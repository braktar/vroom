/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/intra_or_opt.h"
#include <algorithm>

#include "utils/helpers.h"

namespace vroom::vrptw {

IntraOrOpt::IntraOrOpt(const Input& input,
                       const utils::SolutionState& sol_state,
                       TWRoute& tw_s_route,
                       Index s_vehicle,
                       Index s_rank,
                       Index t_rank,
                       bool check_reverse)
  : cvrp::IntraOrOpt(input,
                     sol_state,
                     static_cast<RawRoute&>(tw_s_route),
                     s_vehicle,
                     s_rank,
                     t_rank,
                     check_reverse),
    _tw_s_route(tw_s_route) {
}

void IntraOrOpt::compute_gain() {
  set_wait_gain_upper_bound(utils::wait_gain_upper_bound_from_route(
    _input, s_vehicle, s_route, &_tw_s_route));

  cvrp::IntraOrOpt::compute_gain();

  auto moved = _moved_jobs;
  if (reverse_s_edge) {
    std::swap(moved[_s_edge_first], moved[_s_edge_last]);
  }
  utils::adjust_one_route_moved_jobs_wait_gain(_input,
                                               stored_gain,
                                               s_vehicle,
                                               s_route,
                                               _first_rank,
                                               moved,
                                               &_tw_s_route,
                                               best_known_threshold);
}

bool IntraOrOpt::is_valid() {
  bool valid = cvrp::IntraOrOpt::is_valid();

  if (valid) {
    is_normal_valid = is_normal_valid &&
                      _tw_s_route.is_valid_addition_for_tw(_input,
                                                           _delivery,
                                                           _moved_jobs.begin(),
                                                           _moved_jobs.end(),
                                                           _first_rank,
                                                           _last_rank);

    if (check_reverse) {
      std::swap(_moved_jobs[_s_edge_first], _moved_jobs[_s_edge_last]);

      is_reverse_valid =
        is_reverse_valid &&
        _tw_s_route.is_valid_addition_for_tw(_input,
                                             _delivery,
                                             _moved_jobs.begin(),
                                             _moved_jobs.end(),
                                             _first_rank,
                                             _last_rank);

      // Reset to initial situation before potential application.
      std::swap(_moved_jobs[_s_edge_first], _moved_jobs[_s_edge_last]);
    }

    valid = (is_normal_valid || is_reverse_valid);
  }

  if (!valid) {
    return false;
  }

  return utils::max_duration_feasible_for_ls(_input,
                                             stored_gain,
                                             get_wait_gain_upper_bound(),
                                             best_known_threshold,
                                             [&] {
                                               auto check = [&](bool reverse) {
                                                 if (!(reverse ? is_reverse_valid
                                                               : is_normal_valid)) {
                                                   return false;
                                                 }
                                                 auto moved = _moved_jobs;
                                                 if (reverse) {
                                                   std::swap(moved[_s_edge_first],
                                                             moved[_s_edge_last]);
                                                 }
                                                 std::vector<Index> route_after;
                                                 utils::build_one_route_after_moved_jobs(
                                                   s_route,
                                                   _first_rank,
                                                   moved,
                                                   route_after);
                                                 return utils::
                                                   route_jobs_within_max_duration_for_ls(
                                                     _input, s_vehicle, route_after);
                                               };

                                               return check(false) ||
                                                      (check_reverse && check(true));
                                             });
}

void IntraOrOpt::apply() {
  assert(!reverse_s_edge ||
         (_input.jobs[s_route[s_rank]].type == JOB_TYPE::SINGLE &&
          _input.jobs[s_route[s_rank + 1]].type == JOB_TYPE::SINGLE));

  if (reverse_s_edge) {
    std::swap(_moved_jobs[_s_edge_first], _moved_jobs[_s_edge_last]);
  }

  _tw_s_route.replace(_input,
                      _delivery,
                      _moved_jobs.begin(),
                      _moved_jobs.end(),
                      _first_rank,
                      _last_rank);
}

std::vector<Index> IntraOrOpt::addition_candidates() const {
  return {s_vehicle};
}

} // namespace vroom::vrptw
