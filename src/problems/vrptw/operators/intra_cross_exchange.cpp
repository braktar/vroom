/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/intra_cross_exchange.h"
#include <algorithm>

#include "utils/helpers.h"

namespace vroom::vrptw {

IntraCrossExchange::IntraCrossExchange(const Input& input,
                                       const utils::SolutionState& sol_state,
                                       TWRoute& tw_s_route,
                                       Index s_vehicle,
                                       Index s_rank,
                                       Index t_rank,
                                       bool check_s_reverse,
                                       bool check_t_reverse)
  : cvrp::IntraCrossExchange(input,
                             sol_state,
                             static_cast<RawRoute&>(tw_s_route),
                             s_vehicle,
                             s_rank,
                             t_rank,
                             check_s_reverse,
                             check_t_reverse),
    _tw_s_route(tw_s_route) {
}

void IntraCrossExchange::compute_gain() {
  cvrp::IntraCrossExchange::compute_gain();

  auto moved = _moved_jobs;
  if (reverse_t_edge) {
    std::swap(moved[0], moved[1]);
  }
  if (reverse_s_edge) {
    std::swap(moved[moved.size() - 2], moved[moved.size() - 1]);
  }
  auto nr = s_route;
  std::copy(moved.begin(),
            moved.end(),
            nr.begin() + static_cast<std::ptrdiff_t>(_first_rank));
  const Duration dep = utils::min_wait_route_departure(_input, _tw_s_route);
  utils::adjust_stored_gain_for_wait_approx_one_route(_input,
                                                      stored_gain,
                                                      s_vehicle,
                                                      s_route,
                                                      nr,
                                                      dep);
}

bool IntraCrossExchange::is_valid() {
  bool valid = cvrp::IntraCrossExchange::is_valid();

  if (valid) {
    s_normal_t_normal_is_valid =
      s_normal_t_normal_is_valid &&
      _tw_s_route.is_valid_addition_for_tw(_input,
                                           _delivery,
                                           _moved_jobs.begin(),
                                           _moved_jobs.end(),
                                           _first_rank,
                                           _last_rank);

    std::swap(_moved_jobs[0], _moved_jobs[1]);

    if (check_t_reverse) {
      s_normal_t_reverse_is_valid =
        s_normal_t_reverse_is_valid &&
        _tw_s_route.is_valid_addition_for_tw(_input,
                                             _delivery,
                                             _moved_jobs.begin(),
                                             _moved_jobs.end(),
                                             _first_rank,
                                             _last_rank);
    }

    std::swap(_moved_jobs[_moved_jobs.size() - 2],
              _moved_jobs[_moved_jobs.size() - 1]);

    if (check_s_reverse && check_t_reverse) {
      s_reverse_t_reverse_is_valid =
        s_reverse_t_reverse_is_valid &&
        _tw_s_route.is_valid_addition_for_tw(_input,
                                             _delivery,
                                             _moved_jobs.begin(),
                                             _moved_jobs.end(),
                                             _first_rank,
                                             _last_rank);
    }

    std::swap(_moved_jobs[0], _moved_jobs[1]);

    if (check_s_reverse) {
      s_reverse_t_normal_is_valid =
        s_reverse_t_normal_is_valid &&
        _tw_s_route.is_valid_addition_for_tw(_input,
                                             _delivery,
                                             _moved_jobs.begin(),
                                             _moved_jobs.end(),
                                             _first_rank,
                                             _last_rank);
    }

    // Reset to initial situation before potential application.
    std::swap(_moved_jobs[_moved_jobs.size() - 2],
              _moved_jobs[_moved_jobs.size() - 1]);

    valid = s_normal_t_normal_is_valid || s_normal_t_reverse_is_valid ||
            s_reverse_t_reverse_is_valid || s_reverse_t_normal_is_valid;
  }

  return valid;
}

void IntraCrossExchange::apply() {
  assert(!reverse_s_edge ||
         (_input.jobs[s_route[s_rank]].type == JOB_TYPE::SINGLE &&
          _input.jobs[s_route[s_rank + 1]].type == JOB_TYPE::SINGLE));
  assert(!reverse_t_edge ||
         (_input.jobs[t_route[t_rank]].type == JOB_TYPE::SINGLE &&
          _input.jobs[t_route[t_rank + 1]].type == JOB_TYPE::SINGLE));

  if (reverse_t_edge) {
    std::swap(_moved_jobs[0], _moved_jobs[1]);
  }
  if (reverse_s_edge) {
    std::swap(_moved_jobs[_moved_jobs.size() - 2],
              _moved_jobs[_moved_jobs.size() - 1]);
  }

  _tw_s_route.replace(_input,
                      _delivery,
                      _moved_jobs.begin(),
                      _moved_jobs.end(),
                      _first_rank,
                      _last_rank);
}

std::vector<Index> IntraCrossExchange::addition_candidates() const {
  return {s_vehicle};
}

} // namespace vroom::vrptw
