/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/intra_cross_exchange.h"
#include <algorithm>

#include "utils/helpers.h"
#include "utils/helpers_vrptw_ls.h"

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

bool IntraCrossExchange::prunable_by_travel_upper_bound(const Eval& current_best) {
  return utils::vrptw_ls::prunable_by_travel_upper_bound(
    _input,
    current_best,
    gain_upper_bound(),
    [&] {
      return utils::wait_gain_upper_bound_from_route(_input,
                                                     s_vehicle,
                                                     s_route,
                                                     &_tw_s_route);
    });
}

void IntraCrossExchange::compute_gain() {
  if (!_gain_upper_bound_computed) {
    (void)gain_upper_bound();
  }

  if (utils::vrptw_ls::ls_simple_eval(_input)) {
    utils::vrptw_ls::run_travel_compute_gain(
      [&] { cvrp::IntraCrossExchange::compute_gain(); });
    return;
  }

  utils::vrptw_ls::run_edge_swap_compute_gain(
    stored_gain,
    gain_computed,
    _input,
    [&] { return cvrp::IntraCrossExchange::is_valid(); },
    [&] {
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

      std::swap(_moved_jobs[_moved_jobs.size() - 2],
                _moved_jobs[_moved_jobs.size() - 1]);

      return s_normal_t_normal_is_valid || s_normal_t_reverse_is_valid ||
             s_reverse_t_reverse_is_valid || s_reverse_t_normal_is_valid;
    },
    [&] {
      const auto [reverse_s, reverse_t] =
        utils::intra_cross_exchange_chosen_reverse_edges(_normal_s_gain,
                                                         _reversed_s_gain,
                                                         _normal_t_gain,
                                                         _reversed_t_gain,
                                                         s_normal_t_normal_is_valid,
                                                         s_normal_t_reverse_is_valid,
                                                         s_reverse_t_normal_is_valid,
                                                         s_reverse_t_reverse_is_valid);

      const bool tw_ok = (reverse_s && reverse_t)
                           ? s_reverse_t_reverse_is_valid
                           : (reverse_s ? s_reverse_t_normal_is_valid
                                        : (reverse_t ? s_normal_t_reverse_is_valid
                                                     : s_normal_t_normal_is_valid));
      if (!tw_ok) {
        return false;
      }

      auto moved = _moved_jobs;
      if (reverse_t) {
        std::swap(moved[0], moved[1]);
      }
      if (reverse_s) {
        std::swap(moved[moved.size() - 2], moved[moved.size() - 1]);
      }
      std::vector<Index> route_after;
      utils::build_one_route_after_moved_jobs(s_route,
                                              _first_rank,
                                              moved,
                                              route_after);
      return utils::route_jobs_within_max_duration_for_ls(_input,
                                                         s_vehicle,
                                                         route_after,
                                                         &_tw_s_route);
    },
    [&] { cvrp::IntraCrossExchange::compute_gain(); });
}

void IntraCrossExchange::apply_wait_gain_adjustment() {
  if (wait_gain_adjusted || !gain_computed) {
    return;
  }
  auto moved = _moved_jobs;
  if (reverse_t_edge) {
    std::swap(moved[0], moved[1]);
  }
  if (reverse_s_edge) {
    std::swap(moved[moved.size() - 2], moved[moved.size() - 1]);
  }
  utils::adjust_one_route_moved_jobs_wait_gain(_input,
                                               stored_gain,
                                               s_vehicle,
                                               s_route,
                                               _first_rank,
                                               moved,
                                               &_tw_s_route,
                                               best_known_threshold);
  wait_gain_adjusted = true;
}

bool IntraCrossExchange::is_valid() {
  if (!utils::vrptw_ls::ls_simple_eval(_input)) {
    return gain_computed && stored_gain != NO_GAIN;
  }

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
