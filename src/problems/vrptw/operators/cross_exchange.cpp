/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/cross_exchange.h"
#include "utils/helpers.h"
#include "utils/helpers_vrptw_ls.h"

namespace vroom::vrptw {

CrossExchange::CrossExchange(const Input& input,
                             const utils::SolutionState& sol_state,
                             TWRoute& tw_s_route,
                             Index s_vehicle,
                             Index s_rank,
                             TWRoute& tw_t_route,
                             Index t_vehicle,
                             Index t_rank,
                             bool check_s_reverse,
                             bool check_t_reverse)
  : cvrp::CrossExchange(input,
                        sol_state,
                        static_cast<RawRoute&>(tw_s_route),
                        s_vehicle,
                        s_rank,
                        static_cast<RawRoute&>(tw_t_route),
                        t_vehicle,
                        t_rank,
                        check_s_reverse,
                        check_t_reverse),
    _tw_s_route(tw_s_route),
    _tw_t_route(tw_t_route) {
}

bool CrossExchange::prunable_by_travel_upper_bound(const Eval& current_best) {
  return utils::vrptw_ls::prunable_by_travel_upper_bound(
    _input,
    current_best,
    gain_upper_bound(),
    [&] {
      return utils::wait_gain_upper_bound_from_routes(_input,
                                                      s_vehicle,
                                                      s_route,
                                                      &_tw_s_route,
                                                      t_vehicle,
                                                      t_route,
                                                      &_tw_t_route);
    });
}

void CrossExchange::compute_gain() {
  if (!_gain_upper_bound_computed) {
    (void)gain_upper_bound();
  }

  if (utils::vrptw_ls::ls_simple_eval(_input)) {
    utils::vrptw_ls::run_travel_compute_gain(
      [&] { cvrp::CrossExchange::compute_gain(); });
    return;
  }

  utils::vrptw_ls::run_edge_swap_compute_gain(
    stored_gain,
    gain_computed,
    _input,
    [&] { return cvrp::CrossExchange::is_valid(); },
    [&] {
      bool valid = true;

      auto t_start = t_route.begin() + t_rank;
      s_is_normal_valid =
        s_is_normal_valid && _tw_s_route.is_valid_addition_for_tw(_input,
                                                                  target_delivery,
                                                                  t_start,
                                                                  t_start + 2,
                                                                  s_rank,
                                                                  s_rank + 2);

      if (check_t_reverse) {
        auto t_reverse_start = t_route.rbegin() + t_route.size() - 2 - t_rank;
        s_is_reverse_valid =
          s_is_reverse_valid &&
          _tw_s_route.is_valid_addition_for_tw(_input,
                                               target_delivery,
                                               t_reverse_start,
                                               t_reverse_start + 2,
                                               s_rank,
                                               s_rank + 2);
      }

      valid = s_is_normal_valid || s_is_reverse_valid;

      if (valid) {
        auto s_start = s_route.begin() + s_rank;
        t_is_normal_valid =
          t_is_normal_valid && _tw_t_route.is_valid_addition_for_tw(_input,
                                                                    source_delivery,
                                                                    s_start,
                                                                    s_start + 2,
                                                                    t_rank,
                                                                    t_rank + 2);

        if (check_s_reverse) {
          auto s_reverse_start = s_route.rbegin() + s_route.size() - 2 - s_rank;
          t_is_reverse_valid =
            t_is_reverse_valid &&
            _tw_t_route.is_valid_addition_for_tw(_input,
                                                 source_delivery,
                                                 s_reverse_start,
                                                 s_reverse_start + 2,
                                                 t_rank,
                                                 t_rank + 2);
        }

        valid = t_is_normal_valid || t_is_reverse_valid;
      }

      return valid;
    },
    [&] {
      return utils::cross_exchange_within_max_duration(_input,
                                                       s_vehicle,
                                                       s_route,
                                                       s_rank,
                                                       t_vehicle,
                                                       t_route,
                                                       t_rank,
                                                       s_is_normal_valid,
                                                       s_is_reverse_valid,
                                                       t_is_normal_valid,
                                                       t_is_reverse_valid,
                                                       _normal_s_gain,
                                                       _reversed_s_gain,
                                                       _normal_t_gain,
                                                       _reversed_t_gain,
                                                       &_tw_s_route,
                                                       &_tw_t_route);
    },
    [&] { cvrp::CrossExchange::select_stored_gain(); });
}

void CrossExchange::apply_wait_gain_adjustment() {
  if (wait_gain_adjusted || !gain_computed) {
    return;
  }
  utils::adjust_cross_exchange_wait_gain(_input,
                                         stored_gain,
                                         s_vehicle,
                                         s_route,
                                         s_rank,
                                         reverse_s_edge,
                                         reverse_t_edge,
                                         t_vehicle,
                                         t_route,
                                         t_rank,
                                         &_tw_s_route,
                                         &_tw_t_route,
                                         best_known_threshold);
  wait_gain_adjusted = true;
}

bool CrossExchange::is_valid() {
  if (!utils::vrptw_ls::ls_simple_eval(_input)) {
    if (!gain_computed || stored_gain == NO_GAIN) {
      return false;
    }
    std::vector<Index> t_job_ranks;
    if (!reverse_t_edge) {
      auto t_start = t_route.begin() + t_rank;
      t_job_ranks.insert(t_job_ranks.begin(), t_start, t_start + 2);
    } else {
      auto t_reverse_start = t_route.rbegin() + t_route.size() - 2 - t_rank;
      t_job_ranks.insert(t_job_ranks.begin(),
                         t_reverse_start,
                         t_reverse_start + 2);
    }

    const bool s_ok =
      _tw_s_route.is_valid_addition_for_tw(_input,
                                           target_delivery,
                                           t_job_ranks.begin(),
                                           t_job_ranks.end(),
                                           s_rank,
                                           s_rank + 2);

    const bool t_ok =
      !reverse_s_edge
        ? _tw_t_route.is_valid_addition_for_tw(_input,
                                               source_delivery,
                                               s_route.begin() + s_rank,
                                               s_route.begin() + s_rank + 2,
                                               t_rank,
                                               t_rank + 2)
        : _tw_t_route.is_valid_addition_for_tw(
            _input,
            source_delivery,
            s_route.rbegin() + s_route.size() - 2 - s_rank,
            s_route.rbegin() + s_route.size() - s_rank,
            t_rank,
            t_rank + 2);

    return s_ok && t_ok;
  }

  bool valid = cvrp::CrossExchange::is_valid();

  if (valid) {
    auto t_start = t_route.begin() + t_rank;
    s_is_normal_valid =
      s_is_normal_valid && _tw_s_route.is_valid_addition_for_tw(_input,
                                                                target_delivery,
                                                                t_start,
                                                                t_start + 2,
                                                                s_rank,
                                                                s_rank + 2);

    if (check_t_reverse) {
      auto t_reverse_start = t_route.rbegin() + t_route.size() - 2 - t_rank;
      s_is_reverse_valid =
        s_is_reverse_valid &&
        _tw_s_route.is_valid_addition_for_tw(_input,
                                             target_delivery,
                                             t_reverse_start,
                                             t_reverse_start + 2,
                                             s_rank,
                                             s_rank + 2);
    }

    valid = s_is_normal_valid || s_is_reverse_valid;
  }

  if (valid) {
    auto s_start = s_route.begin() + s_rank;
    t_is_normal_valid =
      t_is_normal_valid && _tw_t_route.is_valid_addition_for_tw(_input,
                                                                source_delivery,
                                                                s_start,
                                                                s_start + 2,
                                                                t_rank,
                                                                t_rank + 2);

    if (check_s_reverse) {
      auto s_reverse_start = s_route.rbegin() + s_route.size() - 2 - s_rank;
      t_is_reverse_valid =
        t_is_reverse_valid &&
        _tw_t_route.is_valid_addition_for_tw(_input,
                                             source_delivery,
                                             s_reverse_start,
                                             s_reverse_start + 2,
                                             t_rank,
                                             t_rank + 2);
    }

    valid = t_is_normal_valid || t_is_reverse_valid;
  }

  return valid;
}

void CrossExchange::apply() {
  assert(!reverse_s_edge ||
         (_input.jobs[s_route[s_rank]].type == JOB_TYPE::SINGLE &&
          _input.jobs[s_route[s_rank + 1]].type == JOB_TYPE::SINGLE));
  assert(!reverse_t_edge ||
         (_input.jobs[t_route[t_rank]].type == JOB_TYPE::SINGLE &&
          _input.jobs[t_route[t_rank + 1]].type == JOB_TYPE::SINGLE));

  std::vector<Index> t_job_ranks;
  if (!reverse_t_edge) {
    auto t_start = t_route.begin() + t_rank;
    t_job_ranks.insert(t_job_ranks.begin(), t_start, t_start + 2);
  } else {
    auto t_reverse_start = t_route.rbegin() + t_route.size() - 2 - t_rank;
    t_job_ranks.insert(t_job_ranks.begin(),
                       t_reverse_start,
                       t_reverse_start + 2);
  }

  if (!reverse_s_edge) {
    _tw_t_route.replace(_input,
                        source_delivery,
                        s_route.begin() + s_rank,
                        s_route.begin() + s_rank + 2,
                        t_rank,
                        t_rank + 2);
  } else {
    auto s_reverse_start = s_route.rbegin() + s_route.size() - 2 - s_rank;
    _tw_t_route.replace(_input,
                        source_delivery,
                        s_reverse_start,
                        s_reverse_start + 2,
                        t_rank,
                        t_rank + 2);
  }

  _tw_s_route.replace(_input,
                      target_delivery,
                      t_job_ranks.begin(),
                      t_job_ranks.end(),
                      s_rank,
                      s_rank + 2);
}

} // namespace vroom::vrptw
