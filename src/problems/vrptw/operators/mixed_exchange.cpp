/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/mixed_exchange.h"
#include "utils/helpers.h"
#include "utils/helpers_vrptw_ls.h"

namespace vroom::vrptw {

MixedExchange::MixedExchange(const Input& input,
                             const utils::SolutionState& sol_state,
                             TWRoute& tw_s_route,
                             Index s_vehicle,
                             Index s_rank,
                             TWRoute& tw_t_route,
                             Index t_vehicle,
                             Index t_rank,
                             bool check_t_reverse)
  : cvrp::MixedExchange(input,
                        sol_state,
                        static_cast<RawRoute&>(tw_s_route),
                        s_vehicle,
                        s_rank,
                        static_cast<RawRoute&>(tw_t_route),
                        t_vehicle,
                        t_rank,
                        check_t_reverse),
    _tw_s_route(tw_s_route),
    _tw_t_route(tw_t_route) {
}

bool MixedExchange::prunable_by_travel_upper_bound(const Eval& current_best) {
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

void MixedExchange::compute_gain() {
  if (!_gain_upper_bound_computed) {
    (void)gain_upper_bound();
  }

  if (utils::vrptw_ls::ls_simple_eval(_input)) {
    utils::vrptw_ls::run_travel_compute_gain(
      [&] { cvrp::MixedExchange::compute_gain(); });
    return;
  }

  utils::vrptw_ls::run_edge_swap_compute_gain(
    stored_gain,
    gain_computed,
    _input,
    [&] { return cvrp::MixedExchange::is_valid(); },
    [&] {
      bool valid =
        _tw_t_route.is_valid_addition_for_tw(_input,
                                             source_delivery,
                                             s_route.begin() + s_rank,
                                             s_route.begin() + s_rank + 1,
                                             t_rank,
                                             t_rank + 2);

      if (valid) {
        auto t_start = t_route.begin() + t_rank;
        s_is_normal_valid =
          s_is_normal_valid && _tw_s_route.is_valid_addition_for_tw(_input,
                                                                    target_delivery,
                                                                    t_start,
                                                                    t_start + 2,
                                                                    s_rank,
                                                                    s_rank + 1);

        if (check_t_reverse) {
          auto t_reverse_start = t_route.rbegin() + t_route.size() - 2 - t_rank;
          s_is_reverse_valid =
            s_is_reverse_valid &&
            _tw_s_route.is_valid_addition_for_tw(_input,
                                                 target_delivery,
                                                 t_reverse_start,
                                                 t_reverse_start + 2,
                                                 s_rank,
                                                 s_rank + 1);
        }
        valid = s_is_normal_valid || s_is_reverse_valid;
      }

      return valid;
    },
    [&] {
      return utils::mixed_exchange_within_max_duration(_input,
                                                       s_vehicle,
                                                       s_route,
                                                       s_rank,
                                                       t_vehicle,
                                                       t_route,
                                                       t_rank,
                                                       s_is_normal_valid,
                                                       s_is_reverse_valid,
                                                       _normal_s_gain,
                                                       _reversed_s_gain,
                                                       &_tw_s_route,
                                                       &_tw_t_route);
    },
    [&] { cvrp::MixedExchange::select_stored_gain(); });
}

void MixedExchange::apply_wait_gain_adjustment() {
  if (wait_gain_adjusted || !gain_computed) {
    return;
  }
  utils::adjust_mixed_exchange_wait_gain(_input,
                                         stored_gain,
                                         s_vehicle,
                                         s_route,
                                         s_rank,
                                         reverse_t_edge,
                                         t_vehicle,
                                         t_route,
                                         t_rank,
                                         &_tw_s_route,
                                         &_tw_t_route,
                                         best_known_threshold);
  wait_gain_adjusted = true;
}

bool MixedExchange::is_valid() {
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
                                           s_rank + 1);
    const std::vector<Index> s_job_ranks({s_route[s_rank]});
    const bool t_ok =
      _tw_t_route.is_valid_addition_for_tw(_input,
                                           source_delivery,
                                           s_job_ranks.begin(),
                                           s_job_ranks.end(),
                                           t_rank,
                                           t_rank + 2);
    return s_ok && t_ok;
  }

  bool valid = cvrp::MixedExchange::is_valid();

  valid =
    valid && _tw_t_route.is_valid_addition_for_tw(_input,
                                                  source_delivery,
                                                  s_route.begin() + s_rank,
                                                  s_route.begin() + s_rank + 1,
                                                  t_rank,
                                                  t_rank + 2);

  if (valid) {
    auto t_start = t_route.begin() + t_rank;
    s_is_normal_valid =
      s_is_normal_valid && _tw_s_route.is_valid_addition_for_tw(_input,
                                                                  target_delivery,
                                                                  t_start,
                                                                  t_start + 2,
                                                                  s_rank,
                                                                  s_rank + 1);

    if (check_t_reverse) {
      auto t_reverse_start = t_route.rbegin() + t_route.size() - 2 - t_rank;
      s_is_reverse_valid =
        s_is_reverse_valid &&
        _tw_s_route.is_valid_addition_for_tw(_input,
                                             target_delivery,
                                             t_reverse_start,
                                             t_reverse_start + 2,
                                             s_rank,
                                             s_rank + 1);
    }
    valid = s_is_normal_valid || s_is_reverse_valid;
  }

  return valid;
}

void MixedExchange::apply() {
  assert(!reverse_t_edge ||
         (_input.jobs[t_route[t_rank]].type == JOB_TYPE::SINGLE &&
          _input.jobs[t_route[t_rank + 1]].type == JOB_TYPE::SINGLE));

  std::vector<Index> s_job_ranks({s_route[s_rank]});
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

  _tw_s_route.replace(_input,
                      target_delivery,
                      t_job_ranks.begin(),
                      t_job_ranks.end(),
                      s_rank,
                      s_rank + 1);

  _tw_t_route.replace(_input,
                      source_delivery,
                      s_job_ranks.begin(),
                      s_job_ranks.end(),
                      t_rank,
                      t_rank + 2);
}

} // namespace vroom::vrptw
