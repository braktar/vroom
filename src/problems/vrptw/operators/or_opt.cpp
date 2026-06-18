/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/or_opt.h"
#include "utils/helpers.h"
#include "utils/helpers_vrptw_ls.h"

namespace vroom::vrptw {

OrOpt::OrOpt(const Input& input,
             const utils::SolutionState& sol_state,
             TWRoute& tw_s_route,
             Index s_vehicle,
             Index s_rank,
             TWRoute& tw_t_route,
             Index t_vehicle,
             Index t_rank)
  : cvrp::OrOpt(input,
                sol_state,
                static_cast<RawRoute&>(tw_s_route),
                s_vehicle,
                s_rank,
                static_cast<RawRoute&>(tw_t_route),
                t_vehicle,
                t_rank),
    _tw_s_route(tw_s_route),
    _tw_t_route(tw_t_route) {
}

bool OrOpt::prunable_by_travel_upper_bound(const Eval& current_best) {
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

void OrOpt::compute_gain() {
  if (!_gain_upper_bound_computed) {
    (void)gain_upper_bound();
  }

  if (utils::vrptw_ls::ls_simple_eval(_input)) {
    utils::vrptw_ls::run_travel_compute_gain([&] { cvrp::OrOpt::compute_gain(); });
    return;
  }

  utils::vrptw_ls::run_edge_swap_compute_gain(
    stored_gain,
    gain_computed,
    _input,
    [&] {
      return cvrp::OrOpt::is_valid() &&
             _tw_s_route.is_valid_removal(_input, s_rank, 2);
    },
    [&] {
      auto s_start = s_route.begin() + s_rank;
      is_normal_valid =
        is_normal_valid && _tw_t_route.is_valid_addition_for_tw(_input,
                                                                edge_delivery,
                                                                s_start,
                                                                s_start + 2,
                                                                t_rank,
                                                                t_rank);
      auto s_reverse_start = s_route.rbegin() + s_route.size() - 2 - s_rank;
      is_reverse_valid = is_reverse_valid &&
                         _tw_t_route.is_valid_addition_for_tw(_input,
                                                              edge_delivery,
                                                              s_reverse_start,
                                                              s_reverse_start + 2,
                                                              t_rank,
                                                              t_rank);

      return is_normal_valid || is_reverse_valid;
    },
    [&] {
      return utils::or_opt_within_max_duration(_input,
                                                 s_vehicle,
                                                 s_route,
                                                 s_rank,
                                                 t_vehicle,
                                                 t_route,
                                                 t_rank,
                                                 is_normal_valid,
                                                 is_reverse_valid,
                                                 _normal_t_gain,
                                                 _reversed_t_gain,
                                                 &_tw_s_route,
                                                 &_tw_t_route);
    },
    [&] { cvrp::OrOpt::select_stored_gain(); });
}

void OrOpt::apply_wait_gain_adjustment() {
  if (wait_gain_adjusted || !gain_computed) {
    return;
  }
  utils::adjust_or_opt_wait_gain(_input,
                                 stored_gain,
                                 s_vehicle,
                                 s_route,
                                 s_rank,
                                 reverse_s_edge,
                                 t_vehicle,
                                 t_route,
                                 t_rank,
                                 &_tw_s_route,
                                 &_tw_t_route,
                                 best_known_threshold);
  wait_gain_adjusted = true;
}

bool OrOpt::is_valid() {
  if (!utils::vrptw_ls::ls_simple_eval(_input)) {
    if (!gain_computed || stored_gain == NO_GAIN) {
      return false;
    }
    if (reverse_s_edge) {
      auto s_reverse_start = s_route.rbegin() + s_route.size() - 2 - s_rank;
      return _tw_t_route.is_valid_addition_for_tw(_input,
                                                    edge_delivery,
                                                    s_reverse_start,
                                                    s_reverse_start + 2,
                                                    t_rank,
                                                    t_rank);
    }
    auto s_start = s_route.begin() + s_rank;
    return _tw_t_route.is_valid_addition_for_tw(_input,
                                                  edge_delivery,
                                                  s_start,
                                                  s_start + 2,
                                                  t_rank,
                                                  t_rank);
  }

  bool valid =
    cvrp::OrOpt::is_valid() && _tw_s_route.is_valid_removal(_input, s_rank, 2);

  if (valid) {
    auto s_start = s_route.begin() + s_rank;
    is_normal_valid =
      is_normal_valid && _tw_t_route.is_valid_addition_for_tw(_input,
                                                              edge_delivery,
                                                              s_start,
                                                              s_start + 2,
                                                              t_rank,
                                                              t_rank);
    auto s_reverse_start = s_route.rbegin() + s_route.size() - 2 - s_rank;
    is_reverse_valid = is_reverse_valid &&
                       _tw_t_route.is_valid_addition_for_tw(_input,
                                                            edge_delivery,
                                                            s_reverse_start,
                                                            s_reverse_start + 2,
                                                            t_rank,
                                                            t_rank);

    valid = is_normal_valid || is_reverse_valid;
  }

  return valid;
}

void OrOpt::apply() {
  if (reverse_s_edge) {
    auto s_reverse_start = s_route.rbegin() + s_route.size() - 2 - s_rank;
    _tw_t_route.replace(_input,
                        edge_delivery,
                        s_reverse_start,
                        s_reverse_start + 2,
                        t_rank,
                        t_rank);
    _tw_s_route.remove(_input, s_rank, 2);
  } else {
    auto s_start = s_route.begin() + s_rank;
    _tw_t_route
      .replace(_input, edge_delivery, s_start, s_start + 2, t_rank, t_rank);
    _tw_s_route.remove(_input, s_rank, 2);
  }
}

} // namespace vroom::vrptw
