/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/route_exchange.h"
#include "utils/helpers_vrptw_ls.h"

namespace vroom::vrptw {

RouteExchange::RouteExchange(const Input& input,
                             const utils::SolutionState& sol_state,
                             TWRoute& tw_s_route,
                             Index s_vehicle,
                             TWRoute& tw_t_route,
                             Index t_vehicle)
  : cvrp::RouteExchange(input,
                        sol_state,
                        static_cast<RawRoute&>(tw_s_route),
                        s_vehicle,
                        static_cast<RawRoute&>(tw_t_route),
                        t_vehicle),
    _tw_s_route(tw_s_route),
    _tw_t_route(tw_t_route),
    _source_job_deliveries_sum(source.job_deliveries_sum()),
    _target_job_deliveries_sum(target.job_deliveries_sum()) {
}

bool RouteExchange::prunable_by_travel_upper_bound(const Eval& current_best) {
  if (utils::vrptw_ls::ls_simple_eval(_input)) {
    return false;
  }

  const Eval travel_ub = utils::vrptw_ls::route_exchange_travel_upper_bound(
    _input, _sol_state, source, s_vehicle, target, t_vehicle);
  return utils::vrptw_ls::prunable_by_travel_upper_bound(
    _input,
    current_best,
    travel_ub,
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

void RouteExchange::compute_gain() {
  utils::vrptw_ls::run_travel_compute_gain(
    [&] { cvrp::RouteExchange::compute_gain(); });
}
void RouteExchange::apply_wait_gain_adjustment() {
  if (wait_gain_adjusted || !gain_computed) {
    return;
  }

      utils::adjust_route_exchange_wait_gain(_input,
                                             stored_gain,
                                             s_vehicle,
                                             s_route,
                                             t_vehicle,
                                             t_route,
                                             &_tw_s_route,
                                             &_tw_t_route,
                                             best_known_threshold);
      wait_gain_adjusted = true;
}


bool RouteExchange::is_valid() {
  const auto tw_ok = [&] {
    return cvrp::RouteExchange::is_valid() &&
           _tw_t_route.is_valid_addition_for_tw(_input,
                                                _source_job_deliveries_sum,
                                                s_route.begin(),
                                                s_route.end(),
                                                0,
                                                t_route.size()) &&
           _tw_s_route.is_valid_addition_for_tw(_input,
                                                  _target_job_deliveries_sum,
                                                  t_route.begin(),
                                                  t_route.end(),
                                                  0,
                                                  s_route.size());
  };

  if (!_input.has_bounded_max_duration()) {
    return tw_ok();
  }

  return utils::vrptw_ls::is_valid(
    _input,
    tw_ok,
    [&] {
      return utils::routes_within_max_duration_for_ls(_input,
                                                      s_vehicle,
                                                      t_route,
                                                      t_vehicle,
                                                      s_route,
                                                      &_tw_s_route,
                                                      &_tw_t_route);
    });
}

void RouteExchange::apply() {
  std::vector<Index> t_job_ranks(t_route);

  if (s_route.empty()) {
    _tw_t_route.remove(_input, 0, t_route.size());
  } else {
    _tw_t_route.replace(_input,
                        _source_job_deliveries_sum,
                        s_route.begin(),
                        s_route.end(),
                        0,
                        t_route.size());
  }

  if (t_job_ranks.empty()) {
    _tw_s_route.remove(_input, 0, s_route.size());
  } else {
    _tw_s_route.replace(_input,
                        _target_job_deliveries_sum,
                        t_job_ranks.begin(),
                        t_job_ranks.end(),
                        0,
                        s_route.size());
  }
}

} // namespace vroom::vrptw
