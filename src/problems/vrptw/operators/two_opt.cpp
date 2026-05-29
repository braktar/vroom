/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/two_opt.h"
#include "utils/helpers_vrptw_ls.h"

namespace vroom::vrptw {

TwoOpt::TwoOpt(const Input& input,
               const utils::SolutionState& sol_state,
               TWRoute& tw_s_route,
               Index s_vehicle,
               Index s_rank,
               TWRoute& tw_t_route,
               Index t_vehicle,
               Index t_rank)
  : cvrp::TwoOpt(input,
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

void TwoOpt::compute_gain() {
  utils::vrptw_ls::run_compute_gain(
    [&] {
      set_wait_gain_upper_bound(
        utils::wait_gain_upper_bound_from_routes(_input,
                                                 s_vehicle,
                                                 s_route,
                                                 &_tw_s_route,
                                                 t_vehicle,
                                                 t_route,
                                                 &_tw_t_route));
    },
    [&] { cvrp::TwoOpt::compute_gain(); });
}

void TwoOpt::apply_wait_gain_adjustment() {
  if (wait_gain_adjusted || !gain_computed) {
    return;
  }
  utils::adjust_two_opt_wait_gain(_input,
                                  stored_gain,
                                  s_vehicle,
                                  s_route,
                                  s_rank,
                                  t_vehicle,
                                  t_route,
                                  t_rank,
                                  &_tw_s_route,
                                  &_tw_t_route,
                                  best_known_threshold);
  wait_gain_adjusted = true;
}

bool TwoOpt::is_valid() {
  return utils::vrptw_ls::is_valid(
    _input,
    [&] {
      return cvrp::TwoOpt::is_valid() &&
             _tw_t_route.is_valid_addition_for_tw(_input,
                                                  _s_delivery,
                                                  s_route.begin() + s_rank + 1,
                                                  s_route.end(),
                                                  t_rank + 1,
                                                  t_route.size()) &&
             _tw_s_route.is_valid_addition_for_tw(_input,
                                                    _t_delivery,
                                                    t_route.begin() + t_rank + 1,
                                                    t_route.end(),
                                                    s_rank + 1,
                                                    s_route.size());
    },
    [&] {
      std::vector<Index> ns;
      std::vector<Index> nt;
      utils::build_two_opt_post_routes(s_route, s_rank, t_route, t_rank, ns, nt);
      return utils::routes_within_max_duration_for_ls(_input,
                                                      s_vehicle,
                                                      ns,
                                                      t_vehicle,
                                                      nt,
                                                      &_tw_s_route,
                                                      &_tw_t_route);
    });
}

void TwoOpt::apply() {
  std::vector<Index> t_job_ranks;
  t_job_ranks.insert(t_job_ranks.begin(),
                     t_route.begin() + t_rank + 1,
                     t_route.end());

  _tw_t_route.replace(_input,
                      _s_delivery,
                      s_route.begin() + s_rank + 1,
                      s_route.end(),
                      t_rank + 1,
                      t_route.size());
  _tw_s_route.replace(_input,
                      _t_delivery,
                      t_job_ranks.begin(),
                      t_job_ranks.end(),
                      s_rank + 1,
                      s_route.size());
}

} // namespace vroom::vrptw
