/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/reverse_two_opt.h"
#include "utils/helpers.h"

namespace vroom::vrptw {

ReverseTwoOpt::ReverseTwoOpt(const Input& input,
                             const utils::SolutionState& sol_state,
                             TWRoute& tw_s_route,
                             Index s_vehicle,
                             Index s_rank,
                             TWRoute& tw_t_route,
                             Index t_vehicle,
                             Index t_rank)
  : cvrp::ReverseTwoOpt(input,
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

void ReverseTwoOpt::compute_gain() {
  cvrp::ReverseTwoOpt::compute_gain();

  auto ns = s_route;
  auto nt = t_route;
  const auto nb_source = ns.size() - 1 - s_rank;
  nt.insert(nt.begin(),
            ns.rbegin(),
            ns.rbegin() + static_cast<std::ptrdiff_t>(nb_source));
  ns.erase(ns.begin() + static_cast<std::ptrdiff_t>(s_rank) + 1, ns.end());
  ns.insert(ns.end(),
            nt.rend() - static_cast<std::ptrdiff_t>(t_rank) -
              static_cast<std::ptrdiff_t>(nb_source) - 1,
            nt.rend() - static_cast<std::ptrdiff_t>(nb_source));
  nt.erase(nt.begin() + static_cast<std::ptrdiff_t>(nb_source),
           nt.begin() + static_cast<std::ptrdiff_t>(nb_source) +
             static_cast<std::ptrdiff_t>(t_rank) + 1);

  utils::adjust_stored_gain_for_wait_approx_two_routes(_input,
                                                       stored_gain,
                                                       s_vehicle,
                                                       s_route,
                                                       ns,
                                                       t_vehicle,
                                                       t_route,
                                                       nt);
}

bool ReverseTwoOpt::is_valid() {
  return cvrp::ReverseTwoOpt::is_valid() &&
         _tw_t_route.is_valid_addition_for_tw(_input,
                                              _s_delivery,
                                              s_route.rbegin(),
                                              s_route.rbegin() +
                                                s_route.size() - 1 - s_rank,
                                              0,
                                              t_rank + 1) &&
         _tw_s_route.is_valid_addition_for_tw(_input,
                                              _t_delivery,
                                              t_route.rbegin() +
                                                t_route.size() - 1 - t_rank,
                                              t_route.rend(),
                                              s_rank + 1,
                                              s_route.size());
}

void ReverseTwoOpt::apply() {
  std::vector<Index> t_job_ranks;
  t_job_ranks.insert(t_job_ranks.begin(),
                     t_route.rbegin() + t_route.size() - 1 - t_rank,
                     t_route.rend());

  _tw_t_route.replace(_input,
                      _s_delivery,
                      s_route.rbegin(),
                      s_route.rbegin() + s_route.size() - 1 - s_rank,
                      0,
                      t_rank + 1);

  _tw_s_route.replace(_input,
                      _t_delivery,
                      t_job_ranks.begin(),
                      t_job_ranks.end(),
                      s_rank + 1,
                      s_route.size());
}

} // namespace vroom::vrptw
