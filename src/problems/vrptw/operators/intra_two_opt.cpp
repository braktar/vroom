/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/intra_two_opt.h"
#include <algorithm>

#include "utils/helpers.h"

namespace vroom::vrptw {

IntraTwoOpt::IntraTwoOpt(const Input& input,
                         const utils::SolutionState& sol_state,
                         TWRoute& tw_s_route,
                         Index s_vehicle,
                         Index s_rank,
                         Index t_rank)
  : cvrp::IntraTwoOpt(input,
                      sol_state,
                      static_cast<RawRoute&>(tw_s_route),
                      s_vehicle,
                      s_rank,
                      t_rank),
    _tw_s_route(tw_s_route) {
}

void IntraTwoOpt::compute_gain() {
  cvrp::IntraTwoOpt::compute_gain();

  auto nr = s_route;
  std::reverse(nr.begin() + static_cast<std::ptrdiff_t>(s_rank),
               nr.begin() + static_cast<std::ptrdiff_t>(t_rank) + 1);
  utils::adjust_stored_gain_for_wait_approx_one_route(_input,
                                                      stored_gain,
                                                      s_vehicle,
                                                      s_route,
                                                      nr);
}

bool IntraTwoOpt::is_valid() {
  bool valid = cvrp::IntraTwoOpt::is_valid();

  if (valid) {
    auto rev_t = s_route.rbegin() + (s_route.size() - t_rank - 1);
    auto rev_s_next = s_route.rbegin() + (s_route.size() - s_rank);

    valid = _tw_s_route.is_valid_addition_for_tw(_input,
                                                 delivery,
                                                 rev_t,
                                                 rev_s_next,
                                                 s_rank,
                                                 t_rank + 1);
  }

  return valid;
}

void IntraTwoOpt::apply() {
  std::vector<Index> reversed(s_route.rbegin() + (s_route.size() - t_rank - 1),
                              s_route.rbegin() + (s_route.size() - s_rank));

  _tw_s_route.replace(_input,
                      delivery,
                      reversed.begin(),
                      reversed.end(),
                      s_rank,
                      t_rank + 1);
}

} // namespace vroom::vrptw
