/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>

#include "problems/vrptw/operators/swap_star.h"
#include "utils/helpers.h"

namespace vroom::vrptw {

namespace {

// Mirror cvrp::SwapStar::apply route mutations on copies (for wait approx).
void apply_swap_star_to_copies(std::vector<Index>& ns,
                               std::vector<Index>& nt,
                               const ls::SwapChoice& ch) {
  const auto s_value = ns[ch.s_rank];
  const auto t_value = nt[ch.t_rank];

  if (ch.s_rank == ch.insertion_in_source) {
    ns[ch.s_rank] = t_value;
  } else if (ch.s_rank < ch.insertion_in_source) {
    std::copy(ns.begin() + static_cast<std::ptrdiff_t>(ch.s_rank) + 1,
              ns.begin() + static_cast<std::ptrdiff_t>(ch.insertion_in_source),
              ns.begin() + static_cast<std::ptrdiff_t>(ch.s_rank));
    ns[ch.insertion_in_source - 1] = t_value;
  } else {
    std::copy(ns.rend() - static_cast<std::ptrdiff_t>(ch.s_rank),
              ns.rend() - static_cast<std::ptrdiff_t>(ch.insertion_in_source),
              ns.rend() - static_cast<std::ptrdiff_t>(ch.s_rank) - 1);
    ns[ch.insertion_in_source] = t_value;
  }

  if (ch.t_rank == ch.insertion_in_target) {
    nt[ch.t_rank] = s_value;
  } else if (ch.t_rank < ch.insertion_in_target) {
    std::copy(nt.begin() + static_cast<std::ptrdiff_t>(ch.t_rank) + 1,
              nt.begin() + static_cast<std::ptrdiff_t>(ch.insertion_in_target),
              nt.begin() + static_cast<std::ptrdiff_t>(ch.t_rank));
    nt[ch.insertion_in_target - 1] = s_value;
  } else {
    std::copy(nt.rend() - static_cast<std::ptrdiff_t>(ch.t_rank),
              nt.rend() - static_cast<std::ptrdiff_t>(ch.insertion_in_target),
              nt.rend() - static_cast<std::ptrdiff_t>(ch.t_rank) - 1);
    nt[ch.insertion_in_target] = s_value;
  }
}

} // namespace

SwapStar::SwapStar(const Input& input,
                   const utils::SolutionState& sol_state,
                   TWRoute& tw_s_route,
                   Index s_vehicle,
                   TWRoute& tw_t_route,
                   Index t_vehicle,
                   const Eval& best_known_gain)
  : cvrp::SwapStar(input,
                   sol_state,
                   static_cast<RawRoute&>(tw_s_route),
                   s_vehicle,
                   static_cast<RawRoute&>(tw_t_route),
                   t_vehicle,
                   best_known_gain),
    _tw_s_route(tw_s_route),
    _tw_t_route(tw_t_route) {
}

void SwapStar::compute_gain() {
  choice = ls::compute_best_swap_star_choice(_input,
                                             _sol_state,
                                             s_vehicle,
                                             _tw_s_route,
                                             t_vehicle,
                                             _tw_t_route,
                                             _best_known_gain);
  if (choice.gain.cost > 0) {
    stored_gain = choice.gain;

    auto ns = s_route;
    auto nt = t_route;
    apply_swap_star_to_copies(ns, nt, choice);

    const Duration dep_s = utils::min_wait_route_departure(_input, _tw_s_route);
    const Duration dep_t = utils::min_wait_route_departure(_input, _tw_t_route);
    utils::adjust_stored_gain_for_wait_approx_two_routes(_input,
                                                         stored_gain,
                                                         s_vehicle,
                                                         s_route,
                                                         ns,
                                                         dep_s,
                                                         t_vehicle,
                                                         t_route,
                                                         nt,
                                                         dep_t);
  }
  gain_computed = true;
}

void SwapStar::apply() {
  const auto s_insert = ls::get_insert_range(s_route,
                                             choice.s_rank,
                                             t_route[choice.t_rank],
                                             choice.insertion_in_source);

  const auto t_insert = ls::get_insert_range(t_route,
                                             choice.t_rank,
                                             s_route[choice.s_rank],
                                             choice.insertion_in_target);

  _tw_s_route.replace(_input,
                      choice.source_range_delivery,
                      s_insert.range.begin(),
                      s_insert.range.end(),
                      s_insert.first_rank,
                      s_insert.last_rank);

  _tw_t_route.replace(_input,
                      choice.target_range_delivery,
                      t_insert.range.begin(),
                      t_insert.range.end(),
                      t_insert.first_rank,
                      t_insert.last_rank);
}

} // namespace vroom::vrptw
