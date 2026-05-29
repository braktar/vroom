#ifndef HELPERS_H
#define HELPERS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <optional>
#include <string>
#include <tuple>
#include <type_traits>
#include <vector>

#include "structures/typedefs.h"
#include "structures/vroom/eval.h"
#include "structures/vroom/raw_route.h"
#include "structures/vroom/solution_state.h"
#include "structures/vroom/tw_route.h"
#include "utils/exception.h"

namespace vroom::utils {

template <typename T> T round(double value) {
  constexpr double round_increment = 0.5;
  return static_cast<T>(value + round_increment);
}

TimePoint now();

Amount max_amount(std::size_t size);

inline UserCost add_without_overflow(UserCost a, UserCost b) {
  if (a > std::numeric_limits<UserCost>::max() - b) {
    throw InputException(
      "Too high cost values, stopping to avoid overflowing.");
  }
  return a + b;
}

// Taken from https://stackoverflow.com/a/72073933.
inline uint32_t get_vector_hash(const std::vector<uint32_t>& vec) {
  uint32_t seed = vec.size();
  for (auto x : vec) {
    x = ((x >> 16) ^ x) * 0x45d9f3b;
    x = ((x >> 16) ^ x) * 0x45d9f3b;
    x = (x >> 16) ^ x;
    seed ^= x + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  }
  return seed;
}

inline unsigned get_depth(unsigned exploration_level) {
  return exploration_level;
}

inline unsigned get_nb_searches(unsigned exploration_level) {
  assert(exploration_level <= MAX_EXPLORATION_LEVEL);

  unsigned nb_searches = 4 * (exploration_level + 1);
  if (exploration_level >= 4) {
    nb_searches += 4;
  }
  if (exploration_level == MAX_EXPLORATION_LEVEL) {
    nb_searches += 4;
  }

  return nb_searches;
}

// Evaluate adding job with rank job_rank in given route at given rank
// for vehicle v.
inline Eval addition_eval(const Input& input,
                          Index job_rank,
                          const Vehicle& v,
                          const std::vector<Index>& route,
                          Index rank) {
  assert(rank <= route.size());

  const auto& job = input.jobs[job_rank];
  const auto job_index = job.index();
  Eval previous_eval;
  Eval next_eval;
  Eval old_edge_eval;
  std::optional<Index> previous_index;

  // Only considering service here, setup is handled down the line.
  Duration added_task_duration = job.services[v.type];

  if (rank == route.size()) {
    if (route.empty()) {
      if (v.has_start()) {
        previous_index = v.start.value().index();
        previous_eval = v.eval(previous_index.value(), job_index);
      }
      if (v.has_end()) {
        next_eval = v.eval(job_index, v.end.value().index());
      }
    } else {
      // Adding job past the end after a real job.
      previous_index = input.jobs[route[rank - 1]].index();
      previous_eval = v.eval(previous_index.value(), job_index);

      if (v.has_end()) {
        auto n_index = v.end.value().index();
        old_edge_eval = v.eval(previous_index.value(), n_index);
        next_eval = v.eval(job_index, n_index);
      }
    }
  } else {
    // Adding before one of the jobs.
    auto next_index = input.jobs[route[rank]].index();
    next_eval = v.eval(job_index, next_index);

    if (rank == 0) {
      if (v.has_start()) {
        previous_index = v.start.value().index();
        previous_eval = v.eval(previous_index.value(), job_index);
        old_edge_eval = v.eval(previous_index.value(), next_index);
      }
    } else {
      previous_index = input.jobs[route[rank - 1]].index();
      previous_eval = v.eval(previous_index.value(), job_index);
      old_edge_eval = v.eval(previous_index.value(), next_index);
    }

    if (previous_index.has_value()) {
      if (next_index == job_index && previous_index.value() != next_index) {
        added_task_duration -= input.jobs[route[rank]].setups[v.type];
      }
      if (next_index != job_index && previous_index.value() == next_index) {
        added_task_duration += input.jobs[route[rank]].setups[v.type];
      }
    } else {
      if (next_index == job_index) {
        added_task_duration -= input.jobs[route[rank]].setups[v.type];
      }
    }
  }

  if (!previous_index.has_value() || (previous_index.value() != job_index)) {
    added_task_duration += job.setups[v.type];
  }

  return previous_eval + next_eval - old_edge_eval +
         v.task_eval(added_task_duration);
}

// Evaluate adding pickup with rank job_rank and associated delivery
// (with rank job_rank + 1) in given route for vehicle v. Pickup is
// inserted at pickup_rank in route and delivery is inserted at
// delivery_rank in route **with pickup**.
inline Eval addition_eval(const Input& input,
                          Index job_rank,
                          const Vehicle& v,
                          const std::vector<Index>& route,
                          Index pickup_rank,
                          Index delivery_rank) {
  assert(pickup_rank < delivery_rank && delivery_rank <= route.size() + 1);

  // Start with pickup eval.
  auto eval = addition_eval(input, job_rank, v, route, pickup_rank);

  if (delivery_rank == pickup_rank + 1) {
    // Delivery is inserted just after pickup.
    const auto p_index = input.jobs[job_rank].index();
    const auto& d_job = input.jobs[job_rank + 1];
    const auto d_index = d_job.index();
    eval += v.eval(p_index, d_index);

    Eval after_delivery;
    Eval remove_after_pickup;

    Duration added_task_duration = d_job.services[v.type];
    if (d_index != p_index) {
      added_task_duration += d_job.setups[v.type];
    }

    if (pickup_rank == route.size()) {
      // Addition at the end of a route.
      if (v.has_end()) {
        after_delivery = v.eval(d_index, v.end.value().index());
        remove_after_pickup = v.eval(p_index, v.end.value().index());
      }
    } else {
      // There is a job after insertion.
      const auto& next_job = input.jobs[route[pickup_rank]];
      const auto next_index = next_job.index();
      after_delivery = v.eval(d_index, next_index);
      remove_after_pickup = v.eval(p_index, next_index);

      if (next_index == d_index && p_index != next_index) {
        added_task_duration -= next_job.setups[v.type];
      }
      if (next_index != d_index && p_index == next_index) {
        added_task_duration += next_job.setups[v.type];
      }
    }

    eval += after_delivery;
    eval -= remove_after_pickup;

    eval += v.task_eval(added_task_duration);
  } else {
    // Delivery is further away so edges sets for pickup and delivery
    // addition are disjoint.
    eval += addition_eval(input, job_rank + 1, v, route, delivery_rank - 1);
  }

  return eval;
}

inline auto get_indices(const Input& input,
                        const RawRoute& route,
                        Index first_rank,
                        Index last_rank) {
  const auto& r = route.route;
  const auto& v = input.vehicles[route.v_rank];

  std::array<std::optional<Index>, 3> indices;

  auto& before_first = indices[0];
  if (first_rank > 0) {
    before_first = input.jobs[r[first_rank - 1]].index();
  } else {
    if (v.has_start()) {
      before_first = v.start.value().index();
    }
  }

  auto& first_index = indices[1];
  if (first_rank < r.size()) {
    first_index = input.jobs[r[first_rank]].index();
  } else {
    if (v.has_end()) {
      first_index = v.end.value().index();
    }
  }

  auto& last_index = indices[2];
  if (last_rank < r.size()) {
    last_index = input.jobs[r[last_rank]].index();
  } else {
    if (v.has_end()) {
      last_index = v.end.value().index();
    }
  }

  return indices;
}

inline Eval get_range_removal_gain(const SolutionState& sol_state,
                                   Index v,
                                   Index first_rank,
                                   Index last_rank) {
  assert(first_rank <= last_rank);

  Eval removal_gain;

  if (last_rank > first_rank) {
    // Gain related to removed portion.
    removal_gain += sol_state.fwd_evals[v][v][last_rank - 1];
    removal_gain -= sol_state.fwd_evals[v][v][first_rank];

    removal_gain += sol_state.fwd_setup_evals[v][v][last_rank - 1];
    removal_gain += sol_state.service_evals[v][v][last_rank - 1];
    if (first_rank > 0) {
      removal_gain -= sol_state.fwd_setup_evals[v][v][first_rank - 1];
      removal_gain -= sol_state.service_evals[v][v][first_rank - 1];
    }
  }

  return removal_gain;
}

// Compute cost variation when replacing the [first_rank, last_rank)
// portion for route1 with the *non-empty* range [insertion_start;
// insertion_end) from route_2. Returns a tuple to evaluate at once
// both options where new range is inserted as is, or reversed.
inline std::tuple<Eval, Eval>
addition_eval_delta(const Input& input,
                    const SolutionState& sol_state,
                    const RawRoute& route_1,
                    const Index first_rank,
                    const Index last_rank,
                    const RawRoute& route_2,
                    const Index insertion_start,
                    const Index insertion_end) {
  assert(first_rank <= last_rank);
  assert(last_rank <= route_1.route.size());
  assert(insertion_start < insertion_end);
  assert((first_rank < last_rank) || (insertion_start < insertion_end));

  const auto& r1 = route_1.route;
  const auto v1_rank = route_1.v_rank;
  const auto& r2 = route_2.route;
  const auto v2_rank = route_2.v_rank;
  const auto& v1 = input.vehicles[v1_rank];

  // Common part of the cost.
  Eval cost_delta =
    get_range_removal_gain(sol_state, v1_rank, first_rank, last_rank);

  // Tasks service eval.
  Eval service_delta =
    -sol_state.service_evals[v2_rank][v1_rank][insertion_end - 1];
  if (insertion_start > 0) {
    service_delta +=
      sol_state.service_evals[v2_rank][v1_rank][insertion_start - 1];
  }

  // Part of the cost that may depend on insertion orientation.

  // Edges cost eval.
  Eval straight_delta = sol_state.fwd_evals[v2_rank][v1_rank][insertion_start];
  straight_delta -= sol_state.fwd_evals[v2_rank][v1_rank][insertion_end - 1];

  Eval reversed_delta = sol_state.bwd_evals[v2_rank][v1_rank][insertion_start];
  reversed_delta -= sol_state.bwd_evals[v2_rank][v1_rank][insertion_end - 1];

  // Tasks setup eval, this purposefully does not include setup time
  // for the first job in the previous route context (using
  // insertion_start, not the previous rank).
  straight_delta -=
    sol_state.fwd_setup_evals[v2_rank][v1_rank][insertion_end - 1];
  straight_delta +=
    sol_state.fwd_setup_evals[v2_rank][v1_rank][insertion_start];

  reversed_delta -=
    sol_state.bwd_setup_evals[v2_rank][v1_rank][insertion_start];
  reversed_delta +=
    sol_state.bwd_setup_evals[v2_rank][v1_rank][insertion_end - 1];

  // Determine useful values if present.
  const auto [before_first, first_index, last_index] =
    get_indices(input, route_1, first_rank, last_rank);

  // Gain of removed edge before replaced range. If route is empty,
  // before_first and first_index are respectively the start and end
  // of vehicle if defined.
  if (before_first.has_value() && first_index.has_value() && !r1.empty()) {
    cost_delta += v1.eval(before_first.value(), first_index.value());
  }

  if (before_first.has_value()) {
    // Cost of new edge to inserted range.
    straight_delta -=
      v1.eval(before_first.value(), input.jobs[r2[insertion_start]].index());
    reversed_delta -=
      v1.eval(before_first.value(), input.jobs[r2[insertion_end - 1]].index());
  }

  if (last_index.has_value()) {
    // Cost of new edge after inserted range.
    straight_delta -=
      v1.eval(input.jobs[r2[insertion_end - 1]].index(), last_index.value());
    reversed_delta -=
      v1.eval(input.jobs[r2[insertion_start]].index(), last_index.value());
  }

  // Gain of removed edge after replaced range, if any.
  if (last_index.has_value() && last_rank > first_rank) {
    const Index before_last = input.jobs[r1[last_rank - 1]].index();
    cost_delta += v1.eval(before_last, last_index.value());
  }

  // Handle fixed cost addition.
  if (r1.empty()) {
    cost_delta.cost -= v1.fixed_cost();
  }

  // Handle setup delta at the beginning and end of replaced range.
  Duration straight_task_setup = 0;
  Duration reversed_task_setup = 0;

  // We do insert stuff.
  const auto& first_inserted = input.jobs[r2[insertion_start]];
  const auto first_inserted_index = first_inserted.index();
  const auto& last_inserted = input.jobs[r2[insertion_end - 1]];
  const auto last_inserted_index = last_inserted.index();

  if (!before_first.has_value() ||
      before_first.value() != first_inserted_index) {
    straight_task_setup -= first_inserted.setups[v1.type];
  }
  if (!before_first.has_value() ||
      before_first.value() != last_inserted_index) {
    reversed_task_setup -= last_inserted.setups[v1.type];
  }

  if (last_rank < r1.size()) {
    // There are remaining jobs after removed range.
    const auto& next_job = input.jobs[r1[last_rank]];
    const auto next_index = next_job.index();
    const std::optional<Index> previous_index =
      (last_rank > first_rank) ? input.jobs[r1[last_rank - 1]].index()
                               : before_first;

    if (!previous_index.has_value()) {
      if (last_inserted_index == next_index) {
        straight_task_setup += next_job.setups[v1.type];
      }
      if (first_inserted_index == next_index) {
        reversed_task_setup += next_job.setups[v1.type];
      }
    } else {
      if (next_index == last_inserted_index &&
          previous_index.value() != next_index) {
        straight_task_setup += next_job.setups[v1.type];
      }
      if (next_index != last_inserted_index &&
          previous_index.value() == next_index) {
        straight_task_setup -= next_job.setups[v1.type];
      }

      if (next_index == first_inserted_index &&
          previous_index.value() != next_index) {
        reversed_task_setup += next_job.setups[v1.type];
      }
      if (next_index != first_inserted_index &&
          previous_index.value() == next_index) {
        reversed_task_setup -= next_job.setups[v1.type];
      }
    }
  }

  return std::make_tuple(cost_delta + service_delta + straight_delta +
                           v1.task_eval(straight_task_setup),
                         cost_delta + service_delta + reversed_delta +
                           v1.task_eval(reversed_task_setup));
}

// Compute cost variation when replacing the *non-empty* [first_rank,
// last_rank) portion for route raw_route with the job at
// job_rank. The case where the replaced range is empty is already
// covered by addition_eval.
inline Eval addition_eval_delta(const Input& input,
                                const SolutionState& sol_state,
                                const RawRoute& raw_route,
                                Index first_rank,
                                Index last_rank,
                                Index job_rank) {
  assert(first_rank < last_rank && !raw_route.empty());
  assert(last_rank <= raw_route.route.size());

  const auto& r = raw_route.route;
  const auto v_rank = raw_route.v_rank;
  const auto& v = input.vehicles[v_rank];
  const auto& job = input.jobs[job_rank];
  const auto job_index = job.index();

  Eval cost_delta =
    get_range_removal_gain(sol_state, v_rank, first_rank, last_rank);

  // Determine useful values if present.
  const auto [before_first, first_index, last_index] =
    get_indices(input, raw_route, first_rank, last_rank);

  // Gain of removed edge before replaced range.
  if (before_first.has_value() && first_index.has_value()) {
    cost_delta += v.eval(before_first.value(), first_index.value());
  }

  if (before_first.has_value()) {
    // Cost of new edge to inserted job.
    cost_delta -= v.eval(before_first.value(), job_index);
  }

  if (last_index.has_value()) {
    // Cost of new edge after inserted job.
    cost_delta -= v.eval(job_index, last_index.value());
  }

  // Gain of removed edge after replaced range, if any.
  if (last_index.has_value()) {
    const Index before_last = input.jobs[r[last_rank - 1]].index();
    cost_delta += v.eval(before_last, last_index.value());
  }

  // Handle service/setup delta.
  Duration added_task_duration = job.services[v.type];

  if (last_rank < r.size()) {
    // There are remaining jobs after replaced range.
    const auto& next_job = input.jobs[r[last_rank]];
    const auto next_index = next_job.index();
    const auto previous_index = input.jobs[r[last_rank - 1]].index();

    if (next_index == job_index && previous_index != next_index) {
      added_task_duration -= next_job.setups[v.type];
    }
    if (next_index != job_index && previous_index == next_index) {
      added_task_duration += next_job.setups[v.type];
    }
  }

  if (!before_first || before_first.value() != job_index) {
    added_task_duration += job.setups[v.type];
  }

  return cost_delta - v.task_eval(added_task_duration);
}

// Compute cost variation when removing the range [first_rank,
// last_rank) from route.
inline Eval removal_gain(const Input& input,
                         const SolutionState& sol_state,
                         const RawRoute& route,
                         const Index first_rank,
                         const Index last_rank) {
  assert(!route.empty());
  assert(first_rank < last_rank);
  assert(last_rank <= route.route.size());

  const auto& r = route.route;
  const auto v_rank = route.v_rank;
  const auto& v = input.vehicles[v_rank];

  // Common part of the cost.
  Eval cost_delta =
    get_range_removal_gain(sol_state, v_rank, first_rank, last_rank);

  const bool emptying_route = first_rank == 0 && last_rank == r.size();
  if (emptying_route) {
    cost_delta.cost += v.fixed_cost();
  }

  // Determine useful values if present.
  const auto [before_first, first_index, last_index] =
    get_indices(input, route, first_rank, last_rank);
  assert(first_index.has_value());

  // Gain of removed edge before replaced range. If route is empty,
  // before_first and first_index are respectively the start and end
  // of vehicle if defined.
  if (before_first.has_value()) {
    cost_delta += v.eval(before_first.value(), first_index.value());
  }

  if (before_first.has_value() && last_index.has_value() && !emptying_route) {
    // Add cost of new edge replacing removed range, except if
    // resulting route is empty.
    cost_delta -= v.eval(before_first.value(), last_index.value());
  }

  // Gain of removed edge after replaced range, if any.
  if (last_index.has_value()) {
    const Index before_last = input.jobs[r[last_rank - 1]].index();
    cost_delta += v.eval(before_last, last_index.value());
  }

  if (last_rank < r.size()) {
    // There are remaining jobs after removed range.
    const auto& next_job = input.jobs[r[last_rank]];
    const auto next_index = next_job.index();
    const auto previous_index = input.jobs[r[last_rank - 1]].index();

    const bool before_same_as_next =
      before_first.has_value() && before_first.value() == next_index;

    if (before_same_as_next && previous_index != next_index) {
      cost_delta += v.task_eval(next_job.setups[v.type]);
    }
    if (!before_same_as_next && previous_index == next_index) {
      cost_delta -= v.task_eval(next_job.setups[v.type]);
    }
  }

  return cost_delta;
}

inline Eval max_edge_eval(const Input& input,
                          const Vehicle& v,
                          const std::vector<Index>& route) {
  Eval max_eval;

  if (!route.empty()) {
    if (v.has_start()) {
      const auto start_to_first =
        v.eval(v.start.value().index(), input.jobs[route.front()].index());
      max_eval = std::max(max_eval, start_to_first);
    }

    for (std::size_t i = 0; i < route.size() - 1; ++i) {
      const auto job_to_next =
        v.eval(input.jobs[route[i]].index(), input.jobs[route[i + 1]].index());
      max_eval = std::max(max_eval, job_to_next);
    }

    if (v.has_end()) {
      const auto last_to_end =
        v.eval(input.jobs[route.back()].index(), v.end.value().index());
      max_eval = std::max(max_eval, last_to_end);
    }
  }

  return max_eval;
}

// Helper function for SwapStar operator, computing part of the eval
// for in-place replacing of job at rank in route r with job at
// job_rank.
inline Eval in_place_delta_eval(const Input& input,
                                Index job_rank,
                                const Vehicle& v,
                                const std::vector<Index>& r,
                                Index rank) {
  assert(!r.empty());
  const auto& job = input.jobs[job_rank];
  const auto job_index = job.index();

  Eval new_previous_eval;
  Eval new_next_eval;
  std::optional<Index> p_index;
  std::optional<Index> n_index;

  if (rank == 0) {
    if (v.has_start()) {
      p_index = v.start.value().index();
      new_previous_eval = v.eval(p_index.value(), job_index);
    }
  } else {
    p_index = input.jobs[r[rank - 1]].index();
    new_previous_eval = v.eval(p_index.value(), job_index);
  }

  if (rank == r.size() - 1) {
    if (v.has_end()) {
      n_index = v.end.value().index();
      new_next_eval = v.eval(job_index, n_index.value());
    }
  } else {
    n_index = input.jobs[r[rank + 1]].index();
    new_next_eval = v.eval(job_index, n_index.value());
  }

  Eval old_virtual_eval;
  if (p_index.has_value() && n_index.has_value()) {
    old_virtual_eval = v.eval(p_index.value(), n_index.value());
  }

  Duration added_task_duration = job.services[v.type];

  if (rank + 1u < r.size()) {
    // There is a next job after inserted job.
    const auto& next_job = input.jobs[r[rank + 1]];
    const auto next_index = next_job.index();

    const bool before_same_as_next =
      p_index.has_value() && p_index.value() == next_index;

    if (before_same_as_next && job_index != next_index) {
      added_task_duration += next_job.setups[v.type];
    }
    if (!before_same_as_next && job_index == next_index) {
      added_task_duration -= next_job.setups[v.type];
    }
  }

  if (!p_index || p_index.value() != job_index) {
    added_task_duration += job.setups[v.type];
  }

  return new_previous_eval + new_next_eval - old_virtual_eval +
         v.task_eval(added_task_duration);
}

Priority priority_sum_for_route(const Input& input,
                                const std::vector<Index>& route);

Eval route_eval_for_vehicle(const Input& input,
                            Index vehicle_rank,
                            const std::vector<Index>& route);

// Fast reject on travel, distance, and task time (zero wait) before TWRoute simulation.
bool route_jobs_pass_range_pre_filter(const Input& input,
                                      Index vehicle_rank,
                                      const std::vector<Index>& jobs);

inline bool route_jobs_within_capacity(const Input& input,
                                      Index vehicle_rank,
                                      const std::vector<Index>& jobs) {
  return RawRoute::jobs_within_capacity(input, vehicle_rank, jobs);
}

inline bool routes_within_capacity(const Input& input,
                                  Index v1,
                                  const std::vector<Index>& jobs1,
                                  Index v2,
                                  const std::vector<Index>& jobs2) {
  return route_jobs_within_capacity(input, v1, jobs1) &&
         route_jobs_within_capacity(input, v2, jobs2);
}

bool route_jobs_within_max_duration(const Input& input,
                                    Index vehicle_rank,
                                    const std::vector<Index>& jobs);

// VRPTW LS: when tw_live is set, rebuild post-move jobs on a thread-local
// scratch copy of the live route so billable wait matches apply().
bool route_jobs_within_max_duration_for_ls(const Input& input,
                                           Index vehicle_rank,
                                           const std::vector<Index>& jobs,
                                           const TWRoute* tw_live = nullptr);

template <class Route>
bool route_after_jobs_within_max_duration(const Input& input,
                                          Index vehicle_rank,
                                          const std::vector<Index>& jobs,
                                          const Route& route) {
  if constexpr (std::is_same_v<Route, TWRoute>) {
    return route_jobs_within_max_duration_for_ls(input,
                                                 vehicle_rank,
                                                 jobs,
                                                 &route);
  }
  return route_jobs_within_max_duration(input, vehicle_rank, jobs);
}

inline bool routes_within_max_duration(const Input& input,
                                       Index v1,
                                       const std::vector<Index>& jobs1,
                                       Index v2,
                                       const std::vector<Index>& jobs2) {
  return route_jobs_within_max_duration(input, v1, jobs1) &&
         route_jobs_within_max_duration(input, v2, jobs2);
}

inline bool routes_within_max_duration_for_ls(const Input& input,
                                              Index v1,
                                              const std::vector<Index>& jobs1,
                                              Index v2,
                                              const std::vector<Index>& jobs2,
                                              const TWRoute* tw1 = nullptr,
                                              const TWRoute* tw2 = nullptr) {
  if (input.has_bounded_max_duration()) {
    const auto fails_cheap_checks = [&](Index v,
                                        const std::vector<Index>& jobs) {
      return !jobs.empty() &&
             (!RawRoute::jobs_within_capacity(input, v, jobs) ||
              !route_jobs_pass_range_pre_filter(input, v, jobs));
    };
    if (fails_cheap_checks(v1, jobs1) || fails_cheap_checks(v2, jobs2)) {
      return false;
    }
  }
  return route_jobs_within_max_duration_for_ls(input, v1, jobs1, tw1) &&
         route_jobs_within_max_duration_for_ls(input, v2, jobs2, tw2);
}

// Skip exact wait-cost adjustment when the move cannot beat current best even
// if all prior wait cost were removed (used in adjust_stored_gain_for_wait_* only).
bool skip_ls_wait_pruning(const Eval& stored_gain,
                           const std::optional<Cost>& wait_ub,
                           const Eval& best_known);

template <typename RouteCheck>
bool max_duration_feasible_for_ls(const Input& input,
                                  RouteCheck&& route_check) {
  if (!input.has_bounded_max_duration()) {
    return true;
  }
  return route_check();
}

// Post-move job sequences for VRPTW local search (match operator apply paths).
void build_relocate_post_routes(const std::vector<Index>& s_route,
                                Index s_rank,
                                const std::vector<Index>& t_route,
                                Index t_rank,
                                std::vector<Index>& source_after,
                                std::vector<Index>& target_after);

void build_two_opt_post_routes(const std::vector<Index>& s_route,
                               Index s_rank,
                               const std::vector<Index>& t_route,
                               Index t_rank,
                               std::vector<Index>& source_after,
                               std::vector<Index>& target_after);

void build_reverse_two_opt_post_routes(const std::vector<Index>& s_route,
                                       Index s_rank,
                                       const std::vector<Index>& t_route,
                                       Index t_rank,
                                       std::vector<Index>& source_after,
                                       std::vector<Index>& target_after);

void build_or_opt_post_routes(const std::vector<Index>& s_route,
                              Index s_rank,
                              const std::vector<Index>& t_route,
                              Index t_rank,
                              bool reverse_s_edge,
                              std::vector<Index>& source_after,
                              std::vector<Index>& target_after);

void build_cross_exchange_post_routes(const std::vector<Index>& s_route,
                                      Index s_rank,
                                      const std::vector<Index>& t_route,
                                      Index t_rank,
                                      bool reverse_s_edge,
                                      bool reverse_t_edge,
                                      std::vector<Index>& source_after,
                                      std::vector<Index>& target_after);

void build_mixed_exchange_post_routes(const std::vector<Index>& s_route,
                                      Index s_rank,
                                      const std::vector<Index>& t_route,
                                      Index t_rank,
                                      bool reverse_t_edge,
                                      std::vector<Index>& source_after,
                                      std::vector<Index>& target_after);

void build_intra_relocate_post_route(const std::vector<Index>& route,
                                     Index s_rank,
                                     Index t_rank,
                                     std::vector<Index>& route_after);

void build_one_route_after_moved_jobs(
  const std::vector<Index>& route,
  Index first_rank,
  const std::vector<Index>& moved_jobs,
  std::vector<Index>& route_after);

// Edge reversal flags chosen in operator compute_gain() (not every valid combo).
bool edge_swap_chosen_reverse(Eval normal_gain,
                              Eval reversed_gain,
                              bool is_normal_valid,
                              bool is_reverse_valid);

std::pair<bool, bool> intra_cross_exchange_chosen_reverse_edges(
  Eval normal_s_gain,
  Eval reversed_s_gain,
  Eval normal_t_gain,
  Eval reversed_t_gain,
  bool s_normal_t_normal_is_valid,
  bool s_normal_t_reverse_is_valid,
  bool s_reverse_t_normal_is_valid,
  bool s_reverse_t_reverse_is_valid);

std::pair<bool, bool> cross_exchange_chosen_reverse_edges(
  Eval normal_s_gain,
  Eval reversed_s_gain,
  bool s_is_normal_valid,
  bool s_is_reverse_valid,
  Eval normal_t_gain,
  Eval reversed_t_gain,
  bool t_is_normal_valid,
  bool t_is_reverse_valid);

bool cross_exchange_within_max_duration(
  const Input& input,
  Index s_vehicle,
  const std::vector<Index>& s_route,
  Index s_rank,
  Index t_vehicle,
  const std::vector<Index>& t_route,
  Index t_rank,
  bool s_is_normal_valid,
  bool s_is_reverse_valid,
  bool t_is_normal_valid,
  bool t_is_reverse_valid,
  Eval normal_s_gain,
  Eval reversed_s_gain,
  Eval normal_t_gain,
  Eval reversed_t_gain,
  const TWRoute* tw_s = nullptr,
  const TWRoute* tw_t = nullptr);

bool or_opt_within_max_duration(const Input& input,
                                Index s_vehicle,
                                const std::vector<Index>& s_route,
                                Index s_rank,
                                Index t_vehicle,
                                const std::vector<Index>& t_route,
                                Index t_rank,
                                bool is_normal_valid,
                                bool is_reverse_valid,
                                Eval normal_t_gain,
                                Eval reversed_t_gain,
                                const TWRoute* tw_s = nullptr,
                                const TWRoute* tw_t = nullptr);

bool mixed_exchange_within_max_duration(
  const Input& input,
  Index s_vehicle,
  const std::vector<Index>& s_route,
  Index s_rank,
  Index t_vehicle,
  const std::vector<Index>& t_route,
  Index t_rank,
  bool s_is_normal_valid,
  bool s_is_reverse_valid,
  Eval normal_s_gain,
  Eval reversed_s_gain,
  const TWRoute* tw_s = nullptr,
  const TWRoute* tw_t = nullptr);

bool insertion_respects_vehicle_bounds(const Input& input,
                                       Index vehicle_rank,
                                       const Eval& route_eval,
                                       const Eval& insertion_eval,
                                       const std::vector<Index>& route,
                                       Index job_rank,
                                       Index rank,
                                       const TWRoute* tw_live = nullptr);

template <class Route>
bool insertion_respects_vehicle_bounds_for_route(
  const Input& input,
  Index vehicle_rank,
  const Eval& route_eval,
  const Eval& insertion_eval,
  const Route& route,
  Index job_rank,
  Index rank) {
  if constexpr (std::is_same_v<Route, TWRoute>) {
    return insertion_respects_vehicle_bounds(input,
                                            vehicle_rank,
                                            route_eval,
                                            insertion_eval,
                                            route.route,
                                            job_rank,
                                            rank,
                                            &route);
  }
  return insertion_respects_vehicle_bounds(input,
                                           vehicle_rank,
                                           route_eval,
                                           insertion_eval,
                                           route.route,
                                           job_rank,
                                           rank);
}

// Approximate billable wait: depot slack plus waits at jobs, forward from
// fixed_departure without backward re-optimization on `route`. Skips vehicles
// with breaks. nullopt if a job time window cannot absorb forward time.
std::optional<Duration> approx_billable_wait_jobs_only(
  const Input& input,
  Index vehicle_rank,
  const std::vector<Index>& route,
  Duration fixed_departure);

// Billable wait for `jobs` inserted in one batch on an empty route (matches
// RouteSplit::apply and custom routes). nullopt if TW-infeasible.
std::optional<Duration>
billable_wait_for_job_sequence_via_empty_replace(const Input& input,
                                                 Index vehicle_rank,
                                                 const std::vector<Index>& jobs);

// Billable wait duration for `jobs` in order on `vehicle_rank`, aligned with
// route evaluation: scratch TWRoute, then refresh_billable_total_wait_for_eval.
// Multi-job sequences with mandatory breaks use empty-route batch replace.
// nullopt if TW infeasible for that sequence.
std::optional<Duration> billable_wait_for_job_sequence_aligned_with_route_eval(
  const Input& input,
  Index vehicle_rank,
  const std::vector<Index>& jobs);

// Wait cost for `jobs` on `vehicle_rank`. Uses `tw_if_matches` when its route
// equals `jobs` (reuses billable_total_wait) instead of rebuilding a scratch TWRoute.
std::optional<Cost> wait_cost_for_job_sequence(const Input& input,
                                               Index vehicle_rank,
                                               const std::vector<Index>& jobs,
                                               const TWRoute* tw_if_matches);

// Upper bound on wait gain (old_wait_cost - new_wait_cost) with new_wait >= 0.
// nullopt if uncertain (breaks, infeasible approx, etc.) — do not prune on wait.
std::optional<Cost> wait_gain_upper_bound_from_routes(
  const Input& input,
  Index v1,
  const std::vector<Index>& r1,
  const TWRoute* tw_r1,
  Index v2,
  const std::vector<Index>& r2,
  const TWRoute* tw_r2);

inline std::optional<Cost>
wait_gain_upper_bound_from_route(const Input& input,
                                 Index v,
                                 const std::vector<Index>& r,
                                 const TWRoute* tw_r) {
  return wait_gain_upper_bound_from_routes(input, v, r, tw_r, v, r, tw_r);
}

template <class Route>
std::optional<Cost> wait_gain_upper_bound_for_ls_route(const Input& input,
                                                       Index v,
                                                       const Route& route) {
  if constexpr (std::is_same_v<Route, TWRoute>) {
    return wait_gain_upper_bound_from_route(input, v, route.route, &route);
  }
  return std::nullopt;
}

template <class Route>
std::optional<Cost>
wait_gain_upper_bound_for_ls_relocate(const Input& input,
                                      Index source,
                                      const Route& s_route,
                                      Index target,
                                      const Route& t_route) {
  if constexpr (std::is_same_v<Route, TWRoute>) {
    return wait_gain_upper_bound_from_routes(input,
                                             source,
                                             s_route.route,
                                             &s_route,
                                             target,
                                             t_route.route,
                                             &t_route);
  }
  return std::nullopt;
}

// Fast wait cost from forward-only approx (no TWRoute rebuild). nullopt if unknown.
std::optional<Cost> wait_cost_approx_job_sequence(const Input& input,
                                                  Index vehicle_rank,
                                                  const std::vector<Index>& jobs);

// Marginal wait cost (old - new) for inserting into `route_with_insertion` vs
// current `route` state. Returns 0 when wait is not in the objective.
Cost wait_insertion_marginal_cost(const Input& input,
                                  const TWRoute& route,
                                  const std::vector<Index>& route_with_insertion);

// VRPTW local search: add (old_wait_cost - new_wait_cost) to gain.cost for
// vehicles with per_wait_hour and no breaks.
void adjust_stored_gain_for_wait_approx_two_routes(
  const Input& input,
  Eval& stored_gain,
  Index v1,
  const std::vector<Index>& r1_old,
  const std::vector<Index>& r1_new,
  Index v2,
  const std::vector<Index>& r2_old,
  const std::vector<Index>& r2_new,
  const TWRoute* tw_r1_old = nullptr,
  const TWRoute* tw_r2_old = nullptr,
  Eval best_known = NO_EVAL);

// VRPTW LS: wait gain adjustment after cvrp::compute_gain, by operator family.
void adjust_relocate_wait_gain(const Input& input,
                               Eval& stored_gain,
                               Index s_vehicle,
                               const std::vector<Index>& s_route,
                               Index s_rank,
                               Index t_vehicle,
                               const std::vector<Index>& t_route,
                               Index t_rank,
                               const TWRoute* tw_s_route,
                               const TWRoute* tw_t_route,
                               Eval best_known = NO_EVAL);

void adjust_two_opt_wait_gain(const Input& input,
                              Eval& stored_gain,
                              Index s_vehicle,
                              const std::vector<Index>& s_route,
                              Index s_rank,
                              Index t_vehicle,
                              const std::vector<Index>& t_route,
                              Index t_rank,
                              const TWRoute* tw_s_route,
                              const TWRoute* tw_t_route,
                              Eval best_known = NO_EVAL);

void adjust_reverse_two_opt_wait_gain(const Input& input,
                                      Eval& stored_gain,
                                      Index s_vehicle,
                                      const std::vector<Index>& s_route,
                                      Index s_rank,
                                      Index t_vehicle,
                                      const std::vector<Index>& t_route,
                                      Index t_rank,
                                      const TWRoute* tw_s_route,
                                      const TWRoute* tw_t_route,
                                      Eval best_known = NO_EVAL);

void adjust_or_opt_wait_gain(const Input& input,
                             Eval& stored_gain,
                             Index s_vehicle,
                             const std::vector<Index>& s_route,
                             Index s_rank,
                             bool reverse_s_edge,
                             Index t_vehicle,
                             const std::vector<Index>& t_route,
                             Index t_rank,
                             const TWRoute* tw_s_route,
                             const TWRoute* tw_t_route,
                             Eval best_known = NO_EVAL);

void adjust_cross_exchange_wait_gain(const Input& input,
                                     Eval& stored_gain,
                                     Index s_vehicle,
                                     const std::vector<Index>& s_route,
                                     Index s_rank,
                                     bool reverse_s_edge,
                                     bool reverse_t_edge,
                                     Index t_vehicle,
                                     const std::vector<Index>& t_route,
                                     Index t_rank,
                                     const TWRoute* tw_s_route,
                                     const TWRoute* tw_t_route,
                                     Eval best_known = NO_EVAL);

void adjust_mixed_exchange_wait_gain(const Input& input,
                                     Eval& stored_gain,
                                     Index s_vehicle,
                                     const std::vector<Index>& s_route,
                                     Index s_rank,
                                     bool reverse_t_edge,
                                     Index t_vehicle,
                                     const std::vector<Index>& t_route,
                                     Index t_rank,
                                     const TWRoute* tw_s_route,
                                     const TWRoute* tw_t_route,
                                     Eval best_known = NO_EVAL);

void adjust_route_exchange_wait_gain(const Input& input,
                                     Eval& stored_gain,
                                     Index s_vehicle,
                                     const std::vector<Index>& s_route,
                                     Index t_vehicle,
                                     const std::vector<Index>& t_route,
                                     const TWRoute* tw_s_route,
                                     const TWRoute* tw_t_route,
                                     Eval best_known = NO_EVAL);

void adjust_one_route_moved_jobs_wait_gain(
  const Input& input,
  Eval& stored_gain,
  Index v,
  const std::vector<Index>& route_old,
  Index first_rank,
  const std::vector<Index>& moved_jobs,
  const TWRoute* tw_route,
  Eval best_known = NO_EVAL);

void adjust_stored_gain_for_wait_approx_one_route(const Input& input,
                                                  Eval& stored_gain,
                                                  Index v,
                                                  const std::vector<Index>& r_old,
                                                  const std::vector<Index>& r_new,
                                                  const TWRoute* tw_r_old = nullptr,
                                                  Eval best_known = NO_EVAL);

void check_tws(const std::vector<TimeWindow>& tws,
               Id id,
               const std::string& type);

void check_priority(Priority priority, Id id, const std::string& type);

void check_no_empty_keys(const TypeToDurationMap& type_to_duration,
                         const Id id,
                         const std::string& type,
                         const std::string& key_name);

using RawSolution = std::vector<RawRoute>;
using TWSolution = std::vector<TWRoute>;

Solution format_solution(const Input& input, const RawSolution& raw_routes);

Route format_route(const Input& input,
                   const TWRoute& tw_r,
                   std::unordered_set<Index>& unassigned_ranks);

// Depot leave time after backward ETA (latest leave minimizing downstream
// waits), capped by optional vehicle latest departure from depot (`departure`).
Duration min_wait_route_departure(const Input& input, const TWRoute& tw_r);

Solution format_solution(const Input& input, const TWSolution& tw_routes);

} // namespace vroom::utils

#endif
