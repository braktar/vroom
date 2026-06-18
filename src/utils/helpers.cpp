/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>
#include <chrono>
#include <functional>
#include <numeric>
#include <optional>
#include <sstream>

#include "utils/helpers.h"

namespace vroom::utils {

TimePoint now() {
  return std::chrono::high_resolution_clock::now();
}

Amount max_amount(std::size_t size) {
  Amount max(size);
  for (std::size_t i = 0; i < size; ++i) {
    max[i] = std::numeric_limits<Capacity>::max();
  }
  return max;
}

Priority priority_sum_for_route(const Input& input,
                                const std::vector<Index>& route) {
  return std::accumulate(route.begin(),
                         route.end(),
                         0,
                         [&](auto sum, auto job_rank) {
                           return sum + input.jobs[job_rank].priority;
                         });
}

Eval route_eval_for_vehicle(const Input& input,
                            Index v_rank,
                            const std::vector<Index>& route) {
  const auto& v = input.vehicles[v_rank];
  Eval eval;

  if (!route.empty()) {
    eval.cost += v.fixed_cost();

    const auto& first_job = input.jobs[route.front()];
    auto jobs_task_duration = first_job.services[v.type];

    if (v.has_start()) {
      eval += v.eval(v.start.value().index(), first_job.index());
    }

    if (!v.has_start() || v.start.value().index() != first_job.index()) {
      jobs_task_duration += first_job.setups[v.type];
    }

    Index previous_index = input.jobs[route.front()].index();
    for (Index i = 1; i < route.size(); ++i) {
      const auto& current_job = input.jobs[route[i]];
      const auto current_index = current_job.index();

      eval += v.eval(previous_index, current_index);

      jobs_task_duration += current_job.services[v.type];
      if (current_index != previous_index) {
        jobs_task_duration += current_job.setups[v.type];
      }

      previous_index = current_index;
    }

    if (v.has_end()) {
      eval += v.eval(previous_index, v.end.value().index());
    }

    eval += v.task_eval(jobs_task_duration);
  }

  return eval;
}

bool route_jobs_within_max_duration(const Input& input,
                                    Index vehicle_rank,
                                    const std::vector<Index>& jobs) {
  if (!RawRoute::jobs_within_capacity(input, vehicle_rank, jobs)) {
    return false;
  }

  const auto& vehicle = input.vehicles[vehicle_rank];
  if (vehicle.max_duration == DEFAULT_MAX_DURATION) {
    return true;
  }

  if (!route_jobs_pass_range_pre_filter(input, vehicle_rank, jobs)) {
    return false;
  }

  auto eval = route_eval_for_vehicle(input, vehicle_rank, jobs);
  if (!jobs.empty()) {
    if (const auto wait =
          billable_wait_for_job_sequence_aligned_with_route_eval(input,
                                                               vehicle_rank,
                                                               jobs)) {
      eval.wait_duration = *wait;
    } else {
      return false;
    }
  }

  return vehicle.ok_for_range_bounds(eval);
}

bool route_jobs_pass_range_pre_filter(const Input& input,
                                      Index vehicle_rank,
                                      const std::vector<Index>& jobs) {
  if (jobs.empty()) {
    return true;
  }

  const auto& vehicle = input.vehicles[vehicle_rank];
  const auto eval = route_eval_for_vehicle(input, vehicle_rank, jobs);
  if (!vehicle.ok_for_travel_time(eval.duration) ||
      !vehicle.ok_for_distance(eval.distance)) {
    return false;
  }
  if (vehicle.max_duration != DEFAULT_MAX_DURATION &&
      eval.duration + eval.task_duration > vehicle.max_duration) {
    return false;
  }
  return true;
}

namespace {

// Reused across LS max_duration / wait-after-edit checks (one per thread).
struct LsTWRouteEvalScratch {
  Index vehicle_rank{std::numeric_limits<Index>::max()};
  unsigned amount_size{0};
  std::optional<TWRoute> tw;

  TWRoute& route(const Input& input, Index v_rank) {
    const auto as = input.get_amount_size();
    if (vehicle_rank != v_rank || amount_size != as || !tw.has_value()) {
      tw.emplace(input, v_rank, as);
      vehicle_rank = v_rank;
      amount_size = as;
    }
    return *tw;
  }
};

thread_local LsTWRouteEvalScratch tls_tw_eval_scratch;

TWRoute& ls_tw_eval_scratch_route(const Input& input, Index vehicle_rank) {
  return tls_tw_eval_scratch.route(input, vehicle_rank);
}

// Apply the edit from tw.route -> jobs with a single partial replace (same as
// add/remove/replace in operators). Full-route replace(0, n, jobs) would
// re-run break ordering on the whole route and underestimate billable wait.
bool apply_jobs_to_tw_route(TWRoute& tw,
                            const Input& input,
                            const std::vector<Index>& jobs) {
  if (tw.route == jobs) {
    return true;
  }

  if (!RawRoute::jobs_within_capacity(input, tw.v_rank, jobs)) {
    return false;
  }

  const auto& old = tw.route;
  Index i = 0;
  while (i < static_cast<Index>(old.size()) && i < static_cast<Index>(jobs.size()) &&
         old[i] == jobs[i]) {
    ++i;
  }

  Index old_end = static_cast<Index>(old.size());
  Index jobs_end = static_cast<Index>(jobs.size());
  while (old_end > i && jobs_end > i && old[old_end - 1] == jobs[jobs_end - 1]) {
    --old_end;
    --jobs_end;
  }

  Amount delivery = input.zero_amount();
  for (Index k = i; k < jobs_end; ++k) {
    delivery += input.jobs[jobs[k]].delivery;
  }

  if (!tw.is_valid_addition_for_tw(input,
                                   delivery,
                                   jobs.begin() + static_cast<std::ptrdiff_t>(i),
                                   jobs.begin() + static_cast<std::ptrdiff_t>(jobs_end),
                                   i,
                                   old_end)) {
    return false;
  }

  tw.replace(input,
             delivery,
             jobs.begin() + static_cast<std::ptrdiff_t>(i),
             jobs.begin() + static_cast<std::ptrdiff_t>(jobs_end),
             i,
             old_end);
  return true;
}

std::optional<Cost> wait_cost_from_billable_wait(const Vehicle& veh,
                                                 Duration billable_wait) {
  return veh.wait_cost(billable_wait);
}

std::optional<Cost> wait_cost_for_billable_sequence(
  const Input& input,
  Index vehicle_rank,
  const std::vector<Index>& jobs,
  const TWRoute* tw_if_matches) {
  const auto& veh = input.vehicles[vehicle_rank];
  if (veh.costs.per_wait_hour == 0) {
    return Cost{0};
  }
  if (tw_if_matches != nullptr && tw_if_matches->route == jobs) {
    return wait_cost_from_billable_wait(veh, tw_if_matches->billable_total_wait);
  }
  const auto w =
    billable_wait_for_job_sequence_aligned_with_route_eval(input,
                                                           vehicle_rank,
                                                           jobs);
  if (!w.has_value()) {
    return std::nullopt;
  }
  return wait_cost_from_billable_wait(veh, *w);
}

std::optional<Cost> wait_cost_for_job_sequence_after_edit(const Input& input,
                                                          Index vehicle_rank,
                                                          const std::vector<Index>& jobs,
                                                          const TWRoute& tw_live) {
  if (tw_live.route == jobs) {
    return wait_cost_for_billable_sequence(input, vehicle_rank, jobs, &tw_live);
  }

  auto& tw = ls_tw_eval_scratch_route(input, vehicle_rank);
  tw = tw_live;
  if (!apply_jobs_to_tw_route(tw, input, jobs)) {
    return std::nullopt;
  }
  const auto& veh = input.vehicles[vehicle_rank];
  return wait_cost_from_billable_wait(veh, tw.billable_total_wait);
}

bool tw_route_rebuild_and_within_max_duration(const Input& input,
                                              TWRoute& tw,
                                              const std::vector<Index>& jobs) {
  const auto& vehicle = input.vehicles[tw.v_rank];

  if (!jobs.empty() && jobs != tw.route) {
    if (!apply_jobs_to_tw_route(tw, input, jobs)) {
      return false;
    }
  }

  auto eval = route_eval_for_vehicle(input, tw.v_rank, tw.route);
  if (!tw.route.empty()) {
    eval.wait_duration = tw.billable_total_wait;
  }
  return vehicle.ok_for_range_bounds(eval);
}

} // namespace

void build_relocate_post_routes(const std::vector<Index>& s_route,
                                Index s_rank,
                                const std::vector<Index>& t_route,
                                Index t_rank,
                                std::vector<Index>& source_after,
                                std::vector<Index>& target_after) {
  source_after = s_route;
  source_after.erase(source_after.begin() + static_cast<std::ptrdiff_t>(s_rank));
  target_after = t_route;
  target_after.insert(target_after.begin() + static_cast<std::ptrdiff_t>(t_rank),
                      s_route[s_rank]);
}

void build_two_opt_post_routes(const std::vector<Index>& s_route,
                               Index s_rank,
                               const std::vector<Index>& t_route,
                               Index t_rank,
                               std::vector<Index>& source_after,
                               std::vector<Index>& target_after) {
  source_after = s_route;
  target_after = t_route;
  const auto nb_source = source_after.size() - 1 - s_rank;
  target_after.insert(target_after.begin() + static_cast<std::ptrdiff_t>(t_rank) + 1,
                      source_after.begin() + static_cast<std::ptrdiff_t>(s_rank) + 1,
                      source_after.end());
  source_after.erase(source_after.begin() + static_cast<std::ptrdiff_t>(s_rank) + 1,
                     source_after.end());
  source_after.insert(source_after.end(),
                      target_after.begin() + static_cast<std::ptrdiff_t>(t_rank) + 1 +
                        static_cast<std::ptrdiff_t>(nb_source),
                      target_after.end());
  target_after.erase(target_after.begin() + static_cast<std::ptrdiff_t>(t_rank) + 1 +
                       static_cast<std::ptrdiff_t>(nb_source),
                     target_after.end());
}

void build_reverse_two_opt_post_routes(const std::vector<Index>& s_route,
                                       Index s_rank,
                                       const std::vector<Index>& t_route,
                                       Index t_rank,
                                       std::vector<Index>& source_after,
                                       std::vector<Index>& target_after) {
  source_after = s_route;
  target_after = t_route;
  const auto nb_source = source_after.size() - 1 - s_rank;
  target_after.insert(target_after.begin(),
                      source_after.rbegin(),
                      source_after.rbegin() + static_cast<std::ptrdiff_t>(nb_source));
  source_after.erase(source_after.begin() + static_cast<std::ptrdiff_t>(s_rank) + 1,
                     source_after.end());
  source_after.insert(source_after.end(),
                      target_after.rend() - static_cast<std::ptrdiff_t>(t_rank) -
                        static_cast<std::ptrdiff_t>(nb_source) - 1,
                      target_after.rend() - static_cast<std::ptrdiff_t>(nb_source));
  target_after.erase(target_after.begin() + static_cast<std::ptrdiff_t>(nb_source),
                     target_after.begin() + static_cast<std::ptrdiff_t>(nb_source) +
                       static_cast<std::ptrdiff_t>(t_rank) + 1);
}

void build_or_opt_post_routes(const std::vector<Index>& s_route,
                              Index s_rank,
                              const std::vector<Index>& t_route,
                              Index t_rank,
                              bool reverse_s_edge,
                              std::vector<Index>& source_after,
                              std::vector<Index>& target_after) {
  source_after = s_route;
  target_after = t_route;
  target_after.insert(target_after.begin() + static_cast<std::ptrdiff_t>(t_rank),
                      source_after.begin() + static_cast<std::ptrdiff_t>(s_rank),
                      source_after.begin() + static_cast<std::ptrdiff_t>(s_rank) + 2);
  if (reverse_s_edge) {
    std::swap(target_after[t_rank], target_after[t_rank + 1]);
  }
  source_after.erase(source_after.begin() + static_cast<std::ptrdiff_t>(s_rank),
                     source_after.begin() + static_cast<std::ptrdiff_t>(s_rank) + 2);
}

void build_cross_exchange_post_routes(const std::vector<Index>& s_route,
                                      Index s_rank,
                                      const std::vector<Index>& t_route,
                                      Index t_rank,
                                      bool reverse_s_edge,
                                      bool reverse_t_edge,
                                      std::vector<Index>& source_after,
                                      std::vector<Index>& target_after) {
  source_after = s_route;
  target_after = t_route;
  std::swap(source_after[s_rank], target_after[t_rank]);
  std::swap(source_after[s_rank + 1], target_after[t_rank + 1]);
  if (reverse_s_edge) {
    std::swap(target_after[t_rank], target_after[t_rank + 1]);
  }
  if (reverse_t_edge) {
    std::swap(source_after[s_rank], source_after[s_rank + 1]);
  }
}

void build_mixed_exchange_post_routes(const std::vector<Index>& s_route,
                                      Index s_rank,
                                      const std::vector<Index>& t_route,
                                      Index t_rank,
                                      bool reverse_t_edge,
                                      std::vector<Index>& source_after,
                                      std::vector<Index>& target_after) {
  source_after = s_route;
  target_after = t_route;
  std::swap(source_after[s_rank], target_after[t_rank]);
  source_after.insert(source_after.begin() + static_cast<std::ptrdiff_t>(s_rank) + 1,
                      target_after.begin() + static_cast<std::ptrdiff_t>(t_rank) + 1,
                      target_after.begin() + static_cast<std::ptrdiff_t>(t_rank) + 2);
  target_after.erase(target_after.begin() + static_cast<std::ptrdiff_t>(t_rank) + 1);
  if (reverse_t_edge) {
    std::swap(source_after[s_rank], source_after[s_rank + 1]);
  }
}

void build_intra_relocate_post_route(const std::vector<Index>& route,
                                     Index s_rank,
                                     Index t_rank,
                                     std::vector<Index>& route_after) {
  route_after = route;
  const auto moved = route_after[s_rank];
  route_after.erase(route_after.begin() + static_cast<std::ptrdiff_t>(s_rank));
  route_after.insert(route_after.begin() + static_cast<std::ptrdiff_t>(t_rank), moved);
}

void build_one_route_after_moved_jobs(
  const std::vector<Index>& route,
  Index first_rank,
  const std::vector<Index>& moved_jobs,
  std::vector<Index>& route_after) {
  route_after = route;
  std::copy(moved_jobs.begin(),
            moved_jobs.end(),
            route_after.begin() + static_cast<std::ptrdiff_t>(first_rank));
}

bool edge_swap_chosen_reverse(Eval normal_gain,
                              Eval reversed_gain,
                              bool is_normal_valid,
                              bool is_reverse_valid) {
  if (normal_gain < reversed_gain) {
    return is_reverse_valid;
  }
  return !is_normal_valid;
}

std::pair<bool, bool> intra_cross_exchange_chosen_reverse_edges(
  Eval normal_s_gain,
  Eval reversed_s_gain,
  Eval normal_t_gain,
  Eval reversed_t_gain,
  bool s_normal_t_normal_is_valid,
  bool s_normal_t_reverse_is_valid,
  bool s_reverse_t_normal_is_valid,
  bool s_reverse_t_reverse_is_valid) {
  Eval best_gain = NO_GAIN;
  bool reverse_s = false;
  bool reverse_t = false;

  auto consider = [&](bool rev_s, bool rev_t, bool valid, Eval gain) {
    if (!valid) {
      return;
    }
    if (best_gain < gain) {
      best_gain = gain;
      reverse_s = rev_s;
      reverse_t = rev_t;
    }
  };

  consider(false, false, s_normal_t_normal_is_valid, normal_s_gain + normal_t_gain);
  consider(false, true, s_normal_t_reverse_is_valid, reversed_s_gain + normal_t_gain);
  consider(true, true, s_reverse_t_reverse_is_valid, reversed_s_gain + reversed_t_gain);
  consider(true, false, s_reverse_t_normal_is_valid, normal_s_gain + reversed_t_gain);

  return {reverse_s, reverse_t};
}

std::pair<bool, bool> cross_exchange_chosen_reverse_edges(
  Eval normal_s_gain,
  Eval reversed_s_gain,
  bool s_is_normal_valid,
  bool s_is_reverse_valid,
  Eval normal_t_gain,
  Eval reversed_t_gain,
  bool t_is_normal_valid,
  bool t_is_reverse_valid) {
  return {edge_swap_chosen_reverse(normal_t_gain,
                                  reversed_t_gain,
                                  t_is_normal_valid,
                                  t_is_reverse_valid),
          edge_swap_chosen_reverse(normal_s_gain,
                                 reversed_s_gain,
                                 s_is_normal_valid,
                                 s_is_reverse_valid)};
}

namespace {

bool edge_swap_with_single_reverse_within_max_duration(
  const Input& input,
  Index s_vehicle,
  Index t_vehicle,
  Eval normal_gain,
  Eval reversed_gain,
  bool is_normal_valid,
  bool is_reverse_valid,
  const TWRoute* tw_s,
  const TWRoute* tw_t,
  const std::function<void(bool reverse,
                           std::vector<Index>&,
                           std::vector<Index>&)>& build_post) {
  if (!input.has_bounded_max_duration()) {
    return true;
  }

  const bool reverse = edge_swap_chosen_reverse(normal_gain,
                                                reversed_gain,
                                                is_normal_valid,
                                                is_reverse_valid);
  if (!(reverse ? is_reverse_valid : is_normal_valid)) {
    return false;
  }

  std::vector<Index> ns;
  std::vector<Index> nt;
  build_post(reverse, ns, nt);
  return routes_within_max_duration_for_ls(input,
                                           s_vehicle,
                                           ns,
                                           t_vehicle,
                                           nt,
                                           tw_s,
                                           tw_t);
}

} // namespace

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
  const TWRoute* tw_s,
  const TWRoute* tw_t) {
  if (!input.has_bounded_max_duration()) {
    return true;
  }

  const auto [reverse_s, reverse_t] = cross_exchange_chosen_reverse_edges(
    normal_s_gain,
    reversed_s_gain,
    s_is_normal_valid,
    s_is_reverse_valid,
    normal_t_gain,
    reversed_t_gain,
    t_is_normal_valid,
    t_is_reverse_valid);

  if (!(reverse_t ? s_is_reverse_valid : s_is_normal_valid) ||
      !(reverse_s ? t_is_reverse_valid : t_is_normal_valid)) {
    return false;
  }

  std::vector<Index> ns;
  std::vector<Index> nt;
  build_cross_exchange_post_routes(s_route,
                                   s_rank,
                                   t_route,
                                   t_rank,
                                   reverse_s,
                                   reverse_t,
                                   ns,
                                   nt);
  return routes_within_max_duration_for_ls(input,
                                           s_vehicle,
                                           ns,
                                           t_vehicle,
                                           nt,
                                           tw_s,
                                           tw_t);
}

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
                                const TWRoute* tw_s,
                                const TWRoute* tw_t) {
  return edge_swap_with_single_reverse_within_max_duration(
    input,
    s_vehicle,
    t_vehicle,
    normal_t_gain,
    reversed_t_gain,
    is_normal_valid,
    is_reverse_valid,
    tw_s,
    tw_t,
    [&](bool reverse_s, std::vector<Index>& ns, std::vector<Index>& nt) {
      build_or_opt_post_routes(s_route, s_rank, t_route, t_rank, reverse_s, ns, nt);
    });
}

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
  const TWRoute* tw_s,
  const TWRoute* tw_t) {
  return edge_swap_with_single_reverse_within_max_duration(
    input,
    s_vehicle,
    t_vehicle,
    normal_s_gain,
    reversed_s_gain,
    s_is_normal_valid,
    s_is_reverse_valid,
    tw_s,
    tw_t,
    [&](bool reverse_t, std::vector<Index>& ns, std::vector<Index>& nt) {
      build_mixed_exchange_post_routes(s_route,
                                       s_rank,
                                       t_route,
                                       t_rank,
                                       reverse_t,
                                       ns,
                                       nt);
    });
}

bool insertion_respects_vehicle_bounds(const Input& input,
                                       Index vehicle_rank,
                                       const Eval& route_eval,
                                       const Eval& insertion_eval,
                                       const std::vector<Index>& route,
                                       Index job_rank,
                                       Index rank,
                                       const TWRoute* tw_live) {
  const auto& vehicle = input.vehicles[vehicle_rank];
  const Eval combined = route_eval + insertion_eval;
  if (!vehicle.ok_for_travel_time(combined.duration) ||
      !vehicle.ok_for_distance(combined.distance)) {
    return false;
  }
  if (!input.has_bounded_max_duration()) {
    // Match v1.15: eval bounds only; capacity/TW checked separately.
    return vehicle.ok_for_range_bounds(combined);
  }
  if (vehicle.max_duration == DEFAULT_MAX_DURATION) {
    std::vector<Index> jobs = route;
    jobs.insert(jobs.begin() + static_cast<std::ptrdiff_t>(rank), job_rank);
    return RawRoute::jobs_within_capacity(input, vehicle_rank, jobs);
  }
  if (combined.duration + combined.task_duration > vehicle.max_duration) {
    return false;
  }

  std::vector<Index> jobs = route;
  jobs.insert(jobs.begin() + static_cast<std::ptrdiff_t>(rank), job_rank);
  if (tw_live != nullptr) {
    return route_jobs_within_max_duration_for_ls(input,
                                                 vehicle_rank,
                                                 jobs,
                                                 tw_live);
  }
  return route_jobs_within_max_duration(input, vehicle_rank, jobs);
}

namespace {

Duration job_action_duration_at(const Input& input,
                                const Vehicle& v,
                                const std::vector<Index>& route,
                                Index job_rank) {
  const auto& j = input.jobs[route[job_rank]];
  if (job_rank == 0) {
    const bool same_loc =
      v.has_start() && v.start.value().index() == j.index();
    return same_loc ? j.services[v.type]
                    : j.setups[v.type] + j.services[v.type];
  }
  const bool same_loc =
    input.jobs[route[job_rank - 1]].index() == j.index();
  return same_loc ? j.services[v.type]
                  : j.setups[v.type] + j.services[v.type];
}

bool tw_route_can_append_single_for_wait_eval(const Input& input,
                                              TWRoute& tw,
                                              Index job_rank) {
  const auto& job = input.jobs[job_rank];
  if (job.type != JOB_TYPE::SINGLE) {
    return false;
  }
  const Index rank = static_cast<Index>(tw.route.size());
  return job.pickup <= tw.pickup_margin() && job.delivery <= tw.delivery_margin() &&
         tw.is_valid_addition_for_capacity(input,
                                           job.pickup,
                                           job.delivery,
                                           rank) &&
         tw.is_valid_addition_for_tw(input, job_rank, rank);
}

bool tw_route_append_job_for_billable_wait_eval(const Input& input,
                                                TWRoute& tw,
                                                Index job_rank) {
  const Index rank = static_cast<Index>(tw.route.size());
  const auto& job = input.jobs[job_rank];
  if (job.type == JOB_TYPE::SINGLE) {
    if (!tw_route_can_append_single_for_wait_eval(input, tw, job_rank)) {
      return false;
    }
    tw.add(input, job_rank, rank);
    return true;
  }

  const std::array<Index, 1> a({job_rank});
  if (!tw.is_valid_addition_for_tw(input,
                                   job.delivery,
                                   a.begin(),
                                   a.end(),
                                   rank,
                                   rank)) {
    return false;
  }
  tw.replace(input, job.delivery, a.begin(), a.end(), rank, rank);
  return true;
}

} // namespace

std::optional<Duration>
billable_wait_for_job_sequence_via_empty_replace(const Input& input,
                                                 Index vehicle_rank,
                                                 const std::vector<Index>& jobs) {
  if (jobs.empty()) {
    return Duration{0};
  }

  TWRoute tw(input, vehicle_rank, input.get_amount_size());
  Amount delivery = input.zero_amount();
  for (const Index jr : jobs) {
    delivery += input.jobs[jr].delivery;
  }
  if (!tw.is_valid_addition_for_tw(input,
                                   delivery,
                                   jobs.begin(),
                                   jobs.end(),
                                   0,
                                   0)) {
    return std::nullopt;
  }
  tw.replace(input, delivery, jobs.begin(), jobs.end(), 0, 0);
  return tw.billable_total_wait;
}

// Billable wait for a job sequence: three paths (sequential add, empty-route
// batch replace when breaks + multi-job, partial edit via apply_jobs_to_tw_route)
// reflect different break scheduling semantics — keep all three.
std::optional<Duration> billable_wait_for_job_sequence_aligned_with_route_eval(
  const Input& input,
  Index vehicle_rank,
  const std::vector<Index>& jobs) {
  if (jobs.empty()) {
    return Duration{0};
  }

  const auto& vehicle = input.vehicles[vehicle_rank];
  // Multi-job batch replace on an empty route (RouteSplit, custom routes)
  // can schedule mandatory breaks differently than sequential add().
  if (!vehicle.breaks.empty() && jobs.size() > 1) {
    return billable_wait_for_job_sequence_via_empty_replace(input,
                                                            vehicle_rank,
                                                            jobs);
  }

  TWRoute tw(input, vehicle_rank, input.get_amount_size());
  for (const Index jr : jobs) {
    if (!tw_route_append_job_for_billable_wait_eval(input, tw, jr)) {
      return std::nullopt;
    }
  }
  tw.refresh_billable_total_wait_for_eval(input);
  return tw.billable_total_wait;
}

std::optional<Duration> approx_billable_wait_jobs_only(
  const Input& input,
  Index vehicle_rank,
  const std::vector<Index>& route,
  Duration fixed_departure) {
  const auto& v = input.vehicles[vehicle_rank];
  if (!v.breaks.empty()) {
    return std::nullopt;
  }
  if (route.empty()) {
    return Duration{0};
  }

  const Duration e0 = v.earliest_route_start();
  fixed_departure = std::max(fixed_departure, e0);

  Duration total = fixed_departure - e0;
  Duration current = fixed_departure;

  for (Index i = 0; i < static_cast<Index>(route.size()); ++i) {
    const auto& next_j = input.jobs[route[i]];

    const Duration travel_time =
      (i == 0) ? (v.has_start() ? v.duration(v.start.value().index(),
                                              next_j.index())
                                : Duration{0})
               : v.duration(input.jobs[route[i - 1]].index(), next_j.index());

    const Duration previous_action_time =
      (i == 0) ? Duration{0}
               : job_action_duration_at(input, v, route, i - 1);

    current += previous_action_time + travel_time;
    const auto j_tw = std::ranges::find_if(next_j.tws, [&](const auto& tw) {
      return current <= tw.end;
    });
    if (j_tw == next_j.tws.end()) {
      return std::nullopt;
    }

    total += std::max(static_cast<Duration>(0), j_tw->start - current);
    current = std::max(current, j_tw->start);
  }

  return total;
}

bool route_jobs_within_max_duration_for_ls(const Input& input,
                                           Index vehicle_rank,
                                           const std::vector<Index>& jobs,
                                           const TWRoute* tw_live) {
  const auto& vehicle = input.vehicles[vehicle_rank];
  if (vehicle.max_duration == DEFAULT_MAX_DURATION) {
    return true;
  }

  if (!jobs.empty()) {
    if (!RawRoute::jobs_within_capacity(input, vehicle_rank, jobs)) {
      return false;
    }
    if (!route_jobs_pass_range_pre_filter(input, vehicle_rank, jobs)) {
      return false;
    }
  }

  if (tw_live != nullptr && tw_live->v_rank == vehicle_rank) {
    if (jobs == tw_live->route) {
      return true;
    }
    auto& scratch = ls_tw_eval_scratch_route(input, vehicle_rank);
    scratch = *tw_live;
    return tw_route_rebuild_and_within_max_duration(input, scratch, jobs);
  }

  return route_jobs_within_max_duration(input, vehicle_rank, jobs);
}

std::optional<Cost> wait_cost_approx_job_sequence(
  const Input& input,
  Index vehicle_rank,
  const std::vector<Index>& jobs) {
  const auto& veh = input.vehicles[vehicle_rank];
  if (veh.costs.per_wait_hour == 0) {
    return Cost{0};
  }
  if (!veh.breaks.empty()) {
    return wait_cost_for_job_sequence(input, vehicle_rank, jobs, nullptr);
  }
  const auto w = approx_billable_wait_jobs_only(input,
                                                vehicle_rank,
                                                jobs,
                                                veh.earliest_route_start());
  if (!w.has_value()) {
    return std::nullopt;
  }
  return veh.wait_cost(*w);
}

std::optional<Cost> wait_gain_upper_bound_from_routes(
  const Input& input,
  Index v1,
  const std::vector<Index>& r1,
  const TWRoute* tw_r1,
  Index v2,
  const std::vector<Index>& r2,
  const TWRoute* tw_r2) {
  const auto& veh1 = input.vehicles[v1];
  const auto& veh2 = input.vehicles[v2];
  if (veh1.costs.per_wait_hour == 0 && veh2.costs.per_wait_hour == 0) {
    return Cost{0};
  }

  Cost total{0};
  if (veh1.costs.per_wait_hour != 0) {
    const auto wc1 = wait_cost_for_job_sequence(input, v1, r1, tw_r1);
    if (!wc1.has_value()) {
      return std::nullopt;
    }
    total += *wc1;
  }
  if (v2 != v1 || r2 != r1) {
    if (veh2.costs.per_wait_hour != 0) {
      const auto wc2 = wait_cost_for_job_sequence(input, v2, r2, tw_r2);
      if (!wc2.has_value()) {
        return std::nullopt;
      }
      total += *wc2;
    }
  }
  return total;
}

bool skip_ls_wait_pruning(const Eval& stored_gain,
                         const std::optional<Cost>& wait_ub,
                         const Eval& best_known) {
  if (best_known == NO_EVAL || !wait_ub.has_value()) {
    return false;
  }
  return stored_gain.cost + *wait_ub <= best_known.cost;
}

std::optional<Cost> wait_cost_for_job_sequence(const Input& input,
                                               Index vehicle_rank,
                                               const std::vector<Index>& jobs,
                                               const TWRoute* tw_if_matches) {
  return wait_cost_for_billable_sequence(input, vehicle_rank, jobs, tw_if_matches);
}

Cost wait_insertion_marginal_cost(const Input& input,
                                  const TWRoute& route,
                                  const std::vector<Index>& route_with_insertion) {
  const auto v = route.v_rank;
  const auto& veh = input.vehicles[v];
  if (veh.costs.per_wait_hour == 0) {
    return Cost{0};
  }
  const auto old_wc =
    wait_cost_for_job_sequence(input, v, route.route, &route);
  if (!old_wc.has_value()) {
    return Cost{0};
  }
  const auto new_wc = wait_cost_for_job_sequence_after_edit(input,
                                                            v,
                                                            route_with_insertion,
                                                            route);
  if (!new_wc.has_value()) {
    return Cost{0};
  }
  return *old_wc - *new_wc;
}

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
                               Eval best_known) {
  std::vector<Index> ns;
  std::vector<Index> nt;
  build_relocate_post_routes(s_route, s_rank, t_route, t_rank, ns, nt);
  adjust_stored_gain_for_wait_approx_two_routes(input,
                                                stored_gain,
                                                s_vehicle,
                                                s_route,
                                                ns,
                                                t_vehicle,
                                                t_route,
                                                nt,
                                                tw_s_route,
                                                tw_t_route,
                                                best_known);
}

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
                              Eval best_known) {
  std::vector<Index> ns;
  std::vector<Index> nt;
  build_two_opt_post_routes(s_route, s_rank, t_route, t_rank, ns, nt);
  adjust_stored_gain_for_wait_approx_two_routes(input,
                                                stored_gain,
                                                s_vehicle,
                                                s_route,
                                                ns,
                                                t_vehicle,
                                                t_route,
                                                nt,
                                                tw_s_route,
                                                tw_t_route,
                                                best_known);
}

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
                                      Eval best_known) {
  std::vector<Index> ns;
  std::vector<Index> nt;
  build_reverse_two_opt_post_routes(s_route, s_rank, t_route, t_rank, ns, nt);
  adjust_stored_gain_for_wait_approx_two_routes(input,
                                                stored_gain,
                                                s_vehicle,
                                                s_route,
                                                ns,
                                                t_vehicle,
                                                t_route,
                                                nt,
                                                tw_s_route,
                                                tw_t_route,
                                                best_known);
}

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
                             Eval best_known) {
  std::vector<Index> ns;
  std::vector<Index> nt;
  build_or_opt_post_routes(s_route,
                           s_rank,
                           t_route,
                           t_rank,
                           reverse_s_edge,
                           ns,
                           nt);
  adjust_stored_gain_for_wait_approx_two_routes(input,
                                                stored_gain,
                                                s_vehicle,
                                                s_route,
                                                ns,
                                                t_vehicle,
                                                t_route,
                                                nt,
                                                tw_s_route,
                                                tw_t_route,
                                                best_known);
}

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
                                     Eval best_known) {
  std::vector<Index> ns;
  std::vector<Index> nt;
  build_cross_exchange_post_routes(s_route,
                                   s_rank,
                                   t_route,
                                   t_rank,
                                   reverse_s_edge,
                                   reverse_t_edge,
                                   ns,
                                   nt);
  adjust_stored_gain_for_wait_approx_two_routes(input,
                                                stored_gain,
                                                s_vehicle,
                                                s_route,
                                                ns,
                                                t_vehicle,
                                                t_route,
                                                nt,
                                                tw_s_route,
                                                tw_t_route,
                                                best_known);
}

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
                                     Eval best_known) {
  std::vector<Index> ns;
  std::vector<Index> nt;
  build_mixed_exchange_post_routes(s_route,
                                   s_rank,
                                   t_route,
                                   t_rank,
                                   reverse_t_edge,
                                   ns,
                                   nt);
  adjust_stored_gain_for_wait_approx_two_routes(input,
                                                stored_gain,
                                                s_vehicle,
                                                s_route,
                                                ns,
                                                t_vehicle,
                                                t_route,
                                                nt,
                                                tw_s_route,
                                                tw_t_route,
                                                best_known);
}

void adjust_route_exchange_wait_gain(const Input& input,
                                     Eval& stored_gain,
                                     Index s_vehicle,
                                     const std::vector<Index>& s_route,
                                     Index t_vehicle,
                                     const std::vector<Index>& t_route,
                                     const TWRoute* tw_s_route,
                                     const TWRoute* tw_t_route,
                                     Eval best_known) {
  adjust_stored_gain_for_wait_approx_two_routes(input,
                                                stored_gain,
                                                s_vehicle,
                                                s_route,
                                                t_route,
                                                t_vehicle,
                                                t_route,
                                                s_route,
                                                tw_s_route,
                                                tw_t_route,
                                                best_known);
}

void adjust_one_route_moved_jobs_wait_gain(
  const Input& input,
  Eval& stored_gain,
  Index v,
  const std::vector<Index>& route_old,
  Index first_rank,
  const std::vector<Index>& moved_jobs,
  const TWRoute* tw_route,
  Eval best_known) {
  std::vector<Index> route_new;
  build_one_route_after_moved_jobs(route_old, first_rank, moved_jobs, route_new);
  adjust_stored_gain_for_wait_approx_one_route(input,
                                               stored_gain,
                                               v,
                                               route_old,
                                               route_new,
                                               tw_route,
                                               best_known);
}

void adjust_stored_gain_for_wait_approx_two_routes(
  const Input& input,
  Eval& stored_gain,
  Index v1,
  const std::vector<Index>& r1_old,
  const std::vector<Index>& r1_new,
  Index v2,
  const std::vector<Index>& r2_old,
  const std::vector<Index>& r2_new,
  const TWRoute* tw_r1_old,
  const TWRoute* tw_r2_old,
  Eval best_known) {
  const auto& veh1 = input.vehicles[v1];
  const auto& veh2 = input.vehicles[v2];
  if (veh1.costs.per_wait_hour == 0 && veh2.costs.per_wait_hour == 0) {
    return;
  }
  if (r1_old == r1_new && r2_old == r2_new) {
    return;
  }

  const auto wait_ub = wait_gain_upper_bound_from_routes(input,
                                                         v1,
                                                         r1_old,
                                                         tw_r1_old,
                                                         v2,
                                                         r2_old,
                                                         tw_r2_old);
  if (skip_ls_wait_pruning(stored_gain, wait_ub, best_known)) {
    return;
  }

  const auto old_wc1 =
    wait_cost_for_job_sequence(input, v1, r1_old, tw_r1_old);
  const auto old_wc2 =
    wait_cost_for_job_sequence(input, v2, r2_old, tw_r2_old);
  if (!old_wc1.has_value() || !old_wc2.has_value()) {
    return;
  }

  const auto new_wc1 =
    wait_cost_approx_job_sequence(input, v1, r1_new);
  const auto new_wc2 =
    wait_cost_approx_job_sequence(input, v2, r2_new);
  if (new_wc1.has_value() && new_wc2.has_value()) {
    stored_gain.cost += *old_wc1 + *old_wc2 - *new_wc1 - *new_wc2;
    return;
  }

  std::optional<Cost> new_wc1_exact;
  std::optional<Cost> new_wc2_exact;
  if (tw_r1_old != nullptr) {
    new_wc1_exact =
      wait_cost_for_job_sequence_after_edit(input, v1, r1_new, *tw_r1_old);
  } else {
    new_wc1_exact = wait_cost_for_job_sequence(input, v1, r1_new, nullptr);
  }
  if (tw_r2_old != nullptr) {
    new_wc2_exact =
      wait_cost_for_job_sequence_after_edit(input, v2, r2_new, *tw_r2_old);
  } else {
    new_wc2_exact = wait_cost_for_job_sequence(input, v2, r2_new, nullptr);
  }
  if (!new_wc1_exact.has_value() || !new_wc2_exact.has_value()) {
    return;
  }
  stored_gain.cost += *old_wc1 + *old_wc2 - *new_wc1_exact - *new_wc2_exact;
}

void adjust_stored_gain_for_wait_approx_one_route(const Input& input,
                                                  Eval& stored_gain,
                                                  Index v,
                                                  const std::vector<Index>& r_old,
                                                  const std::vector<Index>& r_new,
                                                  const TWRoute* tw_r_old,
                                                  Eval best_known) {
  const auto& veh = input.vehicles[v];
  if (veh.costs.per_wait_hour == 0) {
    return;
  }
  if (r_old == r_new) {
    return;
  }

  const auto wait_ub =
    wait_gain_upper_bound_from_route(input, v, r_old, tw_r_old);
  if (skip_ls_wait_pruning(stored_gain, wait_ub, best_known)) {
    return;
  }

  const auto old_wc =
    wait_cost_for_job_sequence(input, v, r_old, tw_r_old);
  if (!old_wc.has_value()) {
    return;
  }
  const auto new_wc = wait_cost_approx_job_sequence(input, v, r_new);
  if (new_wc.has_value()) {
    stored_gain.cost += *old_wc - *new_wc;
    return;
  }
  std::optional<Cost> new_wc_exact;
  if (tw_r_old != nullptr) {
    new_wc_exact =
      wait_cost_for_job_sequence_after_edit(input, v, r_new, *tw_r_old);
  } else {
    new_wc_exact = wait_cost_for_job_sequence(input, v, r_new, nullptr);
  }
  if (!new_wc_exact.has_value()) {
    return;
  }
  stored_gain.cost += *old_wc - *new_wc_exact;
}

#ifndef NDEBUG
void check_precedence(const Input& input,
                      std::unordered_set<Index>& expected_delivery_ranks,
                      Index job_rank) {
  switch (input.jobs[job_rank].type) {
    using enum JOB_TYPE;
  case SINGLE:
    break;
  case PICKUP:
    expected_delivery_ranks.insert(job_rank + 1);
    break;
  case DELIVERY:
    // Associated pickup has been done before.
    auto search = expected_delivery_ranks.find(job_rank);
    assert(search != expected_delivery_ranks.end());
    expected_delivery_ranks.erase(search);
    break;
  }
}
#endif

void check_tws(const std::vector<TimeWindow>& tws,
               const Id id,
               const std::string& type) {
  if (tws.empty()) {
    throw InputException(
      std::format("Empty time windows for {} {}.", type, id));
  }

  if (tws.size() > 1) {
    for (std::size_t i = 0; i < tws.size() - 1; ++i) {
      if (tws[i + 1].start <= tws[i].end) {
        throw InputException(
          std::format("Unsorted or overlapping time-windows for {} {}.",
                      type,
                      id));
      }
    }
  }
}

void check_priority(const Priority priority,
                    const Id id,
                    const std::string& type) {
  if (priority > MAX_PRIORITY) {
    throw InputException(
      std::format("Invalid priority value for {} {}.", type, id));
  }
}

void check_no_empty_keys(const TypeToDurationMap& type_to_duration,
                         const Id id,
                         const std::string& type,
                         const std::string& key_name) {
  if (std::ranges::any_of(type_to_duration, [](const auto& pair) {
        return pair.first.empty();
      })) {
    throw InputException(
      std::format("Empty type in {} for {} {}.", key_name, type, id));
  }
}

inline std::vector<Job> get_unassigned_jobs_from_ranks(
  const Input& input,
  const std::unordered_set<Index>& unassigned_ranks) {
  std::vector<Job> unassigned_jobs;
  std::ranges::transform(unassigned_ranks,
                         std::back_inserter(unassigned_jobs),
                         [&](auto j) { return input.jobs[j]; });

  return unassigned_jobs;
}

Solution format_solution(const Input& input, const RawSolution& raw_routes) {
  std::vector<Route> routes;
  routes.reserve(raw_routes.size());

  // All job ranks start with unassigned status.
  std::unordered_set<Index> unassigned_ranks;
  for (unsigned i = 0; i < input.jobs.size(); ++i) {
    unassigned_ranks.insert(i);
  }

  for (std::size_t i = 0; i < raw_routes.size(); ++i) {
    const auto& route = raw_routes[i].route;
    if (route.empty()) {
      continue;
    }
    const auto& v = input.vehicles[i];

    assert(route.size() <= v.max_tasks);

    auto previous_location = (v.has_start())
                               ? v.start.value().index()
                               : std::numeric_limits<Index>::max();
    Eval eval_sum;
    Duration setup = 0;
    Duration service = 0;
    Priority priority = 0;
    Amount sum_pickups(input.zero_amount());
    Amount sum_deliveries(input.zero_amount());
#ifndef NDEBUG
    std::unordered_set<Index> expected_delivery_ranks;
#endif
    Amount current_load = raw_routes[i].job_deliveries_sum();
    assert(current_load <= v.capacity);

    // Steps for current route.
    std::vector<Step> steps;
    steps.reserve(route.size() + 2);

    Duration ETA = 0;
    const auto& first_job = input.jobs[route.front()];

    // Handle start.
    const auto start_loc = v.has_start() ? v.start.value() : first_job.location;
    steps.emplace_back(STEP_TYPE::START, start_loc, current_load);
    if (v.has_start()) {
      const auto next_leg = v.eval(v.start.value().index(), first_job.index());
      ETA += next_leg.duration;
      eval_sum += next_leg;
    }

    // Handle jobs.
    assert(input.vehicle_ok_with_job(i, route.front()));

    const auto first_job_setup =
      (first_job.index() == previous_location) ? 0 : first_job.setups[v.type];
    setup += first_job_setup;
    previous_location = first_job.index();

    const auto first_job_service = first_job.services[v.type];
    service += first_job_service;
    priority += first_job.priority;

    current_load += first_job.pickup;
    current_load -= first_job.delivery;
    sum_pickups += first_job.pickup;
    sum_deliveries += first_job.delivery;
    assert(current_load <= v.capacity);

#ifndef NDEBUG
    check_precedence(input, expected_delivery_ranks, route.front());
#endif

    steps.emplace_back(first_job,
                       scale_to_user_duration(first_job_setup),
                       scale_to_user_duration(first_job_service),
                       current_load);
    auto& first = steps.back();
    first.duration = scale_to_user_duration(ETA);
    first.distance = eval_sum.distance;
    first.arrival = scale_to_user_duration(ETA);
    ETA += (first_job_setup + first_job_service);
    unassigned_ranks.erase(route.front());

    for (std::size_t r = 0; r < route.size() - 1; ++r) {
      assert(input.vehicle_ok_with_job(i, route[r + 1]));
      const auto next_leg =
        v.eval(input.jobs[route[r]].index(), input.jobs[route[r + 1]].index());
      ETA += next_leg.duration;
      eval_sum += next_leg;

      const auto& current_job = input.jobs[route[r + 1]];

      const auto current_setup = (current_job.index() == previous_location)
                                   ? 0
                                   : current_job.setups[v.type];
      setup += current_setup;
      previous_location = current_job.index();

      const auto current_service = current_job.services[v.type];
      service += current_service;
      priority += current_job.priority;

      current_load += current_job.pickup;
      current_load -= current_job.delivery;
      sum_pickups += current_job.pickup;
      sum_deliveries += current_job.delivery;
      assert(current_load <= v.capacity);

#ifndef NDEBUG
      check_precedence(input, expected_delivery_ranks, route[r + 1]);
#endif

      steps.emplace_back(current_job,
                         scale_to_user_duration(current_setup),
                         scale_to_user_duration(current_service),
                         current_load);
      auto& current = steps.back();
      current.duration = scale_to_user_duration(eval_sum.duration);
      current.distance = eval_sum.distance;
      current.arrival = scale_to_user_duration(ETA);
      ETA += (current_setup + current_service);
      unassigned_ranks.erase(route[r + 1]);
    }

    // Handle end.
    const auto& last_job = input.jobs[route.back()];
    const auto end_loc = v.has_end() ? v.end.value() : last_job.location;
    steps.emplace_back(STEP_TYPE::END, end_loc, current_load);
    if (v.has_end()) {
      const auto next_leg = v.eval(last_job.index(), v.end.value().index());
      ETA += next_leg.duration;
      eval_sum += next_leg;
    }
    auto& last = steps.back();
    last.duration = scale_to_user_duration(eval_sum.duration);
    last.distance = eval_sum.distance;
    last.arrival = scale_to_user_duration(ETA);

    assert(expected_delivery_ranks.empty());
    assert(v.ok_for_range_bounds(Eval(0,
                                      eval_sum.duration,
                                      eval_sum.distance,
                                      setup + service,
                                      0)));

    assert(v.fixed_cost() % (DURATION_FACTOR * COST_FACTOR) == 0);
    const UserCost user_fixed_cost = scale_to_user_cost(v.fixed_cost());
    const UserCost user_travel_cost = scale_to_user_cost(eval_sum.cost);
    const UserCost user_task_cost =
      scale_to_user_cost(v.task_cost(setup + service));

    routes.emplace_back(v.id,
                        std::move(steps),
                        user_fixed_cost + user_travel_cost + user_task_cost,
                        scale_to_user_duration(eval_sum.duration),
                        eval_sum.distance,
                        scale_to_user_duration(setup),
                        scale_to_user_duration(service),
                        0,
                        priority,
                        sum_deliveries,
                        sum_pickups,
                        v.profile,
                        v.description);
  }

  return Solution(input.zero_amount(),
                  std::move(routes),
                  get_unassigned_jobs_from_ranks(input, unassigned_ranks));
}

namespace {

struct BackwardEtaAnchor {
  Duration depot_departure;
  Duration backward_wt;
  // Depot leave time before optional latest-departure cap; used for
  // debug checks when the cap shifts waiting from depot to in-route.
  Duration ideal_departure;
  std::optional<Location> first_location;
  std::optional<Location> last_location;
};

BackwardEtaAnchor compute_backward_eta_anchor(const Input& input,
                                              const TWRoute& tw_r) {
  const auto& v = input.vehicles[tw_r.v_rank];

  // ETA logic: aim at earliest possible arrival then determine latest
  // possible start time in order to minimize waiting times.
  Duration step_start = tw_r.earliest_end;
  Duration backward_wt = 0;
  std::optional<Location> first_location;
  std::optional<Location> last_location;

  if (v.has_end()) {
    first_location = v.end.value();
    last_location = v.end.value();
  }

  for (std::size_t r = tw_r.route.size(); r > 0; --r) {
    const auto& previous_job = input.jobs[tw_r.route[r - 1]];

    if (!last_location.has_value()) {
      last_location = previous_job.location;
    }
    first_location = previous_job.location;

    // Remaining travel time is the time between two jobs, except for
    // last rank where it depends whether the vehicle has an end or
    // not.
    Duration remaining_travel_time;
    if (r < tw_r.route.size()) {
      remaining_travel_time =
        v.duration(previous_job.index(), input.jobs[tw_r.route[r]].index());
    } else {
      remaining_travel_time =
        (v.has_end()) ? v.duration(previous_job.index(), v.end.value().index())
                      : 0;
    }

    // Take into account timing constraints for breaks before current
    // job.
    assert(tw_r.breaks_at_rank[r] <= tw_r.breaks_counts[r]);
    Index break_rank = tw_r.breaks_counts[r];
    for (Index i = 0; i < tw_r.breaks_at_rank[r]; ++i) {
      --break_rank;
      const auto& b = v.breaks[break_rank];
      assert(b.service <= step_start);
      step_start -= b.service;

      const auto b_tw =
        std::find_if(b.tws.rbegin(), b.tws.rend(), [&](const auto& tw) {
          return tw.start <= step_start;
        });
      assert(b_tw != b.tws.rend());

      if (b_tw->end < step_start) {
        if (const auto margin = step_start - b_tw->end;
            margin < remaining_travel_time) {
          remaining_travel_time -= margin;
        } else {
          backward_wt += (margin - remaining_travel_time);
          remaining_travel_time = 0;
        }

        step_start = b_tw->end;
      }
    }

    const bool same_location =
      (r > 1 &&
       input.jobs[tw_r.route[r - 2]].index() == previous_job.index()) ||
      (r == 1 && v.has_start() &&
       v.start.value().index() == previous_job.index());
    const auto current_setup = same_location ? 0 : previous_job.setups[v.type];

    const Duration diff =
      current_setup + previous_job.services[v.type] + remaining_travel_time;

    assert(diff <= step_start);
    Duration candidate_start = step_start - diff;
    assert(tw_r.earliest[r - 1] <= candidate_start);

    const auto j_tw =
      std::find_if(previous_job.tws.rbegin(),
                   previous_job.tws.rend(),
                   [&](const auto& tw) { return tw.start <= candidate_start; });
    assert(j_tw != previous_job.tws.rend());

    step_start = std::min(candidate_start, j_tw->end);
    if (step_start < candidate_start) {
      backward_wt += (candidate_start - step_start);
    }
    assert(previous_job.is_valid_start(step_start));
  }

  // Now pack everything ASAP based on first job start date.
  Duration remaining_travel_time =
    (v.has_start())
      ? v.duration(v.start.value().index(), input.jobs[tw_r.route[0]].index())
      : 0;

  // Take into account timing constraints for breaks before first job.
  assert(tw_r.breaks_at_rank[0] <= tw_r.breaks_counts[0]);
  Index break_rank = tw_r.breaks_counts[0];
  for (Index r = 0; r < tw_r.breaks_at_rank[0]; ++r) {
    --break_rank;
    const auto& b = v.breaks[break_rank];
    assert(b.service <= step_start);
    step_start -= b.service;

    const auto b_tw =
      std::find_if(b.tws.rbegin(), b.tws.rend(), [&](const auto& tw) {
        return tw.start <= step_start;
      });
    assert(b_tw != b.tws.rend());

    if (b_tw->end < step_start) {
      if (const auto margin = step_start - b_tw->end;
          margin < remaining_travel_time) {
        remaining_travel_time -= margin;
      } else {
        backward_wt += (margin - remaining_travel_time);
        remaining_travel_time = 0;
      }

      step_start = b_tw->end;
    }
  }

  if (v.has_start()) {
    first_location = v.start.value();
    assert(remaining_travel_time <= step_start);
    step_start -= remaining_travel_time;
  }

  const Duration ideal_departure = step_start;

  // Latest departure from depot (optional): cannot leave after this time.
  if (v.has_latest_departure()) {
    assert(v.departure.has_value());
    step_start = std::min(step_start, v.departure.value());
  }
  assert(step_start >= v.tw.start);
  assert(first_location.has_value() && last_location.has_value());

  return {step_start,
          backward_wt,
          ideal_departure,
          std::move(first_location),
          std::move(last_location)};
}

} // namespace

Duration min_wait_route_departure(const Input& input, const TWRoute& tw_r) {
  return compute_backward_eta_anchor(input, tw_r).depot_departure;
}

Route format_route(const Input& input,
                   const TWRoute& tw_r,
                   std::unordered_set<Index>& unassigned_ranks) {
  const auto& v = input.vehicles[tw_r.v_rank];

  assert(tw_r.size() <= v.max_tasks);

  auto anchor = compute_backward_eta_anchor(input, tw_r);
  Duration step_start = anchor.depot_departure;
  Duration backward_wt = anchor.backward_wt;
  const Duration ideal_departure = anchor.ideal_departure;
  auto first_location = std::move(anchor.first_location);
  auto last_location = std::move(anchor.last_location);

  assert(first_location.has_value() && last_location.has_value());

#ifndef NDEBUG
  std::unordered_set<Index> expected_delivery_ranks;
#endif
  Amount current_load = tw_r.job_deliveries_sum();
  assert(current_load <= v.capacity);

  // Steps for current route.
  std::vector<Step> steps;
  steps.reserve(tw_r.size() + 2 + v.breaks.size());

  steps.emplace_back(STEP_TYPE::START, first_location.value(), current_load);
  assert(v.tw.contains(step_start));
  // Start step: arrival = earliest depot release; waiting_time = idle until
  // actual leave (aligned with per_wait_hour / route.waiting_time).
  const Duration e0 = v.earliest_route_start();
  assert(step_start >= e0);
  const UserDuration user_depot_leave = scale_to_user_duration(step_start);
  steps.back().arrival = scale_to_user_duration(e0);
  steps.back().waiting_time = user_depot_leave - steps.back().arrival;
  UserDuration user_previous_end = user_depot_leave;

#ifndef NDEBUG
  const auto front_step_arrival = step_start;
#endif

  auto previous_location = (v.has_start()) ? v.start.value().index()
                                           : std::numeric_limits<Index>::max();

  // Values summed up while going through the route.
  Eval eval_sum;
  Duration duration = 0;
  UserDuration user_duration = 0;
  UserDuration user_waiting_time = steps.back().waiting_time;
  Duration setup = 0;
  Duration service = 0;
  Duration jobs_service = 0;
  Duration forward_wt = 0;
  Priority priority = 0;
  Amount sum_pickups(input.zero_amount());
  Amount sum_deliveries(input.zero_amount());

  // Go through the whole route again to set jobs/breaks ASAP given
  // the latest possible start time.
  Eval current_eval = v.has_start() ? v.eval(v.start.value().index(),
                                             input.jobs[tw_r.route[0]].index())
                                    : Eval();

  Duration travel_time = current_eval.duration;

  for (std::size_t r = 0; r < tw_r.route.size(); ++r) {
    assert(input.vehicle_ok_with_job(tw_r.v_rank, tw_r.route[r]));
    const auto& current_job = input.jobs[tw_r.route[r]];
    auto user_distance = eval_sum.distance;

    if (r > 0) {
      // For r == 0, travel_time already holds the relevant value
      // depending on whether there is a start.
      current_eval =
        v.eval(input.jobs[tw_r.route[r - 1]].index(), current_job.index());
      travel_time = current_eval.duration;
    }

    // Handles breaks before this job.
    assert(tw_r.breaks_at_rank[r] <= tw_r.breaks_counts[r]);
    Index break_rank = tw_r.breaks_counts[r] - tw_r.breaks_at_rank[r];

    for (Index i = 0; i < tw_r.breaks_at_rank[r]; ++i, ++break_rank) {
      const auto& b = v.breaks[break_rank];

      assert(b.is_valid_for_load(current_load));

      steps.emplace_back(b, current_load);
      auto& current_break = steps.back();

      const auto b_tw = std::ranges::find_if(b.tws, [&](const auto& tw) {
        return step_start <= tw.end;
      });
      assert(b_tw != b.tws.end());

      if (step_start < b_tw->start) {
        if (const auto margin = b_tw->start - step_start;
            margin <= travel_time) {
          // Part of the remaining travel time is spent before this
          // break, filling the whole margin.
          duration += margin;
          travel_time -= margin;
          current_break.arrival = scale_to_user_duration(b_tw->start);
        } else {
          // The whole remaining travel time is spent before this
          // break, not filling the whole margin.

          const Duration wt = margin - travel_time;
          forward_wt += wt;

          current_break.arrival =
            scale_to_user_duration(step_start + travel_time);

          // Recompute user-reported waiting time rather than using
          // scale_to_user_duration(wt) to avoid rounding problems.
          current_break.waiting_time =
            scale_to_user_duration(b_tw->start) - current_break.arrival;
          user_waiting_time += current_break.waiting_time;

          duration += travel_time;
          travel_time = 0;
        }

        step_start = b_tw->start;
      } else {
        current_break.arrival = scale_to_user_duration(step_start);
      }

      assert(b_tw->start % DURATION_FACTOR == 0 &&
             scale_to_user_duration(b_tw->start) <=
               current_break.arrival + current_break.waiting_time &&
             (current_break.waiting_time == 0 ||
              scale_to_user_duration(b_tw->start) ==
                current_break.arrival + current_break.waiting_time));

      // Recompute cumulated durations in a consistent way as seen
      // from UserDuration.
      assert(user_previous_end <= current_break.arrival);
      auto user_travel_time = current_break.arrival - user_previous_end;
      user_duration += user_travel_time;
      current_break.duration = user_duration;

      // Pro rata temporis distance increase.
      if (current_eval.duration != 0) {
        user_distance += round<UserDistance>(
          static_cast<double>(user_travel_time * current_eval.distance) /
          scale_to_user_duration(current_eval.duration));
      }
      current_break.distance = user_distance;

      user_previous_end = current_break.arrival + current_break.waiting_time +
                          current_break.service;

      service += b.service;
      step_start += b.service;
    }

    // Back to current job.
    duration += travel_time;
    eval_sum += current_eval;
    const auto current_service = current_job.services[v.type];
    service += current_service;
    jobs_service += current_service;
    priority += current_job.priority;

    const auto current_setup = (current_job.index() == previous_location)
                                 ? 0
                                 : current_job.setups[v.type];
    setup += current_setup;
    previous_location = current_job.index();

    current_load += current_job.pickup;
    current_load -= current_job.delivery;
    sum_pickups += current_job.pickup;
    sum_deliveries += current_job.delivery;
    assert(current_load <= v.capacity);

#ifndef NDEBUG
    check_precedence(input, expected_delivery_ranks, tw_r.route[r]);
#endif

    steps.emplace_back(current_job,
                       scale_to_user_duration(current_setup),
                       scale_to_user_duration(current_service),
                       current_load);
    auto& current = steps.back();

    step_start += travel_time;
    assert(step_start <= tw_r.latest[r]);

    current.arrival = scale_to_user_duration(step_start);
    current.distance = eval_sum.distance;

    const auto j_tw =
      std::ranges::find_if(current_job.tws, [&](const auto& tw) {
        return step_start <= tw.end;
      });
    assert(j_tw != current_job.tws.end());

    if (step_start < j_tw->start) {
      const Duration wt = j_tw->start - step_start;
      forward_wt += wt;

      // Recompute user-reported waiting time rather than using
      // scale_to_user_duration(wt) to avoid rounding problems.
      current.waiting_time =
        scale_to_user_duration(j_tw->start) - current.arrival;
      user_waiting_time += current.waiting_time;

      step_start = j_tw->start;
    }

    // Recompute cumulated durations in a consistent way as seen from
    // UserDuration.
    assert(user_previous_end <= current.arrival);
    auto user_travel_time = current.arrival - user_previous_end;
    user_duration += user_travel_time;
    current.duration = user_duration;
    user_previous_end =
      current.arrival + current.waiting_time + current.setup + current.service;

    assert(
      j_tw->start % DURATION_FACTOR == 0 &&
      scale_to_user_duration(j_tw->start) <=
        current.arrival + current.waiting_time &&
      (current.waiting_time == 0 || scale_to_user_duration(j_tw->start) ==
                                      current.arrival + current.waiting_time));

    step_start += (current_setup + current_service);

    unassigned_ranks.erase(tw_r.route[r]);
  }

  // Handle breaks after last job.
  current_eval = (v.has_end()) ? v.eval(input.jobs[tw_r.route.back()].index(),
                                        v.end.value().index())
                               : Eval();
  travel_time = current_eval.duration;
  auto user_distance = eval_sum.distance;

  auto r = tw_r.route.size();
  assert(tw_r.breaks_at_rank[r] <= tw_r.breaks_counts[r]);
  Index break_rank = tw_r.breaks_counts[r] - tw_r.breaks_at_rank[r];

  for (Index i = 0; i < tw_r.breaks_at_rank[r]; ++i, ++break_rank) {
    const auto& b = v.breaks[break_rank];

    assert(b.is_valid_for_load(current_load));

    steps.emplace_back(b, current_load);
    auto& current_break = steps.back();

    const auto b_tw = std::ranges::find_if(b.tws, [&](const auto& tw) {
      return step_start <= tw.end;
    });
    assert(b_tw != b.tws.end());

    if (step_start < b_tw->start) {
      if (const auto margin = b_tw->start - step_start; margin <= travel_time) {
        // Part of the remaining travel time is spent before this
        // break, filling the whole margin.
        duration += margin;
        travel_time -= margin;
        current_break.arrival = scale_to_user_duration(b_tw->start);
      } else {
        // The whole remaining travel time is spent before this
        // break, not filling the whole margin.

        const Duration wt = margin - travel_time;
        forward_wt += wt;

        current_break.arrival =
          scale_to_user_duration(step_start + travel_time);

        // Recompute user-reported waiting time rather than using
        // scale_to_user_duration(wt) to avoid rounding problems.
        current_break.waiting_time =
          scale_to_user_duration(b_tw->start) - current_break.arrival;
        user_waiting_time += current_break.waiting_time;

        duration += travel_time;
        travel_time = 0;
      }

      step_start = b_tw->start;
    } else {
      current_break.arrival = scale_to_user_duration(step_start);
    }

    assert(b_tw->start % DURATION_FACTOR == 0 &&
           scale_to_user_duration(b_tw->start) <=
             current_break.arrival + current_break.waiting_time &&
           (current_break.waiting_time == 0 ||
            scale_to_user_duration(b_tw->start) ==
              current_break.arrival + current_break.waiting_time));

    // Recompute cumulated durations in a consistent way as seen from
    // UserDuration.
    assert(user_previous_end <= current_break.arrival);
    auto user_travel_time = current_break.arrival - user_previous_end;
    user_duration += user_travel_time;
    current_break.duration = user_duration;

    // Pro rata temporis distance increase.
    if (current_eval.duration != 0) {
      user_distance += round<UserDistance>(
        static_cast<double>(user_travel_time * current_eval.distance) /
        scale_to_user_duration(current_eval.duration));
    }
    current_break.distance = user_distance;

    user_previous_end = current_break.arrival + current_break.waiting_time +
                        current_break.service;

    service += b.service;
    step_start += b.service;
  }

  steps.emplace_back(STEP_TYPE::END, last_location.value(), current_load);
  auto& end_step = steps.back();
  if (v.has_end()) {
    duration += travel_time;
    eval_sum += current_eval;
    step_start += travel_time;
  }
  assert(v.tw.contains(step_start));
  end_step.arrival = scale_to_user_duration(step_start);
  end_step.distance = eval_sum.distance;

  assert(v.tw.end % DURATION_FACTOR == 0 || v.tw.is_default());
  const auto user_v_tw_end = scale_to_user_duration(v.tw.end);
  if (user_v_tw_end < end_step.arrival) {
    end_step.violations.types.insert(VIOLATION::DELAY);
    end_step.violations.delay = end_step.arrival - user_v_tw_end;
  }

  // Recompute cumulated durations in a consistent way as seen from
  // UserDuration.
  assert(user_previous_end <= end_step.arrival);
  auto user_travel_time = end_step.arrival - user_previous_end;
  user_duration += user_travel_time;
  end_step.duration = user_duration;

  assert(step_start == tw_r.earliest_end);
  const bool latest_departure_forces_earlier_leave =
    v.has_latest_departure() && v.departure.has_value() &&
    v.departure.value() < ideal_departure;
  assert(latest_departure_forces_earlier_leave || forward_wt == backward_wt);

  assert(step_start ==
         front_step_arrival + duration + setup + service + forward_wt);

  assert(expected_delivery_ranks.empty());

  assert(eval_sum.duration == duration);
#ifndef NDEBUG
  // Billable wait must match the format_route timeline (depot idle + in-route).
  if (input.has_nonzero_per_wait_hour()) {
    const Duration timeline_billable =
      (front_step_arrival - v.earliest_route_start()) + forward_wt;
    assert(timeline_billable == tw_r.billable_total_wait);
  }
#endif
  const Duration billable_wait_for_bounds = input.has_nonzero_per_wait_hour()
                                              ? tw_r.billable_total_wait
                                              : 0;
  // max_duration: travel + job setup/service + billable wait (break service is
  // excluded from work time).
  assert(v.ok_for_range_bounds(Eval(0,
                                    eval_sum.duration,
                                    eval_sum.distance,
                                    setup + jobs_service,
                                    billable_wait_for_bounds)));

  assert(v.fixed_cost() % (DURATION_FACTOR * COST_FACTOR) == 0);
  const UserCost user_fixed_cost = utils::scale_to_user_cost(v.fixed_cost());
  const UserCost user_travel_cost =
    v.cost_based_on_metrics()
      ? v.cost_wrapper.user_cost_from_user_metrics(user_duration,
                                                   eval_sum.distance)
      : utils::scale_to_user_cost(eval_sum.cost);
  const UserCost user_task_cost =
    scale_to_user_cost(v.task_cost(setup + service));
  const UserCost user_wait_cost =
    scale_to_user_cost(v.wait_cost(tw_r.billable_total_wait));

  return Route(v.id,
               std::move(steps),
               user_fixed_cost + user_travel_cost + user_task_cost +
                 user_wait_cost,
               user_duration,
               eval_sum.distance,
               scale_to_user_duration(setup),
               scale_to_user_duration(service),
               user_waiting_time,
               priority,
               sum_deliveries,
               sum_pickups,
               v.profile,
               v.description);
}

Solution format_solution(const Input& input, const TWSolution& tw_routes) {
  std::vector<Route> routes;
  routes.reserve(tw_routes.size());

  // All job ranks start with unassigned status.
  std::unordered_set<Index> unassigned_ranks;
  for (unsigned i = 0; i < input.jobs.size(); ++i) {
    unassigned_ranks.insert(i);
  }

  for (const auto& tw_route : tw_routes) {
    if (!tw_route.empty()) {
      routes.push_back(format_route(input, tw_route, unassigned_ranks));
    }
  }

  return Solution(input.zero_amount(),
                  std::move(routes),
                  get_unassigned_jobs_from_ranks(input, unassigned_ranks));
}

namespace vrptw_ls {

Eval relocate_travel_upper_bound(const Input& input,
                                 const utils::SolutionState& sol_state,
                                 const std::vector<Index>& s_route,
                                 Index s_vehicle,
                                 Index s_rank,
                                 const std::vector<Index>& t_route,
                                 Index t_vehicle,
                                 Index t_rank) {
  Eval s_gain = sol_state.node_gains[s_vehicle][s_rank];
  if (s_route.size() == 1) {
    s_gain.cost += input.vehicles[s_vehicle].fixed_cost();
  }

  const auto& t_v = input.vehicles[t_vehicle];
  Eval t_gain =
    -addition_eval(input, s_route[s_rank], t_v, t_route, t_rank);
  if (t_route.empty()) {
    t_gain.cost -= t_v.fixed_cost();
  }

  return s_gain + t_gain;
}

Eval two_opt_travel_upper_bound(const Input& input,
                                const utils::SolutionState& sol_state,
                                RawRoute& source,
                                Index /*s_vehicle*/,
                                Index s_rank,
                                RawRoute& target,
                                Index /*t_vehicle*/,
                                Index t_rank) {
  Eval s_gain =
    (t_rank + 1u < target.route.size())
      ? std::get<0>(addition_eval_delta(input,
                                        sol_state,
                                        source,
                                        s_rank + 1,
                                        source.route.size(),
                                        target,
                                        t_rank + 1,
                                        target.route.size()))
      : removal_gain(input, sol_state, source, s_rank + 1, source.route.size());

  Eval t_gain =
    (s_rank + 1u < source.route.size())
      ? std::get<0>(addition_eval_delta(input,
                                        sol_state,
                                        target,
                                        t_rank + 1,
                                        target.route.size(),
                                        source,
                                        s_rank + 1,
                                        source.route.size()))
      : removal_gain(input, sol_state, target, t_rank + 1, target.route.size());

  return s_gain + t_gain;
}

Eval reverse_two_opt_travel_upper_bound(const Input& input,
                                        const utils::SolutionState& sol_state,
                                        RawRoute& source,
                                        Index /*s_vehicle*/,
                                        Index s_rank,
                                        RawRoute& target,
                                        Index /*t_vehicle*/,
                                        Index t_rank) {
  Eval s_gain = std::get<1>(addition_eval_delta(input,
                                                sol_state,
                                                source,
                                                s_rank + 1,
                                                source.route.size(),
                                                target,
                                                0,
                                                t_rank + 1));

  Eval t_gain =
    (s_rank + 1u < source.route.size())
      ? std::get<1>(addition_eval_delta(input,
                                        sol_state,
                                        target,
                                        0,
                                        t_rank + 1,
                                        source,
                                        s_rank + 1,
                                        source.route.size()))
      : removal_gain(input, sol_state, target, 0, t_rank + 1);

  return s_gain + t_gain;
}

Eval route_exchange_travel_upper_bound(const Input& input,
                                       const utils::SolutionState& sol_state,
                                       RawRoute& source,
                                       Index s_vehicle,
                                       RawRoute& target,
                                       Index /*t_vehicle*/) {
  Eval s_gain =
    target.route.empty()
      ? sol_state.route_evals[s_vehicle]
      : std::get<0>(addition_eval_delta(input,
                                        sol_state,
                                        source,
                                        0,
                                        source.route.size(),
                                        target,
                                        0,
                                        target.route.size()));

  Eval t_gain =
    source.route.empty()
      ? sol_state.route_evals[s_vehicle]
      : std::get<0>(addition_eval_delta(input,
                                        sol_state,
                                        target,
                                        0,
                                        target.route.size(),
                                        source,
                                        0,
                                        source.route.size()));

  return s_gain + t_gain;
}

Eval intra_relocate_travel_upper_bound(const Input& input,
                                       const utils::SolutionState& sol_state,
                                       const std::vector<Index>& s_route,
                                       Index s_vehicle,
                                       Index s_rank,
                                       const std::vector<Index>& t_route,
                                       Index t_rank) {
  const auto& v_target = input.vehicles[s_vehicle];
  auto new_rank = t_rank;
  if (s_rank < t_rank) {
    ++new_rank;
  }
  return sol_state.node_gains[s_vehicle][s_rank] -
         addition_eval(input, s_route[s_rank], v_target, t_route, new_rank);
}

Eval intra_exchange_travel_upper_bound(const Input& input,
                                       const utils::SolutionState& sol_state,
                                       const std::vector<Index>& s_route,
                                       Index s_vehicle,
                                       Index s_rank,
                                       Index t_rank) {
  const auto& v = input.vehicles[s_vehicle];
  const Eval s_gain =
    sol_state.node_gains[s_vehicle][s_rank] -
    in_place_delta_eval(input, s_route[t_rank], v, s_route, s_rank);
  const Eval t_gain =
    sol_state.node_gains[s_vehicle][t_rank] -
    in_place_delta_eval(input, s_route[s_rank], v, s_route, t_rank);
  return s_gain + t_gain;
}

Eval intra_two_opt_travel_upper_bound(const Input& input,
                                      const utils::SolutionState& sol_state,
                                      RawRoute& source,
                                      Index s_rank,
                                      Index t_rank) {
  return std::get<1>(addition_eval_delta(input,
                                         sol_state,
                                         source,
                                         s_rank,
                                         t_rank + 1,
                                         source,
                                         s_rank,
                                         t_rank + 1));
}

} // namespace vrptw_ls

} // namespace vroom::utils
