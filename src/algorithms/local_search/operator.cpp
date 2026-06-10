/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "algorithms/local_search/operator.h"

namespace vroom::ls {

OperatorName Operator::get_name() const {
  return _name;
}

void Operator::set_best_known_threshold(Eval threshold) {
  best_known_threshold = threshold;
  gain_computed = false;
  wait_gain_adjusted = false;
  wait_gain_upper_bound.reset();
}

void Operator::set_wait_gain_upper_bound(std::optional<Cost> ub) {
  wait_gain_upper_bound = std::move(ub);
}

const std::optional<Cost>& Operator::get_wait_gain_upper_bound() const {
  return wait_gain_upper_bound;
}

void Operator::ensure_travel_gain_computed() {
  if (!gain_computed) {
    wait_gain_upper_bound.reset();
    compute_gain();
  }
}

bool Operator::prunable_by_travel_upper_bound(const Eval& current_best) {
  (void)current_best;
  return false;
}

Eval Operator::gain() {
  ensure_travel_gain_computed();
  if (_input.has_nonzero_per_wait_hour()) {
    apply_wait_gain_adjustment();
  }
  return stored_gain;
}

namespace {

bool eval_within_travel_and_distance_bounds(const Vehicle& v, const Eval& e) {
  return v.ok_for_travel_time(e.duration) && v.ok_for_distance(e.distance);
}

bool eval_within_range_bounds(const Vehicle& v, const Eval& e) {
  if (!eval_within_travel_and_distance_bounds(v, e)) {
    return false;
  }
  if (v.max_duration != DEFAULT_MAX_DURATION) {
    // Wait is not tracked in operator gains; max_duration is checked in VRPTW
    // operator is_valid() implementations.
    return true;
  }
  return v.ok_for_total_duration(e);
}

} // namespace

bool Operator::is_valid_for_source_range_bounds() const {
  const auto& s_v = _input.vehicles[s_vehicle];
  return eval_within_range_bounds(s_v,
                                  _sol_state.route_evals[s_vehicle] - s_gain);
}

bool Operator::is_valid_for_target_range_bounds() const {
  const auto& t_v = _input.vehicles[t_vehicle];
  return eval_within_range_bounds(t_v,
                                  _sol_state.route_evals[t_vehicle] - t_gain);
}

bool Operator::is_valid_for_range_bounds() const {
  assert(s_vehicle == t_vehicle);
  assert(gain_computed);

  const auto& s_v = _input.vehicles[s_vehicle];
  return eval_within_range_bounds(s_v,
                                  _sol_state.route_evals[s_vehicle] -
                                    stored_gain);
}

std::vector<Index> Operator::required_unassigned() const {
  return std::vector<Index>();
}

bool Operator::invalidated_by(Index) const {
  return false;
}

} // namespace vroom::ls
