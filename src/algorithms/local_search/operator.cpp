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
  wait_gain_upper_bound.reset();
}

void Operator::set_wait_gain_upper_bound(std::optional<Cost> ub) {
  wait_gain_upper_bound = std::move(ub);
}

const std::optional<Cost>& Operator::get_wait_gain_upper_bound() const {
  return wait_gain_upper_bound;
}

Eval Operator::gain() {
  if (!gain_computed) {
    wait_gain_upper_bound.reset();
    this->compute_gain();
  }
  return stored_gain;
}

bool Operator::is_valid_for_source_range_bounds() const {
  const auto& s_v = _input.vehicles[s_vehicle];
  return s_v.ok_for_range_bounds(_sol_state.route_evals[s_vehicle] - s_gain);
}

bool Operator::is_valid_for_target_range_bounds() const {
  const auto& t_v = _input.vehicles[t_vehicle];
  return t_v.ok_for_range_bounds(_sol_state.route_evals[t_vehicle] - t_gain);
}

bool Operator::is_valid_for_range_bounds() const {
  assert(s_vehicle == t_vehicle);
  assert(gain_computed);

  const auto& s_v = _input.vehicles[s_vehicle];
  return s_v.ok_for_range_bounds(_sol_state.route_evals[s_vehicle] -
                                 stored_gain);
}

std::vector<Index> Operator::required_unassigned() const {
  return std::vector<Index>();
}

bool Operator::invalidated_by(Index) const {
  return false;
}

} // namespace vroom::ls
