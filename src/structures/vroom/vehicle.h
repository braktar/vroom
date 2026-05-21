#ifndef VEHICLE_H
#define VEHICLE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <string>
#include <tuple>
#include <unordered_map>

#include "structures/typedefs.h"
#include "structures/vroom/amount.h"
#include "structures/vroom/break.h"
#include "structures/vroom/cost_wrapper.h"
#include "structures/vroom/eval.h"
#include "structures/vroom/input/vehicle_step.h"
#include "structures/vroom/location.h"
#include "structures/vroom/time_window.h"

namespace vroom {

struct VehicleCosts {
  const Cost fixed;
  const Cost per_hour;
  const Cost per_km;
  const Cost per_task_hour;
  const Cost per_wait_hour;

  explicit VehicleCosts(UserCost fixed = 0,
                        UserCost per_hour = DEFAULT_COST_PER_HOUR,
                        UserCost per_km = DEFAULT_COST_PER_KM,
                        UserCost per_task_hour = DEFAULT_COST_PER_TASK_HOUR,
                        UserCost per_wait_hour = DEFAULT_COST_PER_WAIT_HOUR)
    : fixed(utils::scale_from_user_cost(fixed)),
      per_hour(static_cast<Cost>(per_hour)),
      per_km(static_cast<Cost>(per_km)),
      per_task_hour(static_cast<Cost>(per_task_hour)),
      per_wait_hour(static_cast<Cost>(per_wait_hour)){};

  friend bool operator==(const VehicleCosts& lhs,
                         const VehicleCosts& rhs) = default;

  friend bool operator<(const VehicleCosts& lhs, const VehicleCosts& rhs) {
    return std::tie(lhs.fixed,
                    lhs.per_hour,
                    lhs.per_km,
                    lhs.per_task_hour,
                    lhs.per_wait_hour) <
           std::tie(rhs.fixed,
                    rhs.per_hour,
                    rhs.per_km,
                    rhs.per_task_hour,
                    rhs.per_wait_hour);
  }
};

struct Vehicle {
  const Id id;
  std::optional<Location> start;
  std::optional<Location> end;
  const std::string profile;
  const Amount capacity;
  const Skills skills;
  const TimeWindow tw;
  const std::vector<Break> breaks;
  const std::string description;
  const VehicleCosts costs;
  CostWrapper cost_wrapper;
  size_t max_tasks;
  const Duration max_travel_time;
  const Distance max_distance;
  const Duration max_duration;
  const bool has_break_max_load;
  std::vector<VehicleStep> steps;
  Index type;
  const std::string type_str;
  std::unordered_map<Id, Index> break_id_to_rank;
  // Optional latest departure time from depot (internal duration scale):
  // vehicle may not leave start depot after this instant.
  const std::optional<Duration> departure;

  Vehicle(
    Id id,
    const std::optional<Location>& start,
    const std::optional<Location>& end,
    std::string profile = DEFAULT_PROFILE,
    const Amount& capacity = Amount(0),
    Skills skills = Skills(),
    const TimeWindow& tw = TimeWindow(),
    const std::vector<Break>& breaks = std::vector<Break>(),
    std::string description = "",
    const VehicleCosts& costs = VehicleCosts(),
    double speed_factor = 1.,
    const std::optional<size_t>& max_tasks = std::optional<size_t>(),
    const std::optional<UserDuration>& max_travel_time =
      std::optional<UserDuration>(),
    const std::optional<UserDistance>& max_distance =
      std::optional<UserDistance>(),
    const std::optional<UserDuration>& max_duration =
      std::optional<UserDuration>(),
    const std::vector<VehicleStep>& input_steps = std::vector<VehicleStep>(),
    std::string type_str = NO_TYPE,
    const std::optional<UserDuration>& departure = std::nullopt);

  bool has_start() const;

  bool has_end() const;

  bool has_same_locations(const Vehicle& other) const;

  bool has_same_profile(const Vehicle& other) const;

  bool cost_based_on_metrics() const;

  Duration available_duration() const;

  Cost fixed_cost() const {
    return costs.fixed;
  }

  Cost task_cost(Duration task_duration) const {
    return costs.per_task_hour * task_duration;
  }

  Eval task_eval(Duration task_duration) const {
    return Eval(task_cost(task_duration), 0, 0, task_duration, 0);
  }

  // Earliest time the route timeline may begin at depot (depot release).
  Duration earliest_route_start() const {
    return tw.start;
  }

  bool has_latest_departure() const {
    return departure.has_value();
  }

  Cost wait_cost(Duration wait_duration) const {
    return costs.per_wait_hour * wait_duration;
  }

  Duration duration(Index i, Index j) const {
    return cost_wrapper.duration(i, j);
  }

  Cost cost(Index i, Index j) const {
    return cost_wrapper.cost(i, j);
  }

  Eval eval(Index i, Index j) const {
    return Eval(cost_wrapper.cost(i, j),
                cost_wrapper.duration(i, j),
                cost_wrapper.distance(i, j));
  }

  bool ok_for_travel_time(Duration d) const {
    assert(0 <= d);
    return d <= max_travel_time;
  }

  bool ok_for_distance(Distance d) const {
    assert(0 <= d);
    return d <= max_distance;
  }

  bool ok_for_total_duration(const Eval& e) const {
    assert(0 <= e.duration && 0 <= e.task_duration && 0 <= e.wait_duration);
    return e.duration + e.task_duration + e.wait_duration <= max_duration;
  }

  bool ok_for_range_bounds(const Eval& e) const {
    assert(0 <= e.duration && 0 <= e.distance);
    return ok_for_travel_time(e.duration) && ok_for_distance(e.distance) &&
           ok_for_total_duration(e);
  }

  bool has_range_bounds() const;

  Index break_rank(Id break_id) const;

  friend bool operator<(const Vehicle& lhs, const Vehicle& rhs) {
    // Sort by:
    //   - decreasing max_tasks
    //   - decreasing capacity
    //   - decreasing TW length
    //   - decreasing range (max travel time and distance)
    return std::tie(rhs.max_tasks,
                    rhs.capacity,
                    rhs.tw.length,
                    rhs.max_travel_time,
                    rhs.max_distance,
                    rhs.max_duration) < std::tie(lhs.max_tasks,
                                                 lhs.capacity,
                                                 lhs.tw.length,
                                                 lhs.max_travel_time,
                                                 lhs.max_distance,
                                                 lhs.max_duration);
  }
};

} // namespace vroom

#endif
