#ifndef EVAL_H
#define EVAL_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <tuple>

#include "structures/typedefs.h"

namespace vroom {

struct Eval {
  Cost cost;
  Duration duration;
  Distance distance;
  Duration task_duration;
  Duration wait_duration;

  constexpr Eval()
    : cost(0), duration(0), distance(0), task_duration(0), wait_duration(0){};

  constexpr explicit Eval(Cost cost,
                          Duration duration = 0,
                          Distance distance = 0,
                          Duration task_duration = 0,
                          Duration wait_duration = 0)
    : cost(cost),
      duration(duration),
      distance(distance),
      task_duration(task_duration),
      wait_duration(wait_duration){};

  Eval& operator+=(const Eval& rhs) {
    cost += rhs.cost;
    duration += rhs.duration;
    distance += rhs.distance;
    task_duration += rhs.task_duration;
    wait_duration += rhs.wait_duration;

    return *this;
  }

  Eval& operator-=(const Eval& rhs) {
    cost -= rhs.cost;
    duration -= rhs.duration;
    distance -= rhs.distance;
    task_duration -= rhs.task_duration;
    wait_duration -= rhs.wait_duration;

    return *this;
  }

  Eval operator-() const {
    return Eval(-cost, -duration, -distance, -task_duration, -wait_duration);
  }

  friend Eval operator+(Eval lhs, const Eval& rhs) {
    lhs += rhs;
    return lhs;
  }

  friend Eval operator-(Eval lhs, const Eval& rhs) {
    lhs -= rhs;
    return lhs;
  }

  // Lexicographic order; local search move gains are compared on cost only
  // (operator<=). wait_duration is included here for completeness when summing
  // full route Eval values.
  friend bool operator<(const Eval& lhs, const Eval& rhs) {
    return std::tie(lhs.cost,
                    lhs.duration,
                    lhs.distance,
                    lhs.task_duration,
                    lhs.wait_duration) <
           std::tie(rhs.cost,
                    rhs.duration,
                    rhs.distance,
                    rhs.task_duration,
                    rhs.wait_duration);
  }

  friend bool operator<=(const Eval& lhs, const Eval& rhs) {
    return lhs.cost <= rhs.cost;
  }

  friend bool operator==(const Eval& lhs, const Eval& rhs) = default;
};

constexpr Eval NO_EVAL =
  Eval(std::numeric_limits<Cost>::max(), 0, 0, 0, 0);
constexpr Eval NO_GAIN =
  Eval(std::numeric_limits<Cost>::min(), 0, 0, 0, 0);

} // namespace vroom

#endif
