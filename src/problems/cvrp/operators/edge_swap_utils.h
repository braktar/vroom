#ifndef CVRP_EDGE_SWAP_UTILS_H
#define CVRP_EDGE_SWAP_UTILS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/eval.h"

namespace vroom::cvrp {

// Travel deltas (gain_upper_bound) before validity checks.
#define CVRP_EDGE_SWAP_PREP(Class)                                             \
  do {                                                                         \
    if (!_gain_upper_bound_computed) {                                         \
      (void)gain_upper_bound();                                                \
    }                                                                          \
  } while (0)

// Capacity/range checks; Class must be the concrete CVRP operator (non-virtual
// is_valid).
#define CVRP_EDGE_SWAP_REQUIRE_VALID(Class)                                    \
  do {                                                                         \
    if (!Class::is_valid()) {                                                  \
      stored_gain = NO_GAIN;                                                   \
      gain_computed = true;                                                      \
      return;                                                                  \
    }                                                                          \
  } while (0)

} // namespace vroom::cvrp

#endif
