#ifndef INPUT_H
#define INPUT_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <array>
#include <chrono>
#include <unordered_map>
#include <vector>

#include <boost/optional.hpp>

#include "../../../problems/tsp/tsp.h"
#include "../../../routing/routed_wrapper.h"
#include "../../../routing/routing_io.h"
#include "../../../utils/exceptions.h"
#include "../../../utils/helpers.h"
#include "../../abstract/matrix.h"
#include "../../typedefs.h"
#include "../job.h"
#include "../vehicle.h"
#if LIBOSRM
#include "../../../routing/libosrm_wrapper.h"
#endif

class vrp;

class input {
private:
  std::chrono::high_resolution_clock::time_point _start_loading;
  std::chrono::high_resolution_clock::time_point _end_loading;
  std::chrono::high_resolution_clock::time_point _end_solving;
  std::chrono::high_resolution_clock::time_point _end_routing;
  PROBLEM_T _problem_type;
  std::unique_ptr<routing_io<cost_t>> _routing_wrapper;
  const bool _geometry;
  void check_cost_bound();
  void set_matrix();
  std::unordered_map<index_t, index_t> _index_to_job_rank;
  std::unordered_map<index_t, index_t> _index_to_loc_rank;
  std::set<index_t> _all_indices;
  std::unique_ptr<vrp> get_problem() const;

public:
  std::vector<job_t> _jobs;
  std::vector<vehicle_t> _vehicles;
  matrix<cost_t> _matrix;
  std::vector<cost_t> _max_cost_per_line;
  std::vector<cost_t> _max_cost_per_column;

  // List of locations added through add_* matching the matrix
  // ordering.
  std::vector<location_t> _locations;

  input(std::unique_ptr<routing_io<cost_t>> routing_wrapper, bool geometry);

  void add_job(const job_t& job);

  void add_vehicle(const vehicle_t& vehicle);

  index_t get_location_rank_from_index(index_t index) const;

  index_t get_job_rank_from_index(index_t index) const;

  PROBLEM_T get_problem_type() const;

  solution solve(unsigned nb_thread);

  friend input parse(const cl_args_t& cl_args);
};

#endif
