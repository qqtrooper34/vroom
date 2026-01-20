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

Eval Operator::gain() {
  if (!gain_computed) {
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

bool Operator::is_valid_job_rank_for_route_position(Index job_rank,
                                                    Index v,
                                                    Index insertion_rank) const {
  const auto& job = _input.jobs[job_rank];
  const auto first_count = _sol_state.first_jobs_count[v];
  const auto last_count = _sol_state.last_jobs_count[v];
  const auto route_size = (v == s_vehicle) ? s_route.size() : t_route.size();

  switch (job.route_position) {
    case ROUTE_POSITION::FIRST:
      return insertion_rank <= first_count;
    case ROUTE_POSITION::LAST:
      return insertion_rank >= route_size - last_count;
    case ROUTE_POSITION::NONE:
      return (first_count == 0 || insertion_rank >= first_count) &&
             (last_count == 0 || insertion_rank <= route_size - last_count);
  }
  return true;
}

std::vector<Index> Operator::required_unassigned() const {
  return std::vector<Index>();
}

bool Operator::invalidated_by(Index) const {
  return false;
}

} // namespace vroom::ls
