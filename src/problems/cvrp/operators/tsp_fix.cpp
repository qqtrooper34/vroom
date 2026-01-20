/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/tsp_fix.h"
#include "problems/tsp/tsp.h"

namespace vroom::cvrp {

TSPFix::TSPFix(const Input& input,
               const utils::SolutionState& sol_state,
               RawRoute& s_route,
               Index s_vehicle)
  // Use dummy 0 values for unused ranks.
  : Operator(OperatorName::TSPFix,
             input,
             sol_state,
             s_route,
             s_vehicle,
             0,
             s_route,
             s_vehicle,
             0),
    _s_delivery(source.load_at_step(0)) {
  assert(s_route.size() >= 2);
}

void TSPFix::compute_gain() {
  std::vector<Index> jobs = s_route;
  const TSP tsp(_input, std::move(jobs), s_vehicle);
  tsp_route = tsp.raw_solve(1, Timeout());

  s_gain = _sol_state.route_evals[s_vehicle] -
           utils::route_eval_for_vehicle(_input, s_vehicle, tsp_route);
  stored_gain = s_gain;
  gain_computed = true;
}

bool TSPFix::is_valid() {
  bool valid = is_valid_for_source_range_bounds();

  // Check route_position constraints for the new TSP order
  if (valid) {
    Index first_count = 0;
    Index last_count = 0;
    for (const auto& job_rank : tsp_route) {
      const auto& job = _input.jobs[job_rank];
      if (job.route_position == ROUTE_POSITION::FIRST) {
        ++first_count;
      } else if (job.route_position == ROUTE_POSITION::LAST) {
        ++last_count;
      }
    }

    const auto route_size = tsp_route.size();
    for (Index rank = 0; rank < route_size && valid; ++rank) {
      const auto& job = _input.jobs[tsp_route[rank]];
      switch (job.route_position) {
        case ROUTE_POSITION::FIRST:
          if (rank >= first_count) {
            valid = false;
          }
          break;
        case ROUTE_POSITION::LAST:
          if (rank < route_size - last_count) {
            valid = false;
          }
          break;
        case ROUTE_POSITION::NONE:
          if ((first_count > 0 && rank < first_count) ||
              (last_count > 0 && rank >= route_size - last_count)) {
            valid = false;
          }
          break;
      }
    }
  }

  if (valid) {
    const RawRoute route(_input, s_vehicle, _input.zero_amount().size());

    valid = route.is_valid_addition_for_capacity_inclusion(_input,
                                                           _s_delivery,
                                                           tsp_route.begin(),
                                                           tsp_route.end(),
                                                           0,
                                                           0);
  }

  return valid;
}

void TSPFix::apply() {
  s_route = std::move(tsp_route);
  source.update_amounts(_input);
}

std::vector<Index> TSPFix::addition_candidates() const {
  return {s_vehicle};
}

std::vector<Index> TSPFix::update_candidates() const {
  return {s_vehicle};
}

} // namespace vroom::cvrp
