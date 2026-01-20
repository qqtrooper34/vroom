/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/intra_two_opt.h"

namespace vroom::vrptw {

IntraTwoOpt::IntraTwoOpt(const Input& input,
                         const utils::SolutionState& sol_state,
                         TWRoute& tw_s_route,
                         Index s_vehicle,
                         Index s_rank,
                         Index t_rank)
  : cvrp::IntraTwoOpt(input,
                      sol_state,
                      static_cast<RawRoute&>(tw_s_route),
                      s_vehicle,
                      s_rank,
                      t_rank),
    _tw_s_route(tw_s_route) {
}

bool IntraTwoOpt::is_valid() {
  bool valid = cvrp::IntraTwoOpt::is_valid();

  if (!valid) {
    return false;
  }

  // Check route_position constraint: reversal is invalid if it crosses
  // zone boundaries or reverses jobs with FIRST/LAST constraints
  const auto first_count = _sol_state.first_jobs_count[s_vehicle];
  const auto last_count = _sol_state.last_jobs_count[s_vehicle];
  const auto route_size = s_route.size();

  // Check if the segment crosses zone boundaries
  bool crosses_first_boundary = (s_rank < first_count && t_rank >= first_count);
  bool crosses_last_boundary =
    (s_rank < route_size - last_count && t_rank >= route_size - last_count);

  if (crosses_first_boundary || crosses_last_boundary) {
    // Segment spans multiple zones - check each job
    for (Index r = s_rank; r <= t_rank; ++r) {
      const auto& job = _input.jobs[s_route[r]];
      if (job.route_position != ROUTE_POSITION::NONE) {
        // Reversing would move this constrained job
        Index new_rank = s_rank + (t_rank - r);
        switch (job.route_position) {
          case ROUTE_POSITION::FIRST:
            if (new_rank >= first_count) {
              return false;
            }
            break;
          case ROUTE_POSITION::LAST:
            if (new_rank < route_size - last_count) {
              return false;
            }
            break;
          case ROUTE_POSITION::NONE:
            break;
        }
      }
    }
  }

  auto rev_t = s_route.rbegin() + (s_route.size() - t_rank - 1);
  auto rev_s_next = s_route.rbegin() + (s_route.size() - s_rank);

  return _tw_s_route.is_valid_addition_for_tw(_input,
                                               delivery,
                                               rev_t,
                                               rev_s_next,
                                               s_rank,
                                               t_rank + 1);
}

void IntraTwoOpt::apply() {
  std::vector<Index> reversed(s_route.rbegin() + (s_route.size() - t_rank - 1),
                              s_route.rbegin() + (s_route.size() - s_rank));

  _tw_s_route.replace(_input,
                      delivery,
                      reversed.begin(),
                      reversed.end(),
                      s_rank,
                      t_rank + 1);
}

} // namespace vroom::vrptw
