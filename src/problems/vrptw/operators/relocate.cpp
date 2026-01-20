/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/relocate.h"

namespace vroom::vrptw {

Relocate::Relocate(const Input& input,
                   const utils::SolutionState& sol_state,
                   TWRoute& tw_s_route,
                   Index s_vehicle,
                   Index s_rank,
                   TWRoute& tw_t_route,
                   Index t_vehicle,
                   Index t_rank)
  : cvrp::Relocate(input,
                   sol_state,
                   static_cast<RawRoute&>(tw_s_route),
                   s_vehicle,
                   s_rank,
                   static_cast<RawRoute&>(tw_t_route),
                   t_vehicle,
                   t_rank),
    _tw_s_route(tw_s_route),
    _tw_t_route(tw_t_route) {
}

bool Relocate::is_valid() {
  if (!cvrp::Relocate::is_valid()) {
    return false;
  }

  // Check route_position constraint for target route
  const auto& job = _input.jobs[s_route[s_rank]];
  const auto first_count = _sol_state.first_jobs_count[t_vehicle];
  const auto last_count = _sol_state.last_jobs_count[t_vehicle];
  const auto target_size = t_route.size();

  switch (job.route_position) {
    case ROUTE_POSITION::FIRST:
      // FIRST jobs must be in the first positions
      if (t_rank > first_count) {
        return false;
      }
      break;
    case ROUTE_POSITION::LAST:
      // LAST jobs must be in the last positions
      if (t_rank < target_size - last_count) {
        return false;
      }
      break;
    case ROUTE_POSITION::NONE:
      // Normal jobs cannot be placed in FIRST or LAST zones
      if ((first_count > 0 && t_rank < first_count) ||
          (last_count > 0 && t_rank > target_size - last_count)) {
        return false;
      }
      break;
  }

  return _tw_t_route.is_valid_addition_for_tw(_input,
                                              s_route[s_rank],
                                              t_rank) &&
         _tw_s_route.is_valid_removal(_input, s_rank, 1);
}

void Relocate::apply() {
  auto relocate_job_rank = s_route[s_rank];

  _tw_s_route.remove(_input, s_rank, 1);
  _tw_t_route.add(_input, relocate_job_rank, t_rank);
}

} // namespace vroom::vrptw
