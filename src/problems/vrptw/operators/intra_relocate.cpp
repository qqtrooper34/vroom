/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/intra_relocate.h"

namespace vroom::vrptw {

IntraRelocate::IntraRelocate(const Input& input,
                             const utils::SolutionState& sol_state,
                             TWRoute& tw_s_route,
                             Index s_vehicle,
                             Index s_rank,
                             Index t_rank)
  : cvrp::IntraRelocate(input,
                        sol_state,
                        static_cast<RawRoute&>(tw_s_route),
                        s_vehicle,
                        s_rank,
                        t_rank),
    _tw_s_route(tw_s_route) {
}

bool IntraRelocate::is_valid() {
  if (!cvrp::IntraRelocate::is_valid()) {
    return false;
  }

  // Check route_position constraint
  const auto& job = _input.jobs[s_route[s_rank]];
  const auto first_count = _sol_state.first_jobs_count[s_vehicle];
  const auto last_count = _sol_state.last_jobs_count[s_vehicle];
  const auto route_size = s_route.size();

  switch (job.route_position) {
    case ROUTE_POSITION::FIRST:
      // FIRST jobs must stay in the first positions
      if (t_rank >= first_count) {
        return false;
      }
      break;
    case ROUTE_POSITION::LAST:
      // LAST jobs must stay in the last positions
      // After removal, effective position shifts
      if (s_rank < t_rank) {
        // Moving forward
        if (t_rank < route_size - last_count - 1) {
          return false;
        }
      } else {
        // Moving backward
        if (t_rank < route_size - last_count) {
          return false;
        }
      }
      break;
    case ROUTE_POSITION::NONE:
      // Normal jobs cannot move into FIRST or LAST zones
      if ((first_count > 0 && t_rank < first_count) ||
          (last_count > 0 && t_rank >= route_size - last_count)) {
        return false;
      }
      break;
  }

  return _tw_s_route.is_valid_addition_for_tw(_input,
                                              _delivery,
                                              _moved_jobs.begin(),
                                              _moved_jobs.end(),
                                              _first_rank,
                                              _last_rank);
}

void IntraRelocate::apply() {
  _tw_s_route.replace(_input,
                      _delivery,
                      _moved_jobs.begin(),
                      _moved_jobs.end(),
                      _first_rank,
                      _last_rank);
}

std::vector<Index> IntraRelocate::addition_candidates() const {
  return {s_vehicle};
}

} // namespace vroom::vrptw
