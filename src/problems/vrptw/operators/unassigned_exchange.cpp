/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/unassigned_exchange.h"

namespace vroom::vrptw {

UnassignedExchange::UnassignedExchange(const Input& input,
                                       const utils::SolutionState& sol_state,
                                       std::unordered_set<Index>& unassigned,
                                       TWRoute& tw_s_route,
                                       Index s_vehicle,
                                       Index s_rank,
                                       Index t_rank,
                                       Index u)
  : cvrp::UnassignedExchange(input,
                             sol_state,
                             unassigned,
                             static_cast<RawRoute&>(tw_s_route),
                             s_vehicle,
                             s_rank,
                             t_rank,
                             u),
    _tw_s_route(tw_s_route) {
}

bool UnassignedExchange::is_valid() {
  if (!cvrp::UnassignedExchange::is_valid()) {
    return false;
  }

  // Check route_position constraint for the unassigned job being inserted
  const auto& job_u = _input.jobs[_u];
  const auto first_count = _sol_state.first_jobs_count[s_vehicle];
  const auto last_count = _sol_state.last_jobs_count[s_vehicle];
  const auto route_size = s_route.size();

  // Determine the effective insertion rank for _u
  Index u_rank = (s_rank < t_rank) ? t_rank - 1 : t_rank;

  switch (job_u.route_position) {
    case ROUTE_POSITION::FIRST:
      if (u_rank >= first_count) {
        return false;
      }
      break;
    case ROUTE_POSITION::LAST:
      if (u_rank < route_size - last_count - 1) {
        return false;
      }
      break;
    case ROUTE_POSITION::NONE:
      if ((first_count > 0 && u_rank < first_count) ||
          (last_count > 0 && u_rank >= route_size - last_count - 1)) {
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

void UnassignedExchange::apply() {
  _tw_s_route.replace(_input,
                      _delivery,
                      _moved_jobs.begin(),
                      _moved_jobs.end(),
                      _first_rank,
                      _last_rank);

  assert(_unassigned.find(_u) != _unassigned.end());
  _unassigned.erase(_u);
  assert(_unassigned.find(_removed) == _unassigned.end());
  _unassigned.insert(_removed);
}

} // namespace vroom::vrptw
