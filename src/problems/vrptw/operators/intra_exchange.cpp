/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/intra_exchange.h"

namespace vroom::vrptw {

IntraExchange::IntraExchange(const Input& input,
                             const utils::SolutionState& sol_state,
                             TWRoute& tw_s_route,
                             Index s_vehicle,
                             Index s_rank,
                             Index t_rank)
  : cvrp::IntraExchange(input,
                        sol_state,
                        static_cast<RawRoute&>(tw_s_route),
                        s_vehicle,
                        s_rank,
                        t_rank),
    _tw_s_route(tw_s_route) {
}

bool IntraExchange::is_valid() {
  if (!cvrp::IntraExchange::is_valid()) {
    return false;
  }

  // Check route_position constraints for both jobs being exchanged
  const auto& job_s = _input.jobs[s_route[s_rank]];
  const auto& job_t = _input.jobs[s_route[t_rank]];
  const auto first_count = _sol_state.first_jobs_count[s_vehicle];
  const auto last_count = _sol_state.last_jobs_count[s_vehicle];
  const auto route_size = s_route.size();

  // Check if job_s can go to t_rank
  switch (job_s.route_position) {
    case ROUTE_POSITION::FIRST:
      if (t_rank >= first_count) {
        return false;
      }
      break;
    case ROUTE_POSITION::LAST:
      if (t_rank < route_size - last_count) {
        return false;
      }
      break;
    case ROUTE_POSITION::NONE:
      if ((first_count > 0 && t_rank < first_count) ||
          (last_count > 0 && t_rank >= route_size - last_count)) {
        return false;
      }
      break;
  }

  // Check if job_t can go to s_rank
  switch (job_t.route_position) {
    case ROUTE_POSITION::FIRST:
      if (s_rank >= first_count) {
        return false;
      }
      break;
    case ROUTE_POSITION::LAST:
      if (s_rank < route_size - last_count) {
        return false;
      }
      break;
    case ROUTE_POSITION::NONE:
      if ((first_count > 0 && s_rank < first_count) ||
          (last_count > 0 && s_rank >= route_size - last_count)) {
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

void IntraExchange::apply() {
  _tw_s_route.replace(_input,
                      _delivery,
                      _moved_jobs.begin(),
                      _moved_jobs.end(),
                      _first_rank,
                      _last_rank);
}

std::vector<Index> IntraExchange::addition_candidates() const {
  return {s_vehicle};
}

} // namespace vroom::vrptw
