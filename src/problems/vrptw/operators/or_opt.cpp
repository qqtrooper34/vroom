/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/or_opt.h"

namespace vroom::vrptw {

OrOpt::OrOpt(const Input& input,
             const utils::SolutionState& sol_state,
             TWRoute& tw_s_route,
             Index s_vehicle,
             Index s_rank,
             TWRoute& tw_t_route,
             Index t_vehicle,
             Index t_rank)
  : cvrp::OrOpt(input,
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

bool OrOpt::is_valid() {
  bool valid =
    cvrp::OrOpt::is_valid() && _tw_s_route.is_valid_removal(_input, s_rank, 2);

  if (!valid) {
    return false;
  }

  // Check route_position constraints for both jobs being moved
  const auto& job1 = _input.jobs[s_route[s_rank]];
  const auto& job2 = _input.jobs[s_route[s_rank + 1]];

  const auto t_first_count = _sol_state.first_jobs_count[t_vehicle];
  const auto t_last_count = _sol_state.last_jobs_count[t_vehicle];
  const auto t_route_size = t_route.size();

  // Check job1 at t_rank
  switch (job1.route_position) {
    case ROUTE_POSITION::FIRST:
      if (t_rank >= t_first_count) {
        return false;
      }
      break;
    case ROUTE_POSITION::LAST:
      // After inserting 2 jobs, t_rank becomes relative to new size
      if (t_rank < t_route_size - t_last_count) {
        return false;
      }
      break;
    case ROUTE_POSITION::NONE:
      if ((t_first_count > 0 && t_rank < t_first_count) ||
          (t_last_count > 0 && t_rank >= t_route_size - t_last_count)) {
        return false;
      }
      break;
  }

  // Check job2 at t_rank+1
  switch (job2.route_position) {
    case ROUTE_POSITION::FIRST:
      if (t_rank + 1 > t_first_count) {
        return false;
      }
      break;
    case ROUTE_POSITION::LAST:
      if (t_rank + 1 < t_route_size - t_last_count + 1) {
        return false;
      }
      break;
    case ROUTE_POSITION::NONE:
      if ((t_first_count > 0 && t_rank + 1 <= t_first_count) ||
          (t_last_count > 0 && t_rank + 1 > t_route_size - t_last_count)) {
        return false;
      }
      break;
  }

  // Keep edge direction.
  auto s_start = s_route.begin() + s_rank;
  is_normal_valid =
    is_normal_valid && _tw_t_route.is_valid_addition_for_tw(_input,
                                                            edge_delivery,
                                                            s_start,
                                                            s_start + 2,
                                                            t_rank,
                                                            t_rank);
  // Reverse edge direction.
  auto s_reverse_start = s_route.rbegin() + s_route.size() - 2 - s_rank;
  is_reverse_valid = is_reverse_valid &&
                     _tw_t_route.is_valid_addition_for_tw(_input,
                                                          edge_delivery,
                                                          s_reverse_start,
                                                          s_reverse_start + 2,
                                                          t_rank,
                                                          t_rank);

  return is_normal_valid || is_reverse_valid;
}

void OrOpt::apply() {
  if (reverse_s_edge) {
    auto s_reverse_start = s_route.rbegin() + s_route.size() - 2 - s_rank;
    _tw_t_route.replace(_input,
                        edge_delivery,
                        s_reverse_start,
                        s_reverse_start + 2,
                        t_rank,
                        t_rank);
    _tw_s_route.remove(_input, s_rank, 2);
  } else {
    auto s_start = s_route.begin() + s_rank;
    _tw_t_route
      .replace(_input, edge_delivery, s_start, s_start + 2, t_rank, t_rank);
    _tw_s_route.remove(_input, s_rank, 2);
  }
}

} // namespace vroom::vrptw
