/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/mixed_exchange.h"

namespace vroom::vrptw {

MixedExchange::MixedExchange(const Input& input,
                             const utils::SolutionState& sol_state,
                             TWRoute& tw_s_route,
                             Index s_vehicle,
                             Index s_rank,
                             TWRoute& tw_t_route,
                             Index t_vehicle,
                             Index t_rank,
                             bool check_t_reverse)
  : cvrp::MixedExchange(input,
                        sol_state,
                        static_cast<RawRoute&>(tw_s_route),
                        s_vehicle,
                        s_rank,
                        static_cast<RawRoute&>(tw_t_route),
                        t_vehicle,
                        t_rank,
                        check_t_reverse),
    _tw_s_route(tw_s_route),
    _tw_t_route(tw_t_route) {
}

bool MixedExchange::is_valid() {
  bool valid = cvrp::MixedExchange::is_valid();
  if (!valid) {
    return false;
  }

  // Check route_position constraints
  const auto& job_s = _input.jobs[s_route[s_rank]];
  const auto& job_t1 = _input.jobs[t_route[t_rank]];
  const auto& job_t2 = _input.jobs[t_route[t_rank + 1]];

  // Check source job going to target route at t_rank
  const auto t_first_count = _sol_state.first_jobs_count[t_vehicle];
  const auto t_last_count = _sol_state.last_jobs_count[t_vehicle];
  const auto t_route_size = t_route.size();

  switch (job_s.route_position) {
    case ROUTE_POSITION::FIRST:
      if (t_rank >= t_first_count) {
        return false;
      }
      break;
    case ROUTE_POSITION::LAST:
      // After replacing 2 jobs with 1, adjust position
      if (t_rank < t_route_size - t_last_count - 1) {
        return false;
      }
      break;
    case ROUTE_POSITION::NONE:
      if ((t_first_count > 0 && t_rank < t_first_count) ||
          (t_last_count > 0 && t_rank >= t_route_size - t_last_count - 1)) {
        return false;
      }
      break;
  }

  // Check target jobs going to source route at s_rank and s_rank+1
  const auto s_first_count = _sol_state.first_jobs_count[s_vehicle];
  const auto s_last_count = _sol_state.last_jobs_count[s_vehicle];
  const auto s_route_size = s_route.size();

  // job_t1 goes to s_rank
  switch (job_t1.route_position) {
    case ROUTE_POSITION::FIRST:
      if (s_rank >= s_first_count) {
        return false;
      }
      break;
    case ROUTE_POSITION::LAST:
      // After replacing 1 job with 2, adjust position
      if (s_rank < s_route_size - s_last_count) {
        return false;
      }
      break;
    case ROUTE_POSITION::NONE:
      if ((s_first_count > 0 && s_rank < s_first_count) ||
          (s_last_count > 0 && s_rank >= s_route_size - s_last_count)) {
        return false;
      }
      break;
  }

  // job_t2 goes to s_rank+1 (after insertion, effectively at position s_rank+1)
  switch (job_t2.route_position) {
    case ROUTE_POSITION::FIRST:
      if (s_rank + 1 > s_first_count) {
        return false;
      }
      break;
    case ROUTE_POSITION::LAST:
      if (s_rank + 1 < s_route_size - s_last_count + 1) {
        return false;
      }
      break;
    case ROUTE_POSITION::NONE:
      if ((s_first_count > 0 && s_rank + 1 <= s_first_count) ||
          (s_last_count > 0 && s_rank + 1 > s_route_size - s_last_count)) {
        return false;
      }
      break;
  }

  valid = _tw_t_route.is_valid_addition_for_tw(_input,
                                                source_delivery,
                                                s_route.begin() + s_rank,
                                                s_route.begin() + s_rank + 1,
                                                t_rank,
                                                t_rank + 2);

  if (valid) {
    // Keep target edge direction when inserting in source route.
    auto t_start = t_route.begin() + t_rank;
    s_is_normal_valid =
      s_is_normal_valid && _tw_s_route.is_valid_addition_for_tw(_input,
                                                                target_delivery,
                                                                t_start,
                                                                t_start + 2,
                                                                s_rank,
                                                                s_rank + 1);

    if (check_t_reverse) {
      // Reverse target edge direction when inserting in source route.
      auto t_reverse_start = t_route.rbegin() + t_route.size() - 2 - t_rank;
      s_is_reverse_valid =
        s_is_reverse_valid &&
        _tw_s_route.is_valid_addition_for_tw(_input,
                                             target_delivery,
                                             t_reverse_start,
                                             t_reverse_start + 2,
                                             s_rank,
                                             s_rank + 1);
    }
    valid = s_is_normal_valid || s_is_reverse_valid;
  }

  return valid;
}

void MixedExchange::apply() {
  assert(!reverse_t_edge ||
         (_input.jobs[t_route[t_rank]].type == JOB_TYPE::SINGLE &&
          _input.jobs[t_route[t_rank + 1]].type == JOB_TYPE::SINGLE));

  std::vector<Index> s_job_ranks({s_route[s_rank]});
  std::vector<Index> t_job_ranks;
  if (!reverse_t_edge) {
    auto t_start = t_route.begin() + t_rank;
    t_job_ranks.insert(t_job_ranks.begin(), t_start, t_start + 2);
  } else {
    auto t_reverse_start = t_route.rbegin() + t_route.size() - 2 - t_rank;
    t_job_ranks.insert(t_job_ranks.begin(),
                       t_reverse_start,
                       t_reverse_start + 2);
  }

  _tw_s_route.replace(_input,
                      target_delivery,
                      t_job_ranks.begin(),
                      t_job_ranks.end(),
                      s_rank,
                      s_rank + 1);

  _tw_t_route.replace(_input,
                      source_delivery,
                      s_job_ranks.begin(),
                      s_job_ranks.end(),
                      t_rank,
                      t_rank + 2);
}

} // namespace vroom::vrptw
