/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/two_opt.h"

namespace vroom::vrptw {

TwoOpt::TwoOpt(const Input& input,
               const utils::SolutionState& sol_state,
               TWRoute& tw_s_route,
               Index s_vehicle,
               Index s_rank,
               TWRoute& tw_t_route,
               Index t_vehicle,
               Index t_rank)
  : cvrp::TwoOpt(input,
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

bool TwoOpt::is_valid() {
  if (!cvrp::TwoOpt::is_valid()) {
    return false;
  }

  // Check route_position constraints for jobs moving between routes
  const auto s_first_count = _sol_state.first_jobs_count[s_vehicle];
  const auto s_last_count = _sol_state.last_jobs_count[s_vehicle];
  const auto t_first_count = _sol_state.first_jobs_count[t_vehicle];
  const auto t_last_count = _sol_state.last_jobs_count[t_vehicle];

  // Jobs from s_route[s_rank+1..end] go to t_route starting at t_rank+1
  Index t_insert_pos = t_rank + 1;
  for (Index i = s_rank + 1; i < s_route.size(); ++i) {
    const auto& job = _input.jobs[s_route[i]];
    switch (job.route_position) {
      case ROUTE_POSITION::FIRST:
        // FIRST jobs cannot go to the tail of another route
        if (t_insert_pos > t_first_count) {
          return false;
        }
        break;
      case ROUTE_POSITION::LAST:
        // This should be fine as it's going to the end
        break;
      case ROUTE_POSITION::NONE:
        // Normal jobs should not go into FIRST zone of target
        if (t_first_count > 0 && t_insert_pos <= t_first_count) {
          return false;
        }
        break;
    }
    ++t_insert_pos;
  }

  // Jobs from t_route[t_rank+1..end] go to s_route starting at s_rank+1
  Index s_insert_pos = s_rank + 1;
  for (Index i = t_rank + 1; i < t_route.size(); ++i) {
    const auto& job = _input.jobs[t_route[i]];
    switch (job.route_position) {
      case ROUTE_POSITION::FIRST:
        // FIRST jobs cannot go to the tail of another route
        if (s_insert_pos > s_first_count) {
          return false;
        }
        break;
      case ROUTE_POSITION::LAST:
        // This should be fine as it's going to the end
        break;
      case ROUTE_POSITION::NONE:
        // Normal jobs should not go into FIRST zone of target
        if (s_first_count > 0 && s_insert_pos <= s_first_count) {
          return false;
        }
        break;
    }
    ++s_insert_pos;
  }

  return _tw_t_route.is_valid_addition_for_tw(_input,
                                              _s_delivery,
                                              s_route.begin() + s_rank + 1,
                                              s_route.end(),
                                              t_rank + 1,
                                              t_route.size()) &&
         _tw_s_route.is_valid_addition_for_tw(_input,
                                              _t_delivery,
                                              t_route.begin() + t_rank + 1,
                                              t_route.end(),
                                              s_rank + 1,
                                              s_route.size());
}

void TwoOpt::apply() {
  std::vector<Index> t_job_ranks;
  t_job_ranks.insert(t_job_ranks.begin(),
                     t_route.begin() + t_rank + 1,
                     t_route.end());

  _tw_t_route.replace(_input,
                      _s_delivery,
                      s_route.begin() + s_rank + 1,
                      s_route.end(),
                      t_rank + 1,
                      t_route.size());
  _tw_s_route.replace(_input,
                      _t_delivery,
                      t_job_ranks.begin(),
                      t_job_ranks.end(),
                      s_rank + 1,
                      s_route.size());
}

} // namespace vroom::vrptw
