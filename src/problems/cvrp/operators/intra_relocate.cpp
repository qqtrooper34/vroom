/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/intra_relocate.h"
#include "utils/helpers.h"

namespace vroom::cvrp {

IntraRelocate::IntraRelocate(const Input& input,
                             const utils::SolutionState& sol_state,
                             RawRoute& s_raw_route,
                             Index s_vehicle,
                             Index s_rank,
                             Index t_rank)
  : Operator(OperatorName::IntraRelocate,
             input,
             sol_state,
             s_raw_route,
             s_vehicle,
             s_rank,
             s_raw_route,
             s_vehicle,
             t_rank),
    _moved_jobs((s_rank < t_rank) ? t_rank - s_rank + 1 : s_rank - t_rank + 1),
    _first_rank(std::min(s_rank, t_rank)),
    _last_rank(std::max(s_rank, t_rank) + 1),
    _delivery(source.delivery_in_range(_first_rank, _last_rank)) {
  assert(s_route.size() >= 2);
  assert(s_rank < s_route.size());
  assert(t_rank <= s_route.size() - 1);
  assert(s_rank != t_rank);

  if (t_rank < s_rank) {
    _moved_jobs[0] = s_route[s_rank];
    std::copy(s_route.begin() + t_rank,
              s_route.begin() + s_rank,
              _moved_jobs.begin() + 1);
  } else {
    std::copy(s_route.begin() + s_rank + 1,
              s_route.begin() + t_rank + 1,
              _moved_jobs.begin());
    _moved_jobs.back() = s_route[s_rank];
  }
}

void IntraRelocate::compute_gain() {
  const auto& v_target = _input.vehicles[s_vehicle];

  // For removal, we consider the cost of removing job at rank s_rank,
  // already stored in _sol_state.node_gains[s_vehicle][s_rank].

  // For addition, consider the cost of adding source job at new rank
  // *after* removal.
  auto new_rank = t_rank;
  if (s_rank < t_rank) {
    ++new_rank;
  }
  stored_gain =
    _sol_state.node_gains[s_vehicle][s_rank] -
    utils::addition_cost(_input, s_route[s_rank], v_target, t_route, new_rank);

  gain_computed = true;
}

bool IntraRelocate::is_valid() {
  // Check route_position constraint for all affected jobs
  const auto& moved_job = _input.jobs[s_route[s_rank]];
  const auto first_count = _sol_state.first_jobs_count[s_vehicle];
  const auto last_count = _sol_state.last_jobs_count[s_vehicle];
  const auto route_size = s_route.size();

  // Check the moved job's new position
  switch (moved_job.route_position) {
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

  // Check jobs that shift when we move the job
  // When moving from s_rank to t_rank:
  // - If s_rank < t_rank: jobs at [s_rank+1..t_rank] shift down by 1 (to [s_rank..t_rank-1])
  // - If s_rank > t_rank: jobs at [t_rank..s_rank-1] shift up by 1 (to [t_rank+1..s_rank])

  if (s_rank < t_rank) {
    // Jobs at positions [s_rank+1..t_rank] shift to [s_rank..t_rank-1]
    for (Index i = s_rank + 1; i <= t_rank; ++i) {
      const auto& job = _input.jobs[s_route[i]];
      const auto new_pos = i - 1;
      switch (job.route_position) {
        case ROUTE_POSITION::FIRST:
          if (new_pos >= first_count) return false;
          break;
        case ROUTE_POSITION::LAST:
          if (new_pos < route_size - last_count) return false;
          break;
        case ROUTE_POSITION::NONE:
          if ((first_count > 0 && new_pos < first_count) ||
              (last_count > 0 && new_pos >= route_size - last_count)) return false;
          break;
      }
    }
  } else {
    // Jobs at positions [t_rank..s_rank-1] shift to [t_rank+1..s_rank]
    for (Index i = t_rank; i < s_rank; ++i) {
      const auto& job = _input.jobs[s_route[i]];
      const auto new_pos = i + 1;
      switch (job.route_position) {
        case ROUTE_POSITION::FIRST:
          if (new_pos >= first_count) return false;
          break;
        case ROUTE_POSITION::LAST:
          if (new_pos < route_size - last_count) return false;
          break;
        case ROUTE_POSITION::NONE:
          if ((first_count > 0 && new_pos < first_count) ||
              (last_count > 0 && new_pos >= route_size - last_count)) return false;
          break;
      }
    }
  }

  return is_valid_for_range_bounds() &&
         source.is_valid_addition_for_capacity_inclusion(_input,
                                                         _delivery,
                                                         _moved_jobs.begin(),
                                                         _moved_jobs.end(),
                                                         _first_rank,
                                                         _last_rank);
}

void IntraRelocate::apply() {
  auto relocate_job_rank = s_route[s_rank];
  s_route.erase(s_route.begin() + s_rank);
  s_route.insert(t_route.begin() + t_rank, relocate_job_rank);

  source.update_amounts(_input);
}

std::vector<Index> IntraRelocate::addition_candidates() const {
  return {};
}

std::vector<Index> IntraRelocate::update_candidates() const {
  return {s_vehicle};
}

} // namespace vroom::cvrp
