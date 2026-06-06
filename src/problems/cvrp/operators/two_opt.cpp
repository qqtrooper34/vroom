/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/two_opt.h"

namespace vroom::cvrp {

TwoOpt::TwoOpt(const Input& input,
               const utils::SolutionState& sol_state,
               RawRoute& s_route,
               Index s_vehicle,
               Index s_rank,
               RawRoute& t_route,
               Index t_vehicle,
               Index t_rank)
  : Operator(OperatorName::TwoOpt,
             input,
             sol_state,
             s_route,
             s_vehicle,
             s_rank,
             t_route,
             t_vehicle,
             t_rank),
    _s_delivery(source.bwd_deliveries(s_rank)),
    _t_delivery(target.bwd_deliveries(t_rank)) {
  assert(s_vehicle != t_vehicle);
  assert(!s_route.empty());
  assert(!t_route.empty());
  assert(s_rank < s_route.size());
  assert(t_rank < t_route.size());

  assert(_sol_state.bwd_skill_rank[s_vehicle][t_vehicle] <= s_rank + 1);
  assert(_sol_state.bwd_skill_rank[t_vehicle][s_vehicle] <= t_rank + 1);
}

void TwoOpt::compute_gain() {
  const auto& s_v = _input.vehicles[s_vehicle];
  const auto& t_v = _input.vehicles[t_vehicle];

  const Index s_index = _input.jobs[s_route[s_rank]].index();
  const Index t_index = _input.jobs[t_route[t_rank]].index();
  const Index last_s = _input.jobs[s_route.back()].index();
  const Index last_t = _input.jobs[t_route.back()].index();

  Index new_last_s = last_t;
  Index new_last_t = last_s;

  // Cost of swapping route for vehicle s_vehicle after step
  // s_rank with route for vehicle t_vehicle after step
  // t_rank.

  // Basic costs in case we really swap jobs and not only the end of
  // the route. Otherwise remember that last job does not change.
  if (s_rank < s_route.size() - 1) {
    const Index next_index = _input.jobs[s_route[s_rank + 1]].index();
    s_gain += s_v.eval(s_index, next_index);
    t_gain -= t_v.eval(t_index, next_index);

    // Account for the change in cost across vehicles for the end of
    // source route. Cost of remaining route retrieved by subtracting
    // intermediate cost to overall cost.
    s_gain += _sol_state.fwd_costs[s_vehicle][s_vehicle].back();
    s_gain -= _sol_state.fwd_costs[s_vehicle][s_vehicle][s_rank + 1];
    t_gain -= _sol_state.fwd_costs[s_vehicle][t_vehicle].back();
    t_gain += _sol_state.fwd_costs[s_vehicle][t_vehicle][s_rank + 1];
  } else {
    new_last_t = t_index;
  }
  if (t_rank < t_route.size() - 1) {
    const Index next_index = _input.jobs[t_route[t_rank + 1]].index();
    t_gain += t_v.eval(t_index, next_index);
    s_gain -= s_v.eval(s_index, next_index);

    // Account for the change in cost across vehicles for the end of
    // target route. Cost of remaining route retrieved by subtracting
    // intermediate cost to overall cost.
    t_gain += _sol_state.fwd_costs[t_vehicle][t_vehicle].back();
    t_gain -= _sol_state.fwd_costs[t_vehicle][t_vehicle][t_rank + 1];
    s_gain -= _sol_state.fwd_costs[t_vehicle][s_vehicle].back();
    s_gain += _sol_state.fwd_costs[t_vehicle][s_vehicle][t_rank + 1];
  } else {
    new_last_s = s_index;
  }

  // Handling end route cost change because vehicle ends can be
  // different or none.
  if (s_v.has_end()) {
    auto end_s = s_v.end.value().index();
    s_gain += s_v.eval(last_s, end_s);
    s_gain -= s_v.eval(new_last_s, end_s);
  }
  if (t_v.has_end()) {
    auto end_t = t_v.end.value().index();
    t_gain += t_v.eval(last_t, end_t);
    t_gain -= t_v.eval(new_last_t, end_t);
  }

  // TAMS: compute service delta for both routes
  // Source loses jobs [s_rank+1..end], gains jobs from target [t_rank+1..end]
  // Target loses jobs [t_rank+1..end], gains jobs from source [s_rank+1..end]
  Duration s_removed_service = 0;
  Duration t_removed_service = 0;

  for (Index i = s_rank + 1; i < s_route.size(); ++i) {
    s_removed_service += _input.jobs[s_route[i]].services[s_v.type];
  }
  for (Index i = t_rank + 1; i < t_route.size(); ++i) {
    t_removed_service += _input.jobs[t_route[i]].services[t_v.type];
  }

  // s_gain.service = removed from source - added to source (from target)
  s_gain.service = s_removed_service - t_removed_service;
  // t_gain.service = removed from target - added to target (from source)
  t_gain.service = t_removed_service - s_removed_service;

  stored_gain = s_gain + t_gain;
  gain_computed = true;
}

bool TwoOpt::is_valid() {
  assert(gain_computed);

  // Check route_position constraints for tail swap
  // After two_opt:
  // - Source keeps [0..s_rank], gets target[t_rank+1..end]
  // - Target keeps [0..t_rank], gets source[s_rank+1..end]

  const auto new_s_size = (s_rank + 1) + (t_route.size() - t_rank - 1);
  const auto new_t_size = (t_rank + 1) + (s_route.size() - s_rank - 1);

  // Count FIRST/LAST in kept portions and swapped portions
  Index s_kept_first = 0, s_kept_last = 0;
  for (Index i = 0; i <= s_rank; ++i) {
    if (_input.jobs[s_route[i]].route_position == ROUTE_POSITION::FIRST) ++s_kept_first;
    if (_input.jobs[s_route[i]].route_position == ROUTE_POSITION::LAST) ++s_kept_last;
  }

  Index t_kept_first = 0, t_kept_last = 0;
  for (Index i = 0; i <= t_rank; ++i) {
    if (_input.jobs[t_route[i]].route_position == ROUTE_POSITION::FIRST) ++t_kept_first;
    if (_input.jobs[t_route[i]].route_position == ROUTE_POSITION::LAST) ++t_kept_last;
  }

  Index s_swap_first = 0, s_swap_last = 0;
  for (Index i = s_rank + 1; i < s_route.size(); ++i) {
    if (_input.jobs[s_route[i]].route_position == ROUTE_POSITION::FIRST) ++s_swap_first;
    if (_input.jobs[s_route[i]].route_position == ROUTE_POSITION::LAST) ++s_swap_last;
  }

  Index t_swap_first = 0, t_swap_last = 0;
  for (Index i = t_rank + 1; i < t_route.size(); ++i) {
    if (_input.jobs[t_route[i]].route_position == ROUTE_POSITION::FIRST) ++t_swap_first;
    if (_input.jobs[t_route[i]].route_position == ROUTE_POSITION::LAST) ++t_swap_last;
  }

  // New route compositions:
  // Source: first_count = s_kept_first + t_swap_first, last_count = s_kept_last + t_swap_last
  // Target: first_count = t_kept_first + s_swap_first, last_count = t_kept_last + s_swap_last
  const auto new_s_first = s_kept_first + t_swap_first;
  const auto new_s_last = s_kept_last + t_swap_last;
  const auto new_t_first = t_kept_first + s_swap_first;
  const auto new_t_last = t_kept_last + s_swap_last;

  // Validate: jobs kept in source [0..s_rank]
  for (Index i = 0; i <= s_rank; ++i) {
    const auto& job = _input.jobs[s_route[i]];
    switch (job.route_position) {
      case ROUTE_POSITION::FIRST:
        if (i >= new_s_first) return false;
        break;
      case ROUTE_POSITION::LAST:
        if (i < new_s_size - new_s_last) return false;
        break;
      case ROUTE_POSITION::NONE:
        if ((new_s_first > 0 && i < new_s_first) ||
            (new_s_last > 0 && i >= new_s_size - new_s_last)) return false;
        break;
    }
  }

  // Validate: jobs from target to source at positions [s_rank+1..new_s_size-1]
  for (Index i = t_rank + 1; i < t_route.size(); ++i) {
    const auto& job = _input.jobs[t_route[i]];
    const auto new_pos = s_rank + 1 + (i - t_rank - 1);
    switch (job.route_position) {
      case ROUTE_POSITION::FIRST:
        if (new_pos >= new_s_first) return false;
        break;
      case ROUTE_POSITION::LAST:
        if (new_pos < new_s_size - new_s_last) return false;
        break;
      case ROUTE_POSITION::NONE:
        if ((new_s_first > 0 && new_pos < new_s_first) ||
            (new_s_last > 0 && new_pos >= new_s_size - new_s_last)) return false;
        break;
    }
  }

  // Validate: jobs kept in target [0..t_rank]
  for (Index i = 0; i <= t_rank; ++i) {
    const auto& job = _input.jobs[t_route[i]];
    switch (job.route_position) {
      case ROUTE_POSITION::FIRST:
        if (i >= new_t_first) return false;
        break;
      case ROUTE_POSITION::LAST:
        if (i < new_t_size - new_t_last) return false;
        break;
      case ROUTE_POSITION::NONE:
        if ((new_t_first > 0 && i < new_t_first) ||
            (new_t_last > 0 && i >= new_t_size - new_t_last)) return false;
        break;
    }
  }

  // Validate: jobs from source to target at positions [t_rank+1..new_t_size-1]
  for (Index i = s_rank + 1; i < s_route.size(); ++i) {
    const auto& job = _input.jobs[s_route[i]];
    const auto new_pos = t_rank + 1 + (i - s_rank - 1);
    switch (job.route_position) {
      case ROUTE_POSITION::FIRST:
        if (new_pos >= new_t_first) return false;
        break;
      case ROUTE_POSITION::LAST:
        if (new_pos < new_t_size - new_t_last) return false;
        break;
      case ROUTE_POSITION::NONE:
        if ((new_t_first > 0 && new_pos < new_t_first) ||
            (new_t_last > 0 && new_pos >= new_t_size - new_t_last)) return false;
        break;
    }
  }

  const auto& t_pickup = target.bwd_pickups(t_rank);

  const auto& s_pickup = source.bwd_pickups(s_rank);

  return is_valid_for_source_range_bounds() &&
         is_valid_for_target_range_bounds() &&
         source.is_valid_addition_for_capacity_margins(_input,
                                                       t_pickup,
                                                       _t_delivery,
                                                       s_rank + 1,
                                                       s_route.size()) &&
         target.is_valid_addition_for_capacity_margins(_input,
                                                       s_pickup,
                                                       _s_delivery,
                                                       t_rank + 1,
                                                       t_route.size()) &&
         source.is_valid_addition_for_capacity_inclusion(_input,
                                                         _t_delivery,
                                                         t_route.begin() +
                                                           t_rank + 1,
                                                         t_route.end(),
                                                         s_rank + 1,
                                                         s_route.size()) &&
         target.is_valid_addition_for_capacity_inclusion(_input,
                                                         _s_delivery,
                                                         s_route.begin() +
                                                           s_rank + 1,
                                                         s_route.end(),
                                                         t_rank + 1,
                                                         t_route.size());
}

void TwoOpt::apply() {
  auto nb_source = s_route.size() - 1 - s_rank;

  t_route.insert(t_route.begin() + t_rank + 1,
                 s_route.begin() + s_rank + 1,
                 s_route.end());
  s_route.erase(s_route.begin() + s_rank + 1, s_route.end());
  s_route.insert(s_route.end(),
                 t_route.begin() + t_rank + 1 + nb_source,
                 t_route.end());
  t_route.erase(t_route.begin() + t_rank + 1 + nb_source, t_route.end());

  source.update_amounts(_input);
  target.update_amounts(_input);
}

std::vector<Index> TwoOpt::addition_candidates() const {
  return {s_vehicle, t_vehicle};
}

std::vector<Index> TwoOpt::update_candidates() const {
  return {s_vehicle, t_vehicle};
}

} // namespace vroom::cvrp
