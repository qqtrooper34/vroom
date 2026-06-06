/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/unassigned_exchange.h"
#include "utils/helpers.h"

namespace vroom::cvrp {

UnassignedExchange::UnassignedExchange(const Input& input,
                                       const utils::SolutionState& sol_state,
                                       std::unordered_set<Index>& unassigned,
                                       RawRoute& s_raw_route,
                                       Index s_vehicle,
                                       Index s_rank,
                                       Index t_rank,
                                       Index u)
  : Operator(OperatorName::UnassignedExchange,
             input,
             sol_state,
             s_raw_route,
             s_vehicle,
             s_rank,
             s_raw_route,
             s_vehicle,
             t_rank),
    _u(u),
    _unassigned(unassigned),
    _first_rank(std::min(s_rank, t_rank)),
    _last_rank((s_rank < t_rank) ? t_rank : s_rank + 1),
    _moved_jobs(_last_rank - _first_rank),
    _removed(s_route[s_rank]),
    _delivery(source.delivery_in_range(_first_rank, _last_rank)) {
  assert(t_rank != s_rank + 1);
  assert(!s_route.empty());
  assert(s_rank < s_route.size());
  assert(t_rank <= s_route.size());

  assert(_input.jobs[_removed].delivery <= _delivery);
  _delivery -= _input.jobs[_removed].delivery;
  _delivery += _input.jobs[_u].delivery;

  if (s_rank < t_rank) {
    std::copy(s_route.begin() + s_rank + 1,
              s_route.begin() + t_rank,
              _moved_jobs.begin());
    _moved_jobs.back() = u;
  } else {
    std::copy(s_route.begin() + t_rank,
              s_route.begin() + s_rank,
              _moved_jobs.begin() + 1);
    _moved_jobs.front() = u;
  }
}

void UnassignedExchange::compute_gain() {
  const auto& v = _input.vehicles[s_vehicle];

  const Index u_index = _input.jobs[_u].index();

  if (t_rank == s_rank) {
    // Removed job is replaced by the unassigned one so there is no
    // new edge in place of removal.
    s_gain = _sol_state.edge_evals_around_node[s_vehicle][s_rank];

    // No old edge to remove when adding unassigned job in place of
    // removed job.
    if (t_rank == 0) {
      if (v.has_start()) {
        s_gain -= v.eval(v.start.value().index(), u_index);
      }
    } else {
      s_gain -= v.eval(_input.jobs[s_route[t_rank - 1]].index(), u_index);
    }

    if (t_rank == s_route.size() - 1) {
      if (v.has_end()) {
        s_gain -= v.eval(u_index, v.end.value().index());
      }
    } else {
      s_gain -= v.eval(u_index, _input.jobs[s_route[s_rank + 1]].index());
    }
    // TAMS: вычитаем service добавляемого job (edge_evals_around_node уже содержит service удаляемого)
    s_gain.service -= _input.jobs[_u].services[v.type];
  } else {
    // No common edge so both gains can be computed independently.
    s_gain = _sol_state.node_gains[s_vehicle][s_rank] -
             utils::addition_cost(_input, _u, v, s_route, t_rank);
  }

  stored_gain = s_gain;
  gain_computed = true;
}

bool UnassignedExchange::is_valid() {
  // Check route_position constraints
  // This operator removes job at s_rank and inserts _u at t_rank

  const auto& removed_job = _input.jobs[_removed];
  const auto& inserted_job = _input.jobs[_u];

  // Calculate new first/last counts after operation
  auto first_count = _sol_state.first_jobs_count[s_vehicle];
  auto last_count = _sol_state.last_jobs_count[s_vehicle];

  if (removed_job.route_position == ROUTE_POSITION::FIRST) --first_count;
  if (removed_job.route_position == ROUTE_POSITION::LAST) --last_count;
  if (inserted_job.route_position == ROUTE_POSITION::FIRST) ++first_count;
  if (inserted_job.route_position == ROUTE_POSITION::LAST) ++last_count;

  const auto route_size = s_route.size();  // size unchanged (remove one, add one)

  // Check if inserted job's position is valid
  bool position_valid = true;
  switch (inserted_job.route_position) {
    case ROUTE_POSITION::FIRST:
      position_valid = t_rank < first_count;
      break;
    case ROUTE_POSITION::LAST:
      position_valid = t_rank >= route_size - last_count;
      break;
    case ROUTE_POSITION::NONE:
      position_valid = (first_count == 0 || t_rank >= first_count) &&
                       (last_count == 0 || t_rank < route_size - last_count);
      break;
  }

  if (!position_valid) {
    return false;
  }

  // Check that existing jobs remain in valid positions after the exchange
  // Jobs in _moved_jobs will be at positions [_first_rank.._last_rank-1]
  for (Index i = 0; i < _moved_jobs.size(); ++i) {
    const auto& job = _input.jobs[_moved_jobs[i]];
    const auto new_pos = _first_rank + i;
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

  // Check jobs outside the moved range
  for (Index i = 0; i < _first_rank; ++i) {
    const auto& job = _input.jobs[s_route[i]];
    switch (job.route_position) {
      case ROUTE_POSITION::FIRST:
        if (i >= first_count) return false;
        break;
      case ROUTE_POSITION::LAST:
        if (i < route_size - last_count) return false;
        break;
      case ROUTE_POSITION::NONE:
        if ((first_count > 0 && i < first_count) ||
            (last_count > 0 && i >= route_size - last_count)) return false;
        break;
    }
  }

  for (Index i = _last_rank; i < route_size; ++i) {
    const auto& job = _input.jobs[s_route[i]];
    switch (job.route_position) {
      case ROUTE_POSITION::FIRST:
        if (i >= first_count) return false;
        break;
      case ROUTE_POSITION::LAST:
        if (i < route_size - last_count) return false;
        break;
      case ROUTE_POSITION::NONE:
        if ((first_count > 0 && i < first_count) ||
            (last_count > 0 && i >= route_size - last_count)) return false;
        break;
    }
  }

  auto pickup = source.pickup_in_range(_first_rank, _last_rank);
  assert(_input.jobs[_removed].pickup <= pickup);
  pickup -= _input.jobs[_removed].pickup;
  pickup += _input.jobs[_u].pickup;

  bool valid = source.is_valid_addition_for_capacity_margins(_input,
                                                             pickup,
                                                             _delivery,
                                                             _first_rank,
                                                             _last_rank);

  valid = valid &&
          source.is_valid_addition_for_capacity_inclusion(_input,
                                                          _delivery,
                                                          _moved_jobs.begin(),
                                                          _moved_jobs.end(),
                                                          _first_rank,
                                                          _last_rank);

  if (valid) {
    // Check validity with regard to vehicle range bounds, requires
    // valid gain value.
    if (!gain_computed) {
      // We don't check gain before validity if priority is strictly
      // improved.
      this->compute_gain();
    }

    valid = is_valid_for_source_range_bounds();
  }

  return valid;
}

void UnassignedExchange::apply() {
  std::ranges::copy(_moved_jobs, s_route.begin() + _first_rank);

  assert(_unassigned.contains(_u));
  _unassigned.erase(_u);
  assert(!_unassigned.contains(_removed));
  _unassigned.insert(_removed);

  source.update_amounts(_input);
}

std::vector<Index> UnassignedExchange::addition_candidates() const {
  return {s_vehicle};
}

std::vector<Index> UnassignedExchange::update_candidates() const {
  return {s_vehicle};
}

std::vector<Index> UnassignedExchange::required_unassigned() const {
  return {_u};
}

} // namespace vroom::cvrp
