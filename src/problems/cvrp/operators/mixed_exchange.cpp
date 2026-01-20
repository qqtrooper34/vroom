/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/mixed_exchange.h"

namespace vroom::cvrp {

MixedExchange::MixedExchange(const Input& input,
                             const utils::SolutionState& sol_state,
                             RawRoute& s_route,
                             Index s_vehicle,
                             Index s_rank,
                             RawRoute& t_route,
                             Index t_vehicle,
                             Index t_rank,
                             bool check_t_reverse)
  : Operator(OperatorName::MixedExchange,
             input,
             sol_state,
             s_route,
             s_vehicle,
             s_rank,
             t_route,
             t_vehicle,
             t_rank),
    // Required for consistency in compute_gain if check_t_reverse is
    // false.
    check_t_reverse(check_t_reverse),
    source_delivery(_input.jobs[this->s_route[s_rank]].delivery),
    target_delivery(_input.jobs[this->t_route[t_rank]].delivery +
                    _input.jobs[this->t_route[t_rank + 1]].delivery) {
  assert(s_vehicle != t_vehicle);
  assert(s_route.size() >= 1);
  assert(t_route.size() >= 2);
  assert(s_rank < s_route.size());
  assert(t_rank < t_route.size() - 1);

  assert(_input.vehicle_ok_with_job(t_vehicle, this->s_route[s_rank]));
  assert(_input.vehicle_ok_with_job(s_vehicle, this->t_route[t_rank]));
  assert(_input.vehicle_ok_with_job(s_vehicle, this->t_route[t_rank + 1]));

  // Either moving edge with single jobs or a whole shipment.
  assert((_input.jobs[this->t_route[t_rank]].type == JOB_TYPE::SINGLE &&
          _input.jobs[this->t_route[t_rank + 1]].type == JOB_TYPE::SINGLE &&
          check_t_reverse) ||
         (_input.jobs[this->t_route[t_rank]].type == JOB_TYPE::PICKUP &&
          _input.jobs[this->t_route[t_rank + 1]].type == JOB_TYPE::DELIVERY &&
          !check_t_reverse &&
          _sol_state.matching_delivery_rank[t_vehicle][t_rank] == t_rank + 1));
}

Eval MixedExchange::gain_upper_bound() {
  const auto& s_v = _input.vehicles[s_vehicle];
  const auto& t_v = _input.vehicles[t_vehicle];

  // For source vehicle, we consider the cost of replacing job at rank
  // s_rank with target edge. Part of that cost (for adjacent edges)
  // is stored in _sol_state.edge_evals_around_node. reverse_t_edge
  // checks whether we should change the target edge order.
  Index s_index = _input.jobs[s_route[s_rank]].index();
  Index t_index = _input.jobs[t_route[t_rank]].index();
  Index t_after_index = _input.jobs[t_route[t_rank + 1]].index();

  // Determine costs added with target edge.
  Eval previous_cost;
  Eval next_cost;
  Eval reverse_previous_cost;
  Eval reverse_next_cost;

  if (s_rank == 0) {
    if (s_v.has_start()) {
      auto p_index = s_v.start.value().index();
      previous_cost = s_v.eval(p_index, t_index);
      reverse_previous_cost = s_v.eval(p_index, t_after_index);
    }
  } else {
    auto p_index = _input.jobs[s_route[s_rank - 1]].index();
    previous_cost = s_v.eval(p_index, t_index);
    reverse_previous_cost = s_v.eval(p_index, t_after_index);
  }

  if (s_rank == s_route.size() - 1) {
    if (s_v.has_end()) {
      auto n_index = s_v.end.value().index();
      next_cost = s_v.eval(t_after_index, n_index);
      reverse_next_cost = s_v.eval(t_index, n_index);
    }
  } else {
    auto n_index = _input.jobs[s_route[s_rank + 1]].index();
    next_cost = s_v.eval(t_after_index, n_index);
    reverse_next_cost = s_v.eval(t_index, n_index);
  }

  _normal_s_gain = _sol_state.edge_evals_around_node[s_vehicle][s_rank] -
                   previous_cost - next_cost - s_v.eval(t_index, t_after_index);

  auto s_gain_upper_bound = _normal_s_gain;

  if (check_t_reverse) {
    _reversed_s_gain = _sol_state.edge_evals_around_node[s_vehicle][s_rank] -
                       reverse_previous_cost - reverse_next_cost -
                       s_v.eval(t_after_index, t_index);

    s_gain_upper_bound = std::max(_normal_s_gain, _reversed_s_gain);
  }

  // For target vehicle, we consider the cost of replacing edge at
  // rank t_rank with source job. Part of that cost (for adjacent
  // edges) is stored in _sol_state.edge_evals_around_edges.

  // Determine costs added with source job.
  previous_cost = Eval();
  next_cost = Eval();

  if (t_rank == 0) {
    if (t_v.has_start()) {
      auto p_index = t_v.start.value().index();
      previous_cost = t_v.eval(p_index, s_index);
    }
  } else {
    auto p_index = _input.jobs[t_route[t_rank - 1]].index();
    previous_cost = t_v.eval(p_index, s_index);
  }

  if (t_rank == t_route.size() - 2) {
    if (t_v.has_end()) {
      auto n_index = t_v.end.value().index();
      next_cost = t_v.eval(s_index, n_index);
    }
  } else {
    auto n_index = _input.jobs[t_route[t_rank + 2]].index();
    next_cost = t_v.eval(s_index, n_index);
  }

  t_gain = _sol_state.edge_evals_around_edge[t_vehicle][t_rank] +
           t_v.eval(t_index, t_after_index) - previous_cost - next_cost;

  _gain_upper_bound_computed = true;

  return s_gain_upper_bound + t_gain;
}

void MixedExchange::compute_gain() {
  assert(_gain_upper_bound_computed);
  assert(s_is_normal_valid || s_is_reverse_valid);
  if (_normal_s_gain < _reversed_s_gain) {
    // Biggest potential gain is obtained when reversing edge.
    if (s_is_reverse_valid) {
      stored_gain += _reversed_s_gain;
      reverse_t_edge = true;
    } else {
      stored_gain += _normal_s_gain;
    }
  } else {
    // Biggest potential gain is obtained when not reversing edge.
    if (s_is_normal_valid) {
      stored_gain += _normal_s_gain;
    } else {
      stored_gain += _reversed_s_gain;
      reverse_t_edge = true;
    }
  }

  stored_gain += t_gain;

  gain_computed = true;
}

bool MixedExchange::is_valid() {
  assert(_gain_upper_bound_computed);

  // Check route_position constraints for mixed exchange
  const auto s_first_count = _sol_state.first_jobs_count[s_vehicle];
  const auto s_last_count = _sol_state.last_jobs_count[s_vehicle];
  const auto t_first_count = _sol_state.first_jobs_count[t_vehicle];
  const auto t_last_count = _sol_state.last_jobs_count[t_vehicle];
  // After exchange: source loses 1 job, gains 2 -> net +1
  const auto new_s_route_size = s_route.size() + 1;
  const auto new_t_route_size = t_route.size() - 1;

  // Helper to check position validity
  auto is_valid_position = [&](const Job& job, Index new_rank, Index first_c, Index last_c, Index route_sz) {
    switch (job.route_position) {
      case ROUTE_POSITION::FIRST:
        return new_rank < first_c;
      case ROUTE_POSITION::LAST:
        return new_rank >= route_sz - last_c;
      case ROUTE_POSITION::NONE:
        return (first_c == 0 || new_rank >= first_c) &&
               (last_c == 0 || new_rank < route_sz - last_c);
    }
    return true;
  };

  // Check if s_route[s_rank] can go to t_route at t_rank
  const auto& s_job = _input.jobs[s_route[s_rank]];
  bool t_pos_valid = is_valid_position(s_job, t_rank, t_first_count, t_last_count, new_t_route_size);

  // Check if t_route jobs can go to s_route at s_rank, s_rank+1
  const auto& t_job1 = _input.jobs[t_route[t_rank]];
  const auto& t_job2 = _input.jobs[t_route[t_rank + 1]];
  bool s_normal_pos_valid = is_valid_position(t_job1, s_rank, s_first_count, s_last_count, new_s_route_size) &&
                            is_valid_position(t_job2, s_rank + 1, s_first_count, s_last_count, new_s_route_size);
  bool s_reverse_pos_valid = is_valid_position(t_job2, s_rank, s_first_count, s_last_count, new_s_route_size) &&
                             is_valid_position(t_job1, s_rank + 1, s_first_count, s_last_count, new_s_route_size);

  if (!t_pos_valid || !(s_normal_pos_valid || s_reverse_pos_valid)) {
    return false;
  }

  bool valid =
    is_valid_for_target_range_bounds() &&
    target.is_valid_addition_for_capacity_margins(_input,
                                                  _input.jobs[s_route[s_rank]]
                                                    .pickup,
                                                  source_delivery,
                                                  t_rank,
                                                  t_rank + 2);

  auto target_pickup = _input.jobs[t_route[t_rank]].pickup +
                       _input.jobs[t_route[t_rank + 1]].pickup;
  valid =
    valid && source.is_valid_addition_for_capacity_margins(_input,
                                                           target_pickup,
                                                           target_delivery,
                                                           s_rank,
                                                           s_rank + 1);

  if (valid) {
    // Keep target edge direction when inserting in source route.
    auto t_start = t_route.begin() + t_rank;

    const auto& s_v = _input.vehicles[s_vehicle];
    const auto s_eval = _sol_state.route_evals[s_vehicle];

    s_is_normal_valid =
      s_v.ok_for_range_bounds(s_eval - _normal_s_gain) &&
      source.is_valid_addition_for_capacity_inclusion(_input,
                                                      target_delivery,
                                                      t_start,
                                                      t_start + 2,
                                                      s_rank,
                                                      s_rank + 1);
    if (check_t_reverse) {
      // Reverse target edge direction when inserting in source route.
      auto t_reverse_start = t_route.rbegin() + t_route.size() - 2 - t_rank;
      s_is_reverse_valid =
        s_v.ok_for_range_bounds(s_eval - _reversed_s_gain) &&
        source.is_valid_addition_for_capacity_inclusion(_input,
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

  std::swap(s_route[s_rank], t_route[t_rank]);
  s_route.insert(s_route.begin() + s_rank + 1,
                 t_route.begin() + t_rank + 1,
                 t_route.begin() + t_rank + 2);
  t_route.erase(t_route.begin() + t_rank + 1);

  if (reverse_t_edge) {
    std::swap(s_route[s_rank], s_route[s_rank + 1]);
  }

  source.update_amounts(_input);
  target.update_amounts(_input);
}

std::vector<Index> MixedExchange::addition_candidates() const {
  return {s_vehicle, t_vehicle};
}

std::vector<Index> MixedExchange::update_candidates() const {
  return {s_vehicle, t_vehicle};
}

} // namespace vroom::cvrp
