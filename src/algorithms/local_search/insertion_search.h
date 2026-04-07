#ifndef INSERTION_SEARCH_H
#define INSERTION_SEARCH_H

#include "structures/typedefs.h"
#include "structures/vroom/solution_state.h"
#include "utils/helpers.h"

namespace vroom::ls {

// Helper function to validate rank for route_position constraint.
inline bool is_valid_rank_for_route_position(const Input& input,
                                              const utils::SolutionState& sol_state,
                                              Index j,
                                              Index v,
                                              Index rank,
                                              std::size_t route_size) {
  const auto& job = input.jobs[j];
  const auto first_count = sol_state.first_jobs_count[v];
  const auto last_count = sol_state.last_jobs_count[v];

  switch (job.route_position) {
    case ROUTE_POSITION::FIRST:
      // FIRST jobs must be in the first positions (rank <= first_count)
      return rank <= first_count;
    case ROUTE_POSITION::LAST:
      // Allow LAST jobs in empty route (all-LAST scenario)
      if (route_size == 0) {
        return true;
      }
      // LAST jobs must be in the last positions (rank >= route_size - last_count)
      return rank >= route_size - last_count;
    case ROUTE_POSITION::NONE:
      // Normal jobs cannot be placed in FIRST or LAST zones
      return (first_count == 0 || rank >= first_count) &&
             (last_count == 0 || rank <= route_size - last_count);
  }
  return true;
}

struct RouteInsertion {
  Eval eval{NO_EVAL};
  Amount delivery;
  Index single_rank{0};
  Index pickup_rank{0};
  Index delivery_rank{0};

  explicit RouteInsertion(unsigned amount_size)
    : delivery(Amount(amount_size)) {
  }
};

template <class Route>
RouteInsertion
compute_best_insertion_single(const Input& input,
                              const utils::SolutionState& sol_state,
                              const Index j,
                              Index v,
                              const Route& route) {
  RouteInsertion result(input.get_amount_size());
  const auto& current_job = input.jobs[j];
  const auto& v_target = input.vehicles[v];

  if (input.vehicle_ok_with_job(v, j)) {
    // Determine loop bounds, adjusting for route_position constraints
    Index rank_begin = sol_state.insertion_ranks_begin[v][j];
    Index rank_end = sol_state.insertion_ranks_end[v][j];

    // For FIRST jobs, must start from rank 0 to ensure we try the first position
    if (current_job.route_position == ROUTE_POSITION::FIRST) {
      rank_begin = 0;
      // FIRST jobs can only go in first positions, so limit end
      const Index first_limit = static_cast<Index>(sol_state.first_jobs_count[v] + 1);
      rank_end = std::min(rank_end, first_limit);
    }
    // For LAST jobs, must try positions at the end
    if (current_job.route_position == ROUTE_POSITION::LAST) {
      const auto route_size = static_cast<Index>(route.size());
      // LAST jobs can only go in last positions
      if (route_size > 0) {
        const Index last_begin = static_cast<Index>(route_size - sol_state.last_jobs_count[v]);
        rank_begin = std::max(rank_begin, last_begin);
      }
      rank_end = static_cast<Index>(route_size + 1);
    }

    for (Index rank = rank_begin; rank < rank_end; ++rank) {
      // Check route_position constraint
      if (!is_valid_rank_for_route_position(input, sol_state, j, v, rank, route.size())) {
        continue;
      }

      Eval current_eval =
        utils::addition_cost(input, j, v_target, route.route, rank);
      if (current_eval.cost < result.eval.cost &&
          v_target.ok_for_range_bounds(sol_state.route_evals[v] +
                                       current_eval) &&
          route.is_valid_addition_for_capacity(input,
                                               current_job.pickup,
                                               current_job.delivery,
                                               rank) &&
          route.is_valid_addition_for_tw(input, j, rank)) {
        result.eval = current_eval;
        result.delivery = current_job.delivery;
        result.single_rank = rank;
      }
    }
  }
  return result;
}

template <class Route, std::forward_iterator Iter>
bool valid_for_capacity(const Input& input,
                        const Route& r,
                        Iter start,
                        Iter end,
                        Index pickup_r,
                        Index delivery_r) {
  Amount amount = input.zero_amount();

  for (auto it = start + 1; it != end - 1; ++it) {
    const auto& new_modified_job = input.jobs[*it];
    if (new_modified_job.type == JOB_TYPE::SINGLE) {
      amount += new_modified_job.delivery;
    }
  }

  return r.is_valid_addition_for_capacity_inclusion(input,
                                                    std::move(amount),
                                                    start,
                                                    end,
                                                    pickup_r,
                                                    delivery_r);
}

template <class Route>
RouteInsertion compute_best_insertion_pd(const Input& input,
                                         const utils::SolutionState& sol_state,
                                         const Index j,
                                         Index v,
                                         const Route& route,
                                         const Eval& cost_threshold) {
  RouteInsertion result(input.get_amount_size());
  const auto& current_job = input.jobs[j];
  const auto& v_target = input.vehicles[v];

  if (!input.vehicle_ok_with_job(v, j)) {
    return result;
  }

  result.eval = cost_threshold;

  // Pre-compute cost of addition for matching delivery.
  std::vector<Eval> d_adds(route.size() + 1);
  std::vector<unsigned char> valid_delivery_insertions(route.size() + 1, false);

  const auto begin_d_rank = sol_state.insertion_ranks_begin[v][j + 1];
  const auto end_d_rank = sol_state.insertion_ranks_end[v][j + 1];

  bool found_valid = false;
  for (unsigned d_rank = begin_d_rank; d_rank < end_d_rank; ++d_rank) {
    // Check route_position constraint for shipment delivery insertion
    if (!is_valid_rank_for_route_position(input, sol_state, j + 1, v, d_rank, route.size())) {
      valid_delivery_insertions[d_rank] = false;
      continue;
    }
    d_adds[d_rank] =
      utils::addition_cost(input, j + 1, v_target, route.route, d_rank);
    if (result.eval < d_adds[d_rank]) {
      valid_delivery_insertions[d_rank] = false;
    } else {
      valid_delivery_insertions[d_rank] =
        route.is_valid_addition_for_tw_without_max_load(input, j + 1, d_rank);
    }
    found_valid |= valid_delivery_insertions[d_rank];
  }

  if (!found_valid) {
    result.eval = NO_EVAL;
    return result;
  }

  for (Index pickup_r = sol_state.insertion_ranks_begin[v][j];
       pickup_r < sol_state.insertion_ranks_end[v][j];
       ++pickup_r) {
    // Check route_position constraint for shipment pickup insertion
    if (!is_valid_rank_for_route_position(input, sol_state, j, v, pickup_r, route.size())) {
      continue;
    }

    Eval p_add =
      utils::addition_cost(input, j, v_target, route.route, pickup_r);
    if (result.eval < p_add) {
      // Even without delivery insertion more expensive than current best.
      continue;
    }

    if (!route.is_valid_addition_for_load(input,
                                          current_job.pickup,
                                          pickup_r) ||
        !route.is_valid_addition_for_tw_without_max_load(input, j, pickup_r)) {
      continue;
    }

    // Build replacement sequence for current insertion.
    std::vector<Index> modified_with_pd;
    if (pickup_r <= end_d_rank) {
      modified_with_pd.reserve(end_d_rank - pickup_r + 2);
    }
    modified_with_pd.push_back(j);

    Amount modified_delivery = input.zero_amount();

    // No need to use begin_d_rank here thanks to
    // valid_delivery_insertions values.
    for (Index delivery_r = pickup_r; delivery_r < end_d_rank; ++delivery_r) {
      // Update state variables along the way before potential
      // early abort.
      if (pickup_r < delivery_r) {
        modified_with_pd.push_back(route.route[delivery_r - 1]);
        const auto& new_modified_job = input.jobs[route.route[delivery_r - 1]];
        if (new_modified_job.type == JOB_TYPE::SINGLE) {
          modified_delivery += new_modified_job.delivery;
        }
      }

      if (!static_cast<bool>(valid_delivery_insertions[delivery_r])) {
        continue;
      }

      Eval pd_eval;
      if (pickup_r == delivery_r) {
        pd_eval = utils::addition_cost(input,
                                       j,
                                       v_target,
                                       route.route,
                                       pickup_r,
                                       pickup_r + 1);
      } else {
        pd_eval = p_add + d_adds[delivery_r];
      }

      if (pd_eval < result.eval &&
          v_target.ok_for_range_bounds(sol_state.route_evals[v] + pd_eval)) {
        modified_with_pd.push_back(j + 1);

        // Update best cost depending on validity.
        bool is_valid = valid_for_capacity(input,
                                           route,
                                           modified_with_pd.begin(),
                                           modified_with_pd.end(),
                                           pickup_r,
                                           delivery_r);

        is_valid =
          is_valid && route.is_valid_addition_for_tw(input,
                                                     modified_delivery,
                                                     modified_with_pd.begin(),
                                                     modified_with_pd.end(),
                                                     pickup_r,
                                                     delivery_r);

        modified_with_pd.pop_back();

        if (is_valid) {
          result.eval = pd_eval;
          result.delivery = modified_delivery;
          result.pickup_rank = pickup_r;
          result.delivery_rank = delivery_r;
        }
      }
    }
  }

  assert(result.eval <= cost_threshold);
  if (result.eval == cost_threshold) {
    result.eval = NO_EVAL;
  }
  return result;
}

} // namespace vroom::ls
#endif
