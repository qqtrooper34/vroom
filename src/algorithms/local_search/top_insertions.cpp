/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "algorithms/local_search/top_insertions.h"
#include "structures/vroom/tw_route.h"
#include "utils/helpers.h"

namespace vroom::ls {

void update_insertions(ThreeInsertions& insertions, InsertionOption&& option) {
  if (option.cost < insertions[2].cost) {
    if (option.cost < insertions[1].cost) {
      if (option.cost < insertions[0].cost) {
        insertions[2] = std::move(insertions[1]);
        insertions[1] = std::move(insertions[0]);
        insertions[0] = std::move(option);
      } else {
        insertions[2] = std::move(insertions[1]);
        insertions[1] = std::move(option);
      }
    } else {
      insertions[2] = std::move(option);
    }
  }
}

template <class Route>
ThreeInsertions find_top_3_insertions(const Input& input,
                                      Index j,
                                      const Route& r) {
  const auto& v = input.vehicles[r.vehicle_rank];
  const auto& job = input.jobs[j];

  // Count existing FIRST and LAST jobs in route
  Index first_count = 0;
  Index last_count = 0;
  for (const auto& job_rank : r.route) {
    const auto& existing_job = input.jobs[job_rank];
    if (existing_job.route_position == ROUTE_POSITION::FIRST) {
      ++first_count;
    } else if (existing_job.route_position == ROUTE_POSITION::LAST) {
      ++last_count;
    }
  }

  const auto route_size = r.route.size();

  auto best_insertions = empty_three_insertions;

  for (Index rank = 0; rank <= route_size; ++rank) {
    // Check route_position constraint
    bool position_valid = true;
    switch (job.route_position) {
      case ROUTE_POSITION::FIRST:
        position_valid = rank <= first_count;
        break;
      case ROUTE_POSITION::LAST:
        if (route_size == 0) {
          position_valid = true; // Allow LAST in empty route
        } else {
          position_valid = rank >= route_size - last_count;
        }
        break;
      case ROUTE_POSITION::NONE:
        position_valid = (first_count == 0 || rank >= first_count) &&
                         (last_count == 0 || rank <= route_size - last_count);
        break;
    }

    if (!position_valid) {
      continue;
    }

    InsertionOption current_insert =
      {utils::addition_cost(input, j, v, r.route, rank), rank};

    update_insertions(best_insertions, std::move(current_insert));
  }

  return best_insertions;
}

template ThreeInsertions find_top_3_insertions(const Input& input,
                                               Index j,
                                               const RawRoute& r);

template ThreeInsertions find_top_3_insertions(const Input& input,
                                               Index j,
                                               const TWRoute& r);

} // namespace vroom::ls
