#ifndef EVAL_H
#define EVAL_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <tuple>

#include "structures/typedefs.h"

namespace vroom {

struct Eval {
  Cost cost;
  Duration duration;
  Distance distance;
  Duration service;  // TAMS: service time для max_work_time

  constexpr Eval() : cost(0), duration(0), distance(0), service(0){};

  constexpr Eval(Cost cost, Duration duration = 0, Distance distance = 0, Duration service = 0)
    : cost(cost), duration(duration), distance(distance), service(service){};

  Eval& operator+=(const Eval& rhs) {
    cost += rhs.cost;
    duration += rhs.duration;
    distance += rhs.distance;
    service += rhs.service;  // TAMS

    return *this;
  }

  Eval& operator-=(const Eval& rhs) {
    cost -= rhs.cost;
    duration -= rhs.duration;
    distance -= rhs.distance;
    service -= rhs.service;  // TAMS

    return *this;
  }

  Eval operator-() const {
    return {-cost, -duration, -distance, -service};  // TAMS
  }

  friend Eval operator+(Eval lhs, const Eval& rhs) {
    lhs += rhs;
    return lhs;
  }

  friend Eval operator-(Eval lhs, const Eval& rhs) {
    lhs -= rhs;
    return lhs;
  }

  friend bool operator<(const Eval& lhs, const Eval& rhs) {
    return std::tie(lhs.cost, lhs.duration, lhs.distance, lhs.service) <
           std::tie(rhs.cost, rhs.duration, rhs.distance, rhs.service);  // TAMS
  }

  friend bool operator<=(const Eval& lhs, const Eval& rhs) {
    return lhs.cost <= rhs.cost;
  }

  friend bool operator==(const Eval& lhs, const Eval& rhs) = default;
};

constexpr Eval MAX_EVAL = {std::numeric_limits<Cost>::max(),
                           std::numeric_limits<Duration>::max(),
                           std::numeric_limits<Distance>::max(),
                           std::numeric_limits<Duration>::max()};  // TAMS: +service
constexpr Eval NO_EVAL = {std::numeric_limits<Cost>::max(), 0, 0, 0};  // TAMS: +service
constexpr Eval NO_GAIN = {std::numeric_limits<Cost>::min(), 0, 0, 0};  // TAMS: +service

} // namespace vroom

#endif
