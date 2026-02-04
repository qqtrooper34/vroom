/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <ctime>
#include <iomanip>
#include <sstream>

#include "structures/vroom/time_window.h"
#include "utils/exception.h"

namespace {
std::string unix_to_local(uint32_t ts) {
  std::time_t t = static_cast<std::time_t>(ts);
  std::tm tm_buf;
  localtime_r(&t, &tm_buf);
  std::ostringstream oss;
  oss << std::put_time(&tm_buf, "%Y-%m-%d %H:%M:%S");
  return oss.str();
}
} // namespace

namespace vroom {

constexpr Duration TimeWindow::default_length =
  utils::scale_from_user_duration(std::numeric_limits<UserDuration>::max());

TimeWindow::TimeWindow()
  : start(0),
    end(utils::scale_from_user_duration(
      std::numeric_limits<UserDuration>::max())),
    length(end - start) {
}

TimeWindow::TimeWindow(UserDuration start, UserDuration end)
  : start(utils::scale_from_user_duration(start)),
    end(utils::scale_from_user_duration(end)),
    length(utils::scale_from_user_duration(end - start)) {
  if (start > end) {
    // TAMS: помимо unix-timestamp показываем читаемое local time
    throw InputException(
      std::format("Invalid time window: [{} ({}), {} ({})]",
                  unix_to_local(start),
                  start,
                  unix_to_local(end),
                  end));
  }
}

bool TimeWindow::contains(Duration time) const {
  return (start <= time) && (time <= end);
}

bool TimeWindow::is_default() const {
  return end - start == default_length;
}

bool operator<(const TimeWindow& lhs, const TimeWindow& rhs) {
  return lhs.start < rhs.start || (lhs.start == rhs.start && lhs.end < rhs.end);
}

} // namespace vroom
