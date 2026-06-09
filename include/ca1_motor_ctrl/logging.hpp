// Copyright 2026 Benjamin Woerfel
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <rcutils/logging.h>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <ctime>

namespace ca1_motor_ctrl
{

inline void custom_log_handler(
  const rcutils_log_location_t *,
  int severity,
  const char *,
  rcutils_time_point_value_t,
  const char * format,
  va_list * args)
{
  char msg[4096];
  vsnprintf(msg, sizeof(msg), format, *args);

  auto now = std::chrono::system_clock::now();
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    now.time_since_epoch()) % 1000;
  std::time_t t = std::chrono::system_clock::to_time_t(now);
  struct tm tm_buf {};
  localtime_r(&t, &tm_buf);

  char ts[28];
  snprintf(
    ts, sizeof(ts), "%04d-%02d-%02d %02d:%02d:%02d:%03d",
    tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday,
    tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec,
    static_cast<int>(ms.count()));

  static const bool color = [] {
      const char * e = std::getenv("RCUTILS_COLORIZED_OUTPUT");
      return e && e[0] == '1' && e[1] == '\0';
    }();

  const char * on = "", * off = "", * label;
  switch (severity) {
    case RCUTILS_LOG_SEVERITY_DEBUG:
      label = "DEBUG"; if (color) {on = "\033[36m";   off = "\033[0m";} break;
    case RCUTILS_LOG_SEVERITY_INFO:
      label = "INFO "; break;
    case RCUTILS_LOG_SEVERITY_WARN:
      label = "WARN "; if (color) {on = "\033[33m";   off = "\033[0m";} break;
    case RCUTILS_LOG_SEVERITY_ERROR:
      label = "ERROR"; if (color) {on = "\033[31m";   off = "\033[0m";} break;
    case RCUTILS_LOG_SEVERITY_FATAL:
      label = "FATAL"; if (color) {on = "\033[1;31m"; off = "\033[0m";} break;
    default:
      label = "?????"; break;
  }

  // Fixed-width prefix: [XXXXX] [YYYY-MM-DD HH:MM:SS:mmm]:\t
  fprintf(stderr, "%s[%s]%s [%s]:\t%s\n", on, label, off, ts, msg);
}

inline void install_log_handler()
{
  rcutils_logging_set_output_handler(custom_log_handler);
}

}  // namespace ca1_motor_ctrl
