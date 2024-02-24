#pragma once

#include <Arduino.h>
#include <optional>

template <typename T>
class Tween {
    T start;
    T end;
    uint32_t duration;
    uint32_t start_time;
    uint32_t step_time;
    uint32_t last_step = 0;
public:
  Tween(T start, T end, uint32_t duration, uint32_t now, uint32_t step_time) : start(start), end(end), duration(duration), start_time(now), step_time(step_time) {}

  std::optional<T> perform_step(uint32_t now) {
    if (now - last_step > step_time) {
      last_step = now;
      if (is_done(now)) {
        return end;
      }
      return get_value(now);
    }
    return {};
  }

  T get_value(uint32_t now) {
    uint32_t elapsed = now - start_time;
    if (elapsed > duration) {
      return end;
    }
    if (duration == 0) {
      return end;
    }
    return start + (end - start) * (elapsed / duration);
  }

  bool is_done(uint32_t now) {
    return now - start_time > duration;
  }
};