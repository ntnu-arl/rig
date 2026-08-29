#pragma once

#include <chrono>

namespace rig
{
class StopWatch
{
public:
  typedef std::chrono::steady_clock Clock;

public:
  StopWatch() : last_(Clock::now()) {}
  ~StopWatch() {}

  float elapsed() const
  {
    std::chrono::duration<float> duration = Clock::now() - last_;
    return duration.count() * 1000.0f;
  }

  void reset() { last_ = Clock::now(); }

private:
  Clock::time_point last_;
};
}  // namespace rig
