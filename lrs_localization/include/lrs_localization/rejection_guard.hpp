// Protection against rejecting a sensor forever (A2 §3) -- no ROS, unit tested.
//
// The innovation gate assumes the filter is right and asks whether the measurement fits. If the
// filter itself is wrong (diverged, overconfident after a long outage, or the sensor jumped for
// good), every following measurement fails the gate and the filter would never recover.
//
//   normal      a measurement above the gate is rejected; `max_rejections` rejections IN A ROW
//               mean the sensor agrees with itself but not with the filter -> recovering
//   recovering  nothing is rejected: a measurement above the gate is applied with R inflated so
//               that NIS = gate (Ekf8::update with Gate::inflate), i.e. a bounded step towards
//               it, never a jump onto it. Back to normal after `max_rejections` measurements in
//               a row pass the gate on their own.
//
// A single outlier is one rejection, and the next good measurement resets the count, so it can
// never trigger recovery. max_rejections <= 0 disables recovery (reject forever).
#pragma once

#include <cstddef>

namespace lrs_localization
{

class RejectionGuard
{
public:
  enum class Event {kNone, kEnterRecovery, kLeaveRecovery};

  explicit RejectionGuard(int max_rejections = 10) : n_(max_rejections) {}

  // Use Gate::inflate for the next update?
  bool recovering() const {return recovering_;}
  int rejectedInRow() const {return rejected_in_row_;}
  std::size_t inflatedThisEpisode() const {return inflated_;}

  // Report what happened to one gated update.
  Event onRejected()
  {
    ++rejected_in_row_;
    if (n_ > 0 && !recovering_ && rejected_in_row_ >= n_) {
      recovering_ = true;
      inflated_ = 0;
      passed_in_row_ = 0;
      return Event::kEnterRecovery;
    }
    return Event::kNone;
  }

  Event onApplied(bool inflated)
  {
    if (inflated) {
      ++inflated_;
      passed_in_row_ = 0;
      return Event::kNone;
    }
    rejected_in_row_ = 0;
    if (recovering_ && ++passed_in_row_ >= n_) {
      recovering_ = false;
      return Event::kLeaveRecovery;
    }
    return Event::kNone;
  }

  void reset() {*this = RejectionGuard(n_);}

private:
  int n_;
  bool recovering_{false};
  int rejected_in_row_{0};
  int passed_in_row_{0};
  std::size_t inflated_{0};
};

}  // namespace lrs_localization
