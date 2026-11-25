// Bookkeeping for one estimator input (A2 §1).
//
// Every GPS / VO / IMU message goes through SensorChannel::check() before it may reach the
// filter. The check uses the message's own header stamp (simulation time), never the arrival
// time, and drops what a filter must not see:
//   - a zero stamp (publisher not on /clock yet),
//   - a duplicate stamp (same sample twice),
//   - an out-of-order stamp (older than the last accepted one).
// A stamp that jumps back by more than `reset_jump_s` is not an out-of-order sample but a
// simulation reset (Gazebo world reset); it is reported as kReset so the caller can restart.
//
// The channel also collects per-window statistics (count, sim-time rate, largest gap) that the
// node prints periodically, and remembers whether the frame ids matched the sensor contract.
//
// Timeout (A2 §3): checkTimeout(now) is called with the newest header stamp of ANY sensor --
// the simulation time as the sensors see it -- so an outage is measured in stamps, not wall
// time. It returns true exactly once when the channel goes silent for longer than its timeout;
// the next accepted sample ends the outage and takeRecoveredGap() then returns its length.
#pragma once

#include <algorithm>
#include <cstddef>
#include <string>

namespace lrs_localization
{

enum class StampCheck
{
  kOk,
  kZero,
  kDuplicate,
  kOutOfOrder,
  kReset,
};

struct WindowStats
{
  std::size_t count{0};
  double rate_hz{0.0};     // from header stamps: (count - 1) / (last - first)
  double max_gap_s{0.0};   // largest stamp difference between consecutive accepted samples
};

class SensorChannel
{
public:
  SensorChannel(std::string name, std::string frame, std::string child_frame = "")
  : name_(std::move(name)), frame_(std::move(frame)), child_frame_(std::move(child_frame)) {}

  const std::string & name() const {return name_;}
  const std::string & expectedFrame() const {return frame_;}
  const std::string & expectedChild() const {return child_frame_;}

  void setResetJump(double s) {reset_jump_s_ = s;}
  void setTimeout(double s) {timeout_s_ = s;}
  double timeout() const {return timeout_s_;}

  // True once, when the newest stamp `now` is more than timeout() after this channel's last
  // sample. A channel that never sent anything does not time out (it has not started yet).
  bool checkTimeout(double now)
  {
    if (!has_last_ || timed_out_ || !(timeout_s_ > 0.0) || now - last_ <= timeout_s_) {return false;}
    timed_out_ = true;
    return true;
  }
  bool timedOut() const {return timed_out_;}

  // Length of the outage the last accepted sample ended [s], 0 if it did not end one. Cleared.
  double takeRecoveredGap()
  {
    const double g = recovered_gap_;
    recovered_gap_ = 0.0;
    return g;
  }

  // True when frame (and child frame, if this channel has one) match the sensor contract.
  bool frameOk(const std::string & frame, const std::string & child = "") const
  {
    return frame == frame_ && (child_frame_.empty() || child == child_frame_);
  }

  // Classify the stamp t [s]; only kOk and kReset samples are recorded as accepted.
  StampCheck check(double t)
  {
    if (t <= 0.0) {++n_zero_; return StampCheck::kZero;}
    if (has_last_) {
      if (t == last_) {++n_dup_; return StampCheck::kDuplicate;}
      if (t < last_ - reset_jump_s_) {
        ++n_reset_;
        resetWindow();
        timed_out_ = false;
        recovered_gap_ = 0.0;
        has_last_ = false;  // a new timeline: no gap to the old one
        record(t);
        return StampCheck::kReset;
      }
      if (t < last_) {++n_ooo_; return StampCheck::kOutOfOrder;}
    }
    record(t);
    return StampCheck::kOk;
  }

  bool hasData() const {return has_last_;}
  double lastStamp() const {return last_;}

  // Statistics since the previous call, then start a new window.
  WindowStats takeWindow()
  {
    WindowStats s;
    s.count = win_n_;
    if (win_n_ >= 2 && win_last_ > win_first_) {
      s.rate_hz = static_cast<double>(win_n_ - 1) / (win_last_ - win_first_);
    }
    s.max_gap_s = win_max_gap_;
    resetWindow();
    return s;
  }

  std::size_t accepted() const {return n_ok_;}
  std::size_t zeros() const {return n_zero_;}
  std::size_t duplicates() const {return n_dup_;}
  std::size_t outOfOrder() const {return n_ooo_;}
  std::size_t resets() const {return n_reset_;}

private:
  void record(double t)
  {
    // Gap to the previous accepted sample, also across window boundaries, so an outage
    // that ends right after a report still shows up. After a reset t < last_ -> ignored.
    if (has_last_) {win_max_gap_ = std::max(win_max_gap_, t - last_);}
    if (timed_out_) {
      recovered_gap_ = t - last_;
      timed_out_ = false;
    }
    if (win_n_ == 0) {win_first_ = t;}
    win_last_ = t;
    ++win_n_;
    last_ = t;
    has_last_ = true;
    ++n_ok_;
  }

  void resetWindow()
  {
    win_n_ = 0;
    win_first_ = win_last_ = win_max_gap_ = 0.0;
  }

  std::string name_, frame_, child_frame_;
  double reset_jump_s_{1.0};
  double timeout_s_{0.0};  // <= 0: never times out
  bool timed_out_{false};
  double recovered_gap_{0.0};

  bool has_last_{false};
  double last_{0.0};

  std::size_t win_n_{0};
  double win_first_{0.0}, win_last_{0.0}, win_max_gap_{0.0};

  std::size_t n_ok_{0}, n_zero_{0}, n_dup_{0}, n_ooo_{0}, n_reset_{0};
};

}  // namespace lrs_localization
