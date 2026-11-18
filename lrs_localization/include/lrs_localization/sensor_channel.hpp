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

  bool has_last_{false};
  double last_{0.0};

  std::size_t win_n_{0};
  double win_first_{0.0}, win_last_{0.0}, win_max_gap_{0.0};

  std::size_t n_ok_{0}, n_zero_{0}, n_dup_{0}, n_ooo_{0}, n_reset_{0};
};

}  // namespace lrs_localization
