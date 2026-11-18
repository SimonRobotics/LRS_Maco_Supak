#include <gtest/gtest.h>

#include "lrs_localization/sensor_channel.hpp"

using lrs_localization::SensorChannel;
using lrs_localization::StampCheck;

TEST(SensorChannel, FrameCheck)
{
  SensorChannel vo("vo", "vo_odom", "base_link");
  EXPECT_TRUE(vo.frameOk("vo_odom", "base_link"));
  EXPECT_FALSE(vo.frameOk("map", "base_link"));
  EXPECT_FALSE(vo.frameOk("vo_odom", "odom"));

  SensorChannel gps("gps", "map");  // no child frame -> child ignored
  EXPECT_TRUE(gps.frameOk("map"));
  EXPECT_FALSE(gps.frameOk("odom"));
}

TEST(SensorChannel, DropsZeroDuplicateAndOutOfOrder)
{
  SensorChannel ch("imu", "base_link");
  EXPECT_EQ(ch.check(0.0), StampCheck::kZero);
  EXPECT_EQ(ch.check(10.00), StampCheck::kOk);
  EXPECT_EQ(ch.check(10.01), StampCheck::kOk);
  EXPECT_EQ(ch.check(10.01), StampCheck::kDuplicate);
  EXPECT_EQ(ch.check(10.005), StampCheck::kOutOfOrder);
  EXPECT_EQ(ch.check(10.02), StampCheck::kOk);
  EXPECT_DOUBLE_EQ(ch.lastStamp(), 10.02);  // dropped samples do not move the last stamp
  EXPECT_EQ(ch.accepted(), 3u);
  EXPECT_EQ(ch.zeros(), 1u);
  EXPECT_EQ(ch.duplicates(), 1u);
  EXPECT_EQ(ch.outOfOrder(), 1u);
}

TEST(SensorChannel, LargeBackwardJumpIsReset)
{
  SensorChannel ch("gps", "map");
  ch.setResetJump(1.0);
  EXPECT_EQ(ch.check(50.0), StampCheck::kOk);
  EXPECT_EQ(ch.check(49.5), StampCheck::kOutOfOrder);  // within 1 s: a late sample
  EXPECT_EQ(ch.check(2.0), StampCheck::kReset);        // 48 s back: the world was reset
  EXPECT_EQ(ch.check(2.2), StampCheck::kOk);           // and the channel continues from there
  EXPECT_EQ(ch.resets(), 1u);
}

TEST(SensorChannel, WindowRateAndGapUseStamps)
{
  SensorChannel ch("gps", "map");
  for (int i = 0; i <= 10; ++i) {ch.check(100.0 + 0.2 * i);}  // 11 samples at 5 Hz
  auto w = ch.takeWindow();
  EXPECT_EQ(w.count, 11u);
  EXPECT_NEAR(w.rate_hz, 5.0, 1e-9);
  EXPECT_NEAR(w.max_gap_s, 0.2, 1e-9);

  // A 15 s outage that ends in the next window is still seen as a gap there.
  ch.check(117.0);
  ch.check(117.2);
  w = ch.takeWindow();
  EXPECT_EQ(w.count, 2u);
  EXPECT_NEAR(w.max_gap_s, 15.0, 1e-9);

  w = ch.takeWindow();  // nothing arrived
  EXPECT_EQ(w.count, 0u);
  EXPECT_EQ(w.rate_hz, 0.0);
}
