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

// §3: an outage is measured in header stamps: `now` is the newest stamp of any sensor.
TEST(SensorChannel, TimeoutAndRecoveryFromStamps)
{
  SensorChannel gps("gps", "map");
  gps.setTimeout(1.0);
  EXPECT_FALSE(gps.checkTimeout(100.0));  // never sent anything: not started, not timed out
  for (int k = 0; k <= 10; ++k) {ASSERT_EQ(gps.check(10.0 + 0.2 * k), StampCheck::kOk);}  // last 12.0
  EXPECT_FALSE(gps.checkTimeout(12.9));
  EXPECT_FALSE(gps.timedOut());
  EXPECT_TRUE(gps.checkTimeout(13.01));   // 1.01 s of silence
  EXPECT_TRUE(gps.timedOut());
  EXPECT_FALSE(gps.checkTimeout(20.0));   // reported once only
  EXPECT_DOUBLE_EQ(gps.takeRecoveredGap(), 0.0);
  ASSERT_EQ(gps.check(27.0), StampCheck::kOk);  // GPS back after a 15 s outage
  EXPECT_FALSE(gps.timedOut());
  EXPECT_DOUBLE_EQ(gps.takeRecoveredGap(), 15.0);
  EXPECT_DOUBLE_EQ(gps.takeRecoveredGap(), 0.0);  // cleared
  ASSERT_EQ(gps.check(27.2), StampCheck::kOk);
  EXPECT_DOUBLE_EQ(gps.takeRecoveredGap(), 0.0);  // an ordinary sample ends nothing
}

TEST(SensorChannel, ResetClearsTimeout)
{
  SensorChannel imu("imu", "base_link");
  imu.setTimeout(0.2);
  ASSERT_EQ(imu.check(50.0), StampCheck::kOk);
  EXPECT_TRUE(imu.checkTimeout(51.0));
  EXPECT_EQ(imu.check(1.0), StampCheck::kReset);
  EXPECT_FALSE(imu.timedOut());
  EXPECT_DOUBLE_EQ(imu.takeRecoveredGap(), 0.0);  // no "outage" of -49 s
}
