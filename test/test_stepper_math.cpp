// Copyright (C) 2025 Christian Ehrmann — GPLv3, see LICENSE.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "cmeresearch_stepper_driver/stepper_math.hpp"

using namespace cmeresearch_stepper_driver;

TEST(StepResolution, MapsPowersOfTwoToRegister)
{
  EXPECT_EQ(step_resolution_to_register(1), 8);
  EXPECT_EQ(step_resolution_to_register(2), 7);
  EXPECT_EQ(step_resolution_to_register(8), 5);    // SILENT_STEPPER_V2_STEP_RESOLUTION_8
  EXPECT_EQ(step_resolution_to_register(16), 4);
  EXPECT_EQ(step_resolution_to_register(256), 0);
}

TEST(StepResolution, RejectsInvalidValues)
{
  for (int v : {0, -8, 3, 12, 255, 512}) {
    EXPECT_FALSE(step_resolution_to_register(v).has_value()) << v;
  }
}

TEST(StepVelocity, ConvertsOneRevolutionPerSecond)
{
  // 2*pi rad/s = 1 rev/s = 200 full steps * 8 microsteps.
  EXPECT_EQ(rad_s_to_step_velocity(2.0 * M_PI, 200, 8, 5000), 1600);
}

TEST(StepVelocity, UsesMagnitudeOnly)
{
  EXPECT_EQ(rad_s_to_step_velocity(-2.0 * M_PI, 200, 8, 5000), 1600);
}

TEST(StepVelocity, ClampsToMaxAndRegisterRange)
{
  EXPECT_EQ(rad_s_to_step_velocity(1000.0, 200, 8, 5000), 5000);
  EXPECT_EQ(rad_s_to_step_velocity(1e9, 200, 256, 100000), 65535);
  EXPECT_EQ(rad_s_to_step_velocity(1.0, 200, 8, -5), 0);
}

TEST(StepVelocity, NonFiniteIsZero)
{
  EXPECT_EQ(rad_s_to_step_velocity(std::numeric_limits<double>::quiet_NaN(), 200, 8, 5000), 0);
  EXPECT_EQ(rad_s_to_step_velocity(std::numeric_limits<double>::infinity(), 200, 8, 5000), 0);
}

TEST(Direction, FollowsPositionDelta)
{
  EXPECT_EQ(direction_from_position_delta(100, 150, -1), 1);
  EXPECT_EQ(direction_from_position_delta(100, 50, 1), -1);
}

TEST(Direction, FallsBackWhenStationary)
{
  EXPECT_EQ(direction_from_position_delta(100, 100, -1), -1);
  EXPECT_EQ(direction_from_position_delta(100, 100, 1), 1);
}

TEST(Direction, NoOverflowAtExtremes)
{
  constexpr auto lo = std::numeric_limits<int32_t>::min();
  constexpr auto hi = std::numeric_limits<int32_t>::max();
  EXPECT_EQ(direction_from_position_delta(lo, hi, 0), 1);
  EXPECT_EQ(direction_from_position_delta(hi, lo, 0), -1);
}

TEST(UnderVoltage, TripsBelowThreshold)
{
  EXPECT_FALSE(update_under_voltage(false, 17000, 17000, 500));
  EXPECT_TRUE(update_under_voltage(false, 16999, 17000, 500));
}

TEST(UnderVoltage, ClearsOnlyAboveHysteresis)
{
  EXPECT_TRUE(update_under_voltage(true, 17000, 17000, 500));
  EXPECT_TRUE(update_under_voltage(true, 17499, 17000, 500));
  EXPECT_FALSE(update_under_voltage(true, 17500, 17000, 500));
}

TEST(UnderVoltage, NegativeHysteresisTreatedAsZero)
{
  EXPECT_FALSE(update_under_voltage(true, 17000, 17000, -100));
}

TEST(CommandWatchdog, FiresOnceAfterTimeoutWhileMoving)
{
  CommandWatchdog wd(0.5);
  wd.feed(10.0, true);
  EXPECT_FALSE(wd.expired(10.4));
  EXPECT_TRUE(wd.expired(10.6));
  EXPECT_FALSE(wd.expired(11.0)) << "must fire only once";
}

TEST(CommandWatchdog, FreshCommandsKeepItQuiet)
{
  CommandWatchdog wd(0.5);
  for (double t = 0.0; t < 5.0; t += 0.1) {
    wd.feed(t, true);
    EXPECT_FALSE(wd.expired(t + 0.05));
  }
}

TEST(CommandWatchdog, ZeroCommandDisarms)
{
  CommandWatchdog wd(0.5);
  wd.feed(0.0, true);
  wd.feed(0.1, false);     // robot was told to stop: nothing to enforce
  EXPECT_FALSE(wd.expired(5.0));
  wd.feed(6.0, true);      // re-armed by the next motion command
  EXPECT_TRUE(wd.expired(6.6));
}

TEST(CommandWatchdog, DisabledWithZeroTimeout)
{
  CommandWatchdog wd(0.0);
  wd.feed(0.0, true);
  EXPECT_FALSE(wd.expired(100.0));
}
