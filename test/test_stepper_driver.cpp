// Copyright (C) 2025 Christian Ehrmann — GPLv3, see LICENSE.
//
// StepperDriver logic against a fake BrickletIo: connection retries, brickd
// disconnects, stale feedback, configuration errors and the command path.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "cmeresearch_stepper_driver/bricklet_io.hpp"
#include "cmeresearch_stepper_driver/stepper_config.hpp"
#include "cmeresearch_stepper_driver/tinkerforge_bricklet_stepper_driver.hpp"

using namespace cmeresearch_stepper_driver;
using namespace std::chrono_literals;

namespace
{

/// Shared between the test and the fake (the driver owns the fake itself).
struct FakeState
{
  std::mutex mutex;
  std::vector<std::string> calls;
  std::vector<int> connect_results;   // consumed per connect(); empty -> E_OK
  int configure_result = 0;
  std::atomic<int> connect_calls{0};
  BrickletCallbacks callbacks;

  void record(const std::string & call)
  {
    std::lock_guard<std::mutex> lock(mutex);
    calls.push_back(call);
  }
  std::vector<std::string> take()
  {
    std::lock_guard<std::mutex> lock(mutex);
    auto out = calls;
    calls.clear();
    return out;
  }
  bool called(const std::string & call)
  {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & c : calls) {
      if (c == call) {return true;}
    }
    return false;
  }
};

class FakeBrickletIo : public BrickletIo
{
public:
  explicit FakeBrickletIo(std::shared_ptr<FakeState> s)
  : s_(std::move(s)) {}

  void set_callbacks(BrickletCallbacks cb) override {s_->callbacks = std::move(cb);}
  int connect(const std::string &, uint16_t) override
  {
    ++s_->connect_calls;
    std::lock_guard<std::mutex> lock(s_->mutex);
    if (s_->connect_results.empty()) {
      return 0;
    }
    const int r = s_->connect_results.front();
    s_->connect_results.erase(s_->connect_results.begin());
    return r;
  }
  void set_auto_reconnect(bool e) override {s_->record(std::string("auto_reconnect:") + (e ? "1" : "0"));}
  int enumerate() override {s_->record("enumerate"); return 0;}
  void disconnect() override {s_->record("disconnect");}
  void create_device(const std::string & uid) override {s_->record("create:" + uid);}
  void destroy_device() override {s_->record("destroy");}
  int configure(const StepperConfig &, uint8_t reg) override
  {
    s_->record("configure:" + std::to_string(reg));
    return s_->configure_result;
  }
  int set_enabled(bool e) override {s_->record(std::string("enabled:") + (e ? "1" : "0")); return 0;}
  int drive(int dir) override {s_->record("drive:" + std::to_string(dir)); return 0;}
  int set_max_velocity(uint16_t v) override {s_->record("vel:" + std::to_string(v)); return 0;}
  int stop() override {s_->record("stop"); return 0;}

private:
  std::shared_ptr<FakeState> s_;
};

EnumerateInfo stepper_at(char position, const std::string & uid = "Xa1",
  EnumerateInfo::Type type = EnumerateInfo::Type::Available)
{
  EnumerateInfo info;
  info.uid = uid;
  info.position = position;
  info.is_silent_stepper_v2 = true;
  info.type = type;
  return info;
}

class StepperDriverTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    config_.brick_position = 'b';
    config_.microsteps = 8;
    config_.steps_per_revolution = 200;
    config_.max_step_vel = 8000;
    config_.connect_retry_ms = 5;
    config_.shutdown_ramp_ms = 0;
    config_.feedback_timeout_s = 0.5;
  }

  void make_driver()
  {
    state_ = std::make_shared<FakeState>();
    driver_ = std::make_unique<StepperDriver>(
      config_, std::make_unique<FakeBrickletIo>(state_), [](LogLevel, const std::string &) {},
      [this]() {return now_;});
  }

  /// Connected, matching bricklet enumerated and configured, data flowing.
  void make_ready()
  {
    make_driver();
    driver_->handle_connected(false);
    driver_->handle_enumerate(stepper_at('b'));
    driver_->handle_all_data(0, 0, 0, 24000, 100);
    // connect_loop is not started in these tests; mark the connection manually
    ASSERT_EQ(driver_->status(), DriverStatus::Ready);
    state_->take();
  }

  void advance(std::chrono::milliseconds d) {now_ += d;}

  StepperConfig config_;
  std::shared_ptr<FakeState> state_;
  std::unique_ptr<StepperDriver> driver_;
  StepperDriver::Clock::time_point now_{std::chrono::seconds(1000)};
};

}  // namespace

// --- configuration -------------------------------------------------------------

TEST(StepperConfigValidate, DefaultsAndRobotValuesAreValid)
{
  EXPECT_TRUE(validate(StepperConfig{}).ok());
  StepperConfig cmexaiii;
  cmexaiii.acceleration = cmexaiii.deceleration = 10000;
  cmexaiii.max_step_vel = 8000;
  cmexaiii.standstill_current = 0;
  cmexaiii.freewheel_mode = 1;
  cmexaiii.stealth_threshold = 4000;
  cmexaiii.coolstep_threshold = 6000;
  cmexaiii.classic_threshold = 10000;
  const auto check = validate(cmexaiii);
  EXPECT_TRUE(check.ok());
  EXPECT_TRUE(check.warnings.empty());
}

TEST(StepperConfigValidate, RejectsValuesTheApiWouldTruncate)
{
  auto bad = [](auto mutate) {
      StepperConfig c;
      mutate(c);
      return !validate(c).ok();
    };
  EXPECT_TRUE(bad([](StepperConfig & c) {c.acceleration = 70000;}));
  EXPECT_TRUE(bad([](StepperConfig & c) {c.deceleration = -1;}));
  EXPECT_TRUE(bad([](StepperConfig & c) {c.max_step_vel = 0;}));
  EXPECT_TRUE(bad([](StepperConfig & c) {c.max_step_vel = 70000;}));
  EXPECT_TRUE(bad([](StepperConfig & c) {c.standstill_current = 900; c.motor_run_current = 800;}));
  EXPECT_TRUE(bad([](StepperConfig & c) {c.power_down_time = 100000;}));
  EXPECT_TRUE(bad([](StepperConfig & c) {c.port = 0;}));
  EXPECT_TRUE(bad([](StepperConfig & c) {c.brick_position = 'x';}));
  EXPECT_TRUE(bad([](StepperConfig & c) {c.microsteps = 12;}));
  EXPECT_TRUE(bad([](StepperConfig & c) {c.freewheel_mode = 4;}));
  EXPECT_TRUE(bad([](StepperConfig & c) {c.feedback_timeout_s = 0.0;}));
}

TEST(StepperConfigValidate, WarnsOnUnorderedChopperThresholds)
{
  StepperConfig c;
  c.stealth_threshold = 600;
  c.coolstep_threshold = 500;
  const auto check = validate(c);
  EXPECT_TRUE(check.ok());
  EXPECT_EQ(check.warnings.size(), 1u);
}

TEST(StepperDriverConstruction, InvalidConfigThrows)
{
  StepperConfig c;
  c.acceleration = 70000;
  auto state = std::make_shared<FakeState>();
  EXPECT_THROW(
    StepperDriver(c, std::make_unique<FakeBrickletIo>(state)), std::invalid_argument);
}

// --- connection ------------------------------------------------------------------

TEST_F(StepperDriverTest, KeepsRetryingUntilBrickdAnswers)
{
  make_driver();
  state_->connect_results = {-1, -1, -1};   // brickd not up yet (E_NO_CONNECT)
  driver_->start();
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (driver_->status() == DriverStatus::Connecting &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(1ms);
  }
  EXPECT_EQ(state_->connect_calls.load(), 4);
  EXPECT_EQ(driver_->status(), DriverStatus::WaitingForBricklet);
  EXPECT_TRUE(state_->called("auto_reconnect:1"));
}

TEST_F(StepperDriverTest, DestructionStopsRetrying)
{
  make_driver();
  state_->connect_results = std::vector<int>(100000, -1);
  driver_->start();
  std::this_thread::sleep_for(20ms);
  const auto t0 = std::chrono::steady_clock::now();
  driver_.reset();
  EXPECT_LT(std::chrono::steady_clock::now() - t0, 500ms);
}

TEST_F(StepperDriverTest, ConfiguresOnlyTheMatchingBricklet)
{
  make_driver();
  driver_->handle_connected(false);
  EXPECT_TRUE(state_->called("enumerate"));
  state_->take();

  auto other_device = stepper_at('b');
  other_device.is_silent_stepper_v2 = false;
  driver_->handle_enumerate(other_device);
  driver_->handle_enumerate(stepper_at('a'));
  EXPECT_TRUE(state_->take().empty());
  EXPECT_EQ(driver_->status(), DriverStatus::WaitingForBricklet);

  driver_->handle_enumerate(stepper_at('B', "Xb2"));   // port letter is case-insensitive
  EXPECT_EQ(state_->take(), (std::vector<std::string>{"create:Xb2", "configure:5"}));
  EXPECT_EQ(driver_->status(), DriverStatus::Ready);
}

TEST_F(StepperDriverTest, RejectedConfigurationIsReportedAndRetried)
{
  make_driver();
  state_->configure_result = -9;   // E_INVALID_PARAMETER
  driver_->handle_connected(false);
  driver_->handle_enumerate(stepper_at('b'));
  EXPECT_EQ(driver_->status(), DriverStatus::Error);

  state_->take();
  driver_->command_velocity(1.0);
  EXPECT_TRUE(state_->take().empty()) << "commands must be ignored while unconfigured";

  state_->configure_result = 0;    // e.g. after a bricklet reset
  driver_->handle_enumerate(stepper_at('b', "Xa1", EnumerateInfo::Type::Connected));
  EXPECT_EQ(driver_->status(), DriverStatus::Ready);
}

// --- loss of data -----------------------------------------------------------------

TEST_F(StepperDriverTest, StaleFeedbackReportsZeroVelocity)
{
  make_ready();
  driver_->handle_all_data(1600, 100, 0, 24000, 300);
  auto fb = driver_->feedback();
  EXPECT_TRUE(fb.valid);
  EXPECT_EQ(fb.velocity, 1600);

  advance(400ms);
  EXPECT_EQ(driver_->status(), DriverStatus::Ready);
  advance(200ms);                  // 0.6 s without data
  EXPECT_EQ(driver_->status(), DriverStatus::Stale);
  fb = driver_->feedback();
  EXPECT_FALSE(fb.valid);
  EXPECT_EQ(fb.velocity, 0) << "a frozen velocity would keep odometry moving";

  driver_->handle_all_data(1600, 200, 0, 24000, 300);
  EXPECT_EQ(driver_->status(), DriverStatus::Ready);
}

TEST_F(StepperDriverTest, BrickdDisconnectZeroesFeedbackAndReconfiguresOnReconnect)
{
  make_ready();
  driver_->handle_all_data(1600, 100, 0, 24000, 300);

  driver_->handle_disconnected();
  EXPECT_EQ(driver_->status(), DriverStatus::Connecting);
  EXPECT_EQ(driver_->feedback().velocity, 0);
  driver_->command_velocity(1.0);
  EXPECT_TRUE(state_->take().empty());

  driver_->handle_connected(true);   // bindings auto-reconnect
  EXPECT_TRUE(state_->called("enumerate"));
  driver_->handle_enumerate(stepper_at('b'));
  EXPECT_TRUE(state_->called("configure:5"));
  EXPECT_EQ(driver_->status(), DriverStatus::Ready);
}

TEST_F(StepperDriverTest, UnpluggedBrickletIsNotReady)
{
  make_ready();
  driver_->handle_enumerate(stepper_at('b', "Xa1", EnumerateInfo::Type::Disconnected));
  EXPECT_EQ(driver_->status(), DriverStatus::WaitingForBricklet);
  EXPECT_EQ(driver_->feedback().velocity, 0);
}

// --- command path -------------------------------------------------------------------

TEST_F(StepperDriverTest, CommandPath)
{
  make_ready();
  // 2*pi rad/s = 1 rev/s = 200 * 8 microsteps/s; first command cycles the stage.
  driver_->command_velocity(2.0 * M_PI);
  EXPECT_EQ(state_->take(), (std::vector<std::string>{"enabled:0", "enabled:1", "drive:1", "vel:1600"}));

  driver_->command_velocity(-M_PI);
  EXPECT_EQ(state_->take(), (std::vector<std::string>{"drive:-1", "vel:800"}));

  driver_->command_velocity(0.0);  // ramp down without a direction change
  EXPECT_EQ(state_->take(), (std::vector<std::string>{"vel:0"}));

  driver_->command_velocity(1000.0);  // clamped to max_step_vel
  EXPECT_EQ(state_->take(), (std::vector<std::string>{"drive:1", "vel:8000"}));
}

TEST_F(StepperDriverTest, UnderVoltageRecoveryRearmsDriverStage)
{
  make_ready();
  driver_->command_velocity(1.0);
  state_->take();

  driver_->handle_all_data(0, 0, 0, 15000, 0);   // below 17 V
  EXPECT_TRUE(driver_->feedback().under_voltage);
  driver_->handle_all_data(0, 0, 0, 17200, 0);   // inside hysteresis: still low
  EXPECT_TRUE(driver_->feedback().under_voltage);
  driver_->handle_all_data(0, 0, 0, 24000, 0);   // recovered
  EXPECT_FALSE(driver_->feedback().under_voltage);

  driver_->command_velocity(1.0);
  const auto calls = state_->take();
  ASSERT_GE(calls.size(), 2u);
  EXPECT_EQ(calls[0], "enabled:0");
  EXPECT_EQ(calls[1], "enabled:1");
}

TEST_F(StepperDriverTest, VelocitySignFollowsThePositionCounter)
{
  make_ready();
  driver_->command_velocity(1.0);          // commanded forward
  driver_->handle_all_data(800, 1000, 0, 24000, 0);
  driver_->handle_all_data(800, 900, 0, 24000, 0);   // but still moving backward
  EXPECT_EQ(driver_->feedback().velocity, -800);
  driver_->handle_all_data(800, 1000, 0, 24000, 0);
  EXPECT_EQ(driver_->feedback().velocity, 800);
}

TEST_F(StepperDriverTest, ShutdownStopsAndDisablesTheMotor)
{
  make_ready();
  auto state = state_;
  driver_.reset();
  EXPECT_EQ(
    state->take(), (std::vector<std::string>{"stop", "enabled:0", "disconnect", "destroy"}));
}
