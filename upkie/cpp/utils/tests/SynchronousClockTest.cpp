// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#include "gtest/gtest.h"
#include "spdlog/sinks/ostream_sink.h"
#include "spdlog/spdlog.h"
#include "upkie/cpp/utils/SynchronousClock.h"

namespace upkie::cpp::utils {

TEST(SynchronousClock, GetsSomeSleep) {
  SynchronousClock clock(10 /* Hz */);  // 100 ms loop
  ASSERT_DOUBLE_EQ(clock.slack(), 0.);
  for (unsigned i = 0; i < 10; ++i) {
    clock.wait_for_next_tick();
  }
  if (clock.skip_count() < 1) {
    ASSERT_GT(clock.slack(), 1e-4);  // last sleep duration was positive
  } else /* (clock.skip_count() >= 1) */ {
    ASSERT_DOUBLE_EQ(clock.slack(), 0.);
  }
}

TEST(SynchronousClock, NoMarginWhenSkippingCycles) {
  SynchronousClock clock(100 /* Hz */);  // 10 ms loop
  ASSERT_DOUBLE_EQ(clock.slack(), 0.);
  clock.wait_for_next_tick();
  ASSERT_GT(clock.slack(), 0.001);  // 10% of 10 ms loop
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  clock.wait_for_next_tick();
  ASSERT_DOUBLE_EQ(clock.slack(), 0.);
}

TEST(SynchronousClock, SkipWarningsAreRateLimited) {
  // Capture log output
  std::ostringstream log_stream;
  auto ostream_sink =
      std::make_shared<spdlog::sinks::ostream_sink_mt>(log_stream);
  auto logger = std::make_shared<spdlog::logger>("test_logger", ostream_sink);
  spdlog::set_default_logger(logger);
  spdlog::set_level(spdlog::level::warn);

  SynchronousClock clock(100 /* Hz */);  // 10 ms loop

  // Force skips and wait over 1 second to trigger warning
  clock.wait_for_next_tick();
  for (int i = 0; i < 10; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));  // Force skip
    clock.wait_for_next_tick();
    if (i == 4) {
      // Wait slightly more than warning interval to ensure it's exceeded
      const int wait_ms = static_cast<int>(
          (SynchronousClock::skip_warning_interval() + 0.1) * 1000);
      std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
    }
  }

  std::string log_output = log_stream.str();
  size_t warning_count = 0;
  size_t pos = 0;
  while ((pos = log_output.find("Skipped", pos)) != std::string::npos) {
    warning_count++;
    pos += 7;  // length of "Skipped"
  }
  ASSERT_GE(warning_count, 1);

  // Check that the warning contains expected format
  ASSERT_NE(log_output.find("clock cycles over"), std::string::npos);
  ASSERT_NE(log_output.find("frequency is at most"), std::string::npos);
  ASSERT_NE(log_output.find("Hz < desired"), std::string::npos);
}

}  // namespace upkie::cpp::utils
