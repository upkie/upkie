// SPDX-License-Identifier: Apache-2.0
// Copyright 2023 Inria

#include <fstream>

#include "gtest/gtest.h"
#include "upkie/cpp/observation/sources/Keyboard.h"

namespace upkie::cpp::observation::sources {

TEST(Keyboard, WriteOnce) {
  Keyboard keyboard;
  Dictionary observation;
  ASSERT_NO_THROW(keyboard.write(observation));
}

TEST(Keyboard, ReadAlphabetical) {
  // We cannot write directly to STDIN, so we'll redirect a file to it
  char tmpfn[] = "/tmp/kbdXXXXXX";
  ::mkstemp(tmpfn);
  std::ofstream tmpf(tmpfn);
  tmpf << "A";
  tmpf.close();

  ASSERT_TRUE(freopen(tmpfn, "r", stdin) != nullptr);

  Keyboard keyboard = Keyboard();

  Dictionary observation;
  keyboard.write(observation);

  const auto& output = observation(keyboard.prefix());
  ASSERT_TRUE(output.get<bool>("key_pressed"));
  ASSERT_FALSE(output.get<bool>("left"));
  ASSERT_FALSE(output.get<bool>("up"));
  ASSERT_FALSE(output.get<bool>("down"));
  ASSERT_FALSE(output.get<bool>("right"));
  ASSERT_FALSE(output.get<bool>("w"));
  ASSERT_TRUE(output.get<bool>("a"));
  ASSERT_FALSE(output.get<bool>("s"));
  ASSERT_FALSE(output.get<bool>("d"));
  ASSERT_FALSE(output.get<bool>("x"));
}

TEST(Keyboard, ReadArrows) {
  // We cannot write directly to STDIN, so we'll redirect a file to it
  char* tmpfn = std::tmpnam(nullptr);
  std::ofstream tmpf(tmpfn, std::ios::binary | std::ios::out);
  tmpf.write(reinterpret_cast<const char*>(LEFT_BYTES), sizeof(LEFT_BYTES));
  tmpf.close();

  ASSERT_TRUE(freopen(tmpfn, "r", stdin) != nullptr);

  Keyboard keyboard = Keyboard();

  Dictionary observation;
  keyboard.write(observation);

  const auto& output = observation(keyboard.prefix());
  ASSERT_TRUE(output.get<bool>("key_pressed"));
  ASSERT_TRUE(output.get<bool>("left"));
  ASSERT_FALSE(output.get<bool>("up"));
  ASSERT_FALSE(output.get<bool>("down"));
  ASSERT_FALSE(output.get<bool>("right"));
  ASSERT_FALSE(output.get<bool>("w"));
  ASSERT_FALSE(output.get<bool>("a"));
  ASSERT_FALSE(output.get<bool>("s"));
  ASSERT_FALSE(output.get<bool>("d"));
  ASSERT_FALSE(output.get<bool>("x"));
}

TEST(Keyboard, ReadEmptySTDIN) {
  fflush(stdin);  // Clear STDIN

  Keyboard keyboard = Keyboard();

  Dictionary observation;
  keyboard.write(observation);

  const auto& output = observation(keyboard.prefix());
  ASSERT_FALSE(output.get<bool>("key_pressed"));
  ASSERT_FALSE(output.get<bool>("left"));
  ASSERT_FALSE(output.get<bool>("up"));
  ASSERT_FALSE(output.get<bool>("down"));
  ASSERT_FALSE(output.get<bool>("right"));
  ASSERT_FALSE(output.get<bool>("w"));
  ASSERT_FALSE(output.get<bool>("a"));
  ASSERT_FALSE(output.get<bool>("s"));
  ASSERT_FALSE(output.get<bool>("d"));
  ASSERT_FALSE(output.get<bool>("x"));
}

}  // namespace upkie::cpp::observation::sources
