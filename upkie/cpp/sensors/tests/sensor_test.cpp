// SPDX-License-Identifier: Apache-2.0
// Copyright 2023 Inria

#include "upkie/cpp/observers/Source.h"

#include <palimpsest/Dictionary.h>

#include <Eigen/Geometry>
#include <map>
#include <string>
#include <vector>

#include "gtest/gtest.h"

using palimpsest::Dictionary;

namespace upkie::cpp::observation::tests {

TEST(Source, UnknownPrefix) {
  Source source;
  ASSERT_EQ(source.prefix().rfind("unknown", 0), 0);
}

TEST(Source, WritesNothing) {
  Source source;
  Dictionary observation;
  source.write(observation);
  ASSERT_TRUE(observation.is_empty());
}

}  // namespace upkie::cpp::observation::tests
