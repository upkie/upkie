/*
 * Copyright 2022 Stéphane Caron
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <palimpsest/Dictionary.h>
#include <palimpsest/KeyError.h>

#include <memory>

#include "gtest/gtest.h"
#include "upkie_locomotion/observers/FloorContact.h"

namespace upkie_locomotion::observers::tests {

using palimpsest::Dictionary;
using palimpsest::KeyError;

class FloorContactTest : public ::testing::Test {
 protected:
  void SetUp() override {
    config_("spine_frequency") = 1000u;

    FloorContact::Parameters params;
    floor_contact_ = std::make_unique<FloorContact>(params);
  }

 protected:
  //! Test config
  Dictionary config_;

  //! Observer
  std::unique_ptr<FloorContact> floor_contact_;
};

TEST_F(FloorContactTest, WriteEvenAfterEmptyRead) {
  // Writing after an empty read is the current spec though not ideal
  Dictionary observation;
  ASSERT_NO_THROW(floor_contact_->read(observation));
  ASSERT_NO_THROW(floor_contact_->write(observation));
  ASSERT_FALSE(observation.is_empty());
}

}  // namespace upkie_locomotion::observers::tests
