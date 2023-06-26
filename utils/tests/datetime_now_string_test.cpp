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

#include "upkie/utils/datetime_now_string.h"

#include <string>

#include "gtest/gtest.h"

namespace upkie::utils {

TEST(DatetimeNowString, IsADatetimeString) {
  auto now = datetime_now_string();
  ASSERT_EQ(now.rfind("20", 0), 0);  // see at the turn of the next century!
  ASSERT_EQ(now[4], '-');
  ASSERT_EQ(now[7], '-');
  ASSERT_EQ(now[10], '_');
}

}  // namespace upkie::utils
