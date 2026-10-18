#include <gtest/gtest.h>

#include "parking_planner/core/core_smoke.h"

TEST(CoreSmokeTest, ReturnExpectedValue) {
  EXPECT_EQ(42, parking_planner::core::coreSmokeValue());
}