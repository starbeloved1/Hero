#include <gtest/gtest.h>

#include "aim_normal/armor_selector.hpp"

namespace aim_normal
{

TEST(ArmorSelector, PrefersContinuousSameIdTarget)
{
  ArmorSelector selector(0.3, 0U);
  const std::vector<ArmorObservation> first{{1U, 3.0, 0.1, 0.0}, {3U, 2.0, 0.8, 0.0}};
  const auto first_selected = selector.select(first, 0.0, 0.0);
  ASSERT_TRUE(first_selected.has_value());
  EXPECT_EQ(first_selected->observation.id, 1U);

  const std::vector<ArmorObservation> second{{3U, 1.5, 0.0, 0.0}, {1U, 3.1, 0.1, 0.0}};
  const auto second_selected = selector.select(second, 0.0, 0.0);
  ASSERT_TRUE(second_selected.has_value());
  EXPECT_EQ(second_selected->observation.id, 1U);
}

TEST(ArmorSelector, ChoosesTargetClosestToCurrentGimbalViewWhenNoContinuousTarget)
{
  ArmorSelector selector(0.3, 0U);
  const std::vector<ArmorObservation> observations{{3U, 4.0, 1.5, 0.0}, {1U, 3.0, 0.1, 0.0}};
  const auto selected = selector.select(observations, 0.0, 0.0);
  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(selected->observation.id, 1U);
}

TEST(ArmorSelector, ResetForgetsPreviousTarget)
{
  ArmorSelector selector(0.3, 0U);
  ASSERT_TRUE(selector.select({{1U, 3.0, 0.0, 0.0}}, 0.0, 0.0).has_value());
  selector.reset();
  const auto selected = selector.select(
    {{3U, 2.0, 0.4, 0.0}, {1U, 3.1, 0.0, 0.0}}, 0.0, 0.0);
  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(selected->observation.id, 1U);
}

}  // aim_normal
