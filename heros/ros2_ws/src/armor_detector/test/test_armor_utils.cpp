#include <gtest/gtest.h>

#include "armor_detector/armor_utils.hpp"

TEST(ArmorUtils, KeepsExpectedColors)
{
  EXPECT_TRUE(armor_detector::shouldKeepColor(0, 0));
  EXPECT_FALSE(armor_detector::shouldKeepColor(1, 0));
  EXPECT_TRUE(armor_detector::shouldKeepColor(2, -2));
  EXPECT_FALSE(armor_detector::shouldKeepColor(2, -3));
  EXPECT_TRUE(armor_detector::shouldKeepColor(1, -3));
}

TEST(ArmorUtils, MapsModelClassesToHeroIds)
{
  EXPECT_EQ(armor_detector::mapModelClass2HeroId(0), 6);
  EXPECT_EQ(armor_detector::mapModelClass2HeroId(1), 1);
  EXPECT_EQ(armor_detector::mapModelClass2HeroId(6), 7);
  EXPECT_EQ(armor_detector::mapModelClass2HeroId(7), 0);
  EXPECT_EQ(armor_detector::mapModelClass2HeroId(8), -1);
}
