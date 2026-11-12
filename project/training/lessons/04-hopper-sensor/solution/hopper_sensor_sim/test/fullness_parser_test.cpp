#include <gtest/gtest.h>

#include "hopper_sensor_sim/fullness_parser.hpp"

using hopper_sensor_sim::parseFullnessLine;


TEST(FullnessParserTest, ParsesSimpleReading)
{
    const auto v = parseFullnessLine("Fs: 12.5");
    ASSERT_TRUE(v.has_value());
    EXPECT_FLOAT_EQ(*v, 12.5f);
}

TEST(FullnessParserTest, IgnoresLineEndings)
{
    const auto v = parseFullnessLine("Fs: 3.25\r\n");
    ASSERT_TRUE(v.has_value());
    EXPECT_FLOAT_EQ(*v, 3.25f);
}

TEST(FullnessParserTest, ParsesNegativeAndInteger)
{
    EXPECT_FLOAT_EQ(parseFullnessLine("Fs: -0.5").value(), -0.5f);
    EXPECT_FLOAT_EQ(parseFullnessLine("Fs: 7").value(), 7.f);
}

TEST(FullnessParserTest, RejectsTextBeforePrefixLikePython)
{
    // Python: float("[42] 1.0") raises ValueError
    EXPECT_FALSE(parseFullnessLine("[42] Fs: 1.0").has_value());
}

TEST(FullnessParserTest, RejectsLinesWithoutPrefix)
{
    EXPECT_FALSE(parseFullnessLine("").has_value());
    EXPECT_FALSE(parseFullnessLine("Calibrating...").has_value());
    EXPECT_FALSE(parseFullnessLine("12.5").has_value());
    EXPECT_FALSE(parseFullnessLine("Fs:12.5").has_value());  // no space
}

TEST(FullnessParserTest, RejectsGarbageValues)
{
    EXPECT_FALSE(parseFullnessLine("Fs: ").has_value());
    EXPECT_FALSE(parseFullnessLine("Fs: abc").has_value());
    EXPECT_FALSE(parseFullnessLine("Fs: 12.5.3").has_value());
    EXPECT_FALSE(parseFullnessLine("Fs: 4.0 kg").has_value());
}

TEST(FullnessParserTest, RejectsNonFiniteValues)
{
    // Arduino's Serial.print prints these for invalid or huge floats
    EXPECT_FALSE(parseFullnessLine("Fs: nan").has_value());
    EXPECT_FALSE(parseFullnessLine("Fs: inf").has_value());
    EXPECT_FALSE(parseFullnessLine("Fs: ovf").has_value());
}
