// Lesson 4, Part 2: tests for parseFullnessLine.
// These start out FAILING. Implement the parser until they pass, then add
// your own tests for the cases listed at the bottom.

#include <gtest/gtest.h>

#include "hopper_sensor_sim/fullness_parser.hpp"

using hopper_sensor_sim::parseFullnessLine;


TEST(FullnessParserTest, ParsesSimpleReading)
{
    const auto v = parseFullnessLine("Fs: 12.5");
    ASSERT_TRUE(v.has_value());
    EXPECT_FLOAT_EQ(*v, 12.5f);
}

TEST(FullnessParserTest, RejectsLinesWithoutPrefix)
{
    EXPECT_FALSE(parseFullnessLine("").has_value());
    EXPECT_FALSE(parseFullnessLine("Calibrating...").has_value());
}

// TODO (Part 2): add tests for:
//   - a line ending in "\r\n"
//   - negative numbers and whole numbers
//   - "Fs:12.5" (no space after the colon)
//   - "[42] Fs: 1.0" (what does the Python do with this?)
//   - "Fs: ", "Fs: abc", "Fs: 12.5.3", "Fs: 4.0 kg"
//   - "Fs: nan", "Fs: inf", and "Fs: ovf" (Arduino prints these for invalid
//     or too-large floats)
