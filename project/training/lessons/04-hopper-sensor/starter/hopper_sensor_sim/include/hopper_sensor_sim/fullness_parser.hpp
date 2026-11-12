// Lesson 4, Part 2: parse one line from the hopper fullness sensor's serial
// output, e.g. "Fs: 12.5".

#pragma once

#include <cctype>
#include <charconv>
#include <cmath>
#include <optional>
#include <string_view>
#include <system_error>


namespace hopper_sensor_sim
{

/* Parse one line of sensor output. Returns the value after "Fs: ", or
 * std::nullopt if the line isn't a valid reading.
 *
 * Python version (lance/scripts/hopper_fullness.py):
 *     line = self.serial.readline().decode().strip()
 *     if "Fs: " in line:
 *         data = float(line.replace("Fs: ", ""))
 * ...with a bare `except:` catching anything that isn't a number. */
inline std::optional<float> parseFullnessLine(std::string_view line)
{
    // TODO (Part 2): implement this. Steps:
    //   1. Trim whitespace from both ends (like Python's .strip())
    //   2. Check the line starts with "Fs: " and remove that prefix
    //   3. Convert the rest to a float with std::from_chars
    //   4. Reject the line if conversion failed, if anything was left over,
    //      or if the value isn't finite
    (void)line;  // remove this once you use `line`
    return std::nullopt;
}

}  // namespace hopper_sensor_sim
