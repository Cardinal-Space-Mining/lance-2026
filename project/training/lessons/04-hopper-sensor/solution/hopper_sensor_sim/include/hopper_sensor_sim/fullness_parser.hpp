// Lesson 4 reference solution: parsing one line from the hopper fullness
// sensor's serial output, e.g. "Fs: 12.5".

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
 *     if "Fs: " in line:
 *         data = float(line.replace("Fs: ", ""))
 * ...with a bare `except:` catching anything that isn't a number. Since
 * float() has to accept the WHOLE rest of the line, in practice the line must
 * be "Fs: <number>", give or take whitespace. */
inline std::optional<float> parseFullnessLine(std::string_view line)
{
    constexpr std::string_view PREFIX = "Fs: ";

    // Python's float() ignores surrounding whitespace (like the "\r\n" at the
    // end of a serial line), so we trim it too
    while (!line.empty() && std::isspace(static_cast<unsigned char>(line.front())))
    {
        line.remove_prefix(1);
    }
    while (!line.empty() && std::isspace(static_cast<unsigned char>(line.back())))
    {
        line.remove_suffix(1);
    }

    if (!line.starts_with(PREFIX))  // C++20
    {
        return std::nullopt;
    }
    line.remove_prefix(PREFIX.size());

    // "Fs:  5" (two spaces) works in Python, since float() trims it
    while (!line.empty() && std::isspace(static_cast<unsigned char>(line.front())))
    {
        line.remove_prefix(1);
    }

    // std::from_chars reports errors with a return code instead of throwing
    float value{};
    const char* const end = line.data() + line.size();
    const auto [ptr, ec] = std::from_chars(line.data(), end, value);

    if (ec != std::errc{} || ptr != end)
    {
        return std::nullopt;  // not a number, or junk after the number
    }
    if (!std::isfinite(value))
    {
        return std::nullopt;  // "nan" and "inf" parse fine, but aren't readings
    }
    return value;
}

}  // namespace hopper_sensor_sim
