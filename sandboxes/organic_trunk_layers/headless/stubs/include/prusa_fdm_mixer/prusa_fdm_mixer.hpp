#pragma once
// Local stand-in for prusa_fdm_mixer (its download is blocked here). Only color
// previews of virtual extruders use it, which slicing tests don't need.
#include <string>
#include <vector>

namespace prusa_fdm_mixer {
struct Part { std::string color; double ratio; };
struct RGB { double r{0}, g{0}, b{0}; };
struct LAB { double L{0}, a{0}, b{0}; };
inline std::string mix(const std::vector<Part>& parts) { return parts.empty() ? std::string{} : parts.front().color; }
inline RGB hex_to_rgb(const std::string&) { return {}; }
inline LAB rgb_to_lab(const RGB&) { return {}; }
inline double delta_e_2000(const LAB&, const LAB&) { return 100.; }
} // namespace prusa_fdm_mixer
