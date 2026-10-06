#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>
namespace embodied {
constexpr int64_t step_ns = 5000000, period_ns = 100000000;
inline std::array<double, 3> direction(uint64_t i) {
  double az = 2 * M_PI * std::fmod(i * 0.6180339887498949, 1.0);
  double el = (-7 + 59 * std::fmod(i * 0.4142135623730951, 1.0)) * M_PI / 180;
  return {std::cos(el) * std::cos(az), std::cos(el) * std::sin(az),
          std::sin(el)};
}
struct Point {
  float x, y, z, intensity;
  uint32_t offset;
  uint8_t line, tag;
  uint8_t padding[2];
};
static_assert(sizeof(Point) == 24);
struct Packet {
  int64_t start = -1, last = -1;
  std::vector<uint8_t> data;
  void reset() {
    start = last = -1;
    data.clear();
  }
  void begin(int64_t now) {
    if (last >= now)
      reset();
    if (start < 0)
      start = now;
    last = now;
  }
  void add(float x, float y, float z, uint8_t line, int64_t now) {
    Point p{x,    y,    z,     100.f, static_cast<uint32_t>(now - start),
            line, 0x10, {0, 0}};
    const auto *bytes = reinterpret_cast<const uint8_t *>(&p);
    data.insert(data.end(), bytes, bytes + 24);
  }
};
} // namespace embodied
