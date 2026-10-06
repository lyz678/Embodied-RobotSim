#include "x_bot_gazebo/sampling.hpp"
#include <iomanip>
#include <iostream>
#include <stdexcept>
int main(int argc, char **argv) {
  if (argc == 3) {
    uint64_t first = std::stoull(argv[1]), count = std::stoull(argv[2]);
    std::cout << std::setprecision(17);
    for (uint64_t i = first; i < first + count; i++) {
      auto d = embodied::direction(i);
      std::cout << d[0] << ' ' << d[1] << ' ' << d[2] << '\n';
    }
    return 0;
  }
  for (uint64_t i = 0; i < 20000; i++) {
    auto d = embodied::direction(i);
    double n = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
    if (std::abs(n - 1) > 1e-12)
      throw std::runtime_error("direction norm");
  }
  embodied::Packet p;
  p.begin(5000000);
  p.add(1, 2, 3, 3, 5000000);
  p.begin(10000000);
  p.add(4, 5, 6, 0, 10000000);
  embodied::Point point;
  std::memcpy(&point, p.data.data() + 24, 24);
  if (point.offset != 5000000 || point.line != 0 || point.tag != 0x10)
    throw std::runtime_error("wire contract");
  p.begin(1);
  if (!p.data.empty() || p.start != 1)
    throw std::runtime_error("rewind");
}
