/*
  Copyright 2021 - Rafael Barreto
*/


#include "rrt_star_global_planner/random_double_generator.hpp"

namespace rrt_star_global_planner {

void RandomDoubleGenerator::setRange(double min, double max) {
  min_value_ = min;
  max_value_ = max;
}

double RandomDoubleGenerator::generate() {
  std::mt19937 gen(rd_());

  // Note: uniform_real_distribution does [start, stop), but we want to do [start, stop].
  // Therefore passing the next largest value instead.
  return std::uniform_real_distribution {min_value_, std::nextafter(max_value_, DBL_MAX)}(gen);
}

}  // namespace rrt_star_global_planner
