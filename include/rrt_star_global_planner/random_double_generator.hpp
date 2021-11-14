/*
  Copyright 2021 - Rafael Barreto
*/

#ifndef RRT_STAR_GLOBAL_PLANNER_RANDOM_DOUBLE_GENERATOR_HPP_  // NOLINT
#define RRT_STAR_GLOBAL_PLANNER_RANDOM_DOUBLE_GENERATOR_HPP_

#include <random>
#include <cfloat>  // DBL_MAX

namespace rrt_star_global_planner {

// TODO(Rafael) allow different ranges of x and y for non square maps

class RandomDoubleGenerator {
 private:
  std::random_device rd_;
  double min_value_{-1.0};
  double max_value_{1.0};

 public:
  RandomDoubleGenerator() {}

  void setRange(double min, double max) {
    min_value_ = min;
    max_value_ = max;
  }

  double generate() {
    std::mt19937 gen(rd_());

    // Note: uniform_real_distribution does [start, stop), but we want to do [start, stop].
    // Therefore passing the next largest value instead.
    return std::uniform_real_distribution {min_value_, std::nextafter(max_value_, DBL_MAX)}(gen);
  }
};
}  // namespace rrt_star_global_planner

#endif  // RRT_STAR_GLOBAL_PLANNER_RANDOM_DOUBLE_GENERATOR_HPP_  // NOLINT
