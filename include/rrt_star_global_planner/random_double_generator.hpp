#ifndef RANDOM_DOUBLE_GENERATOR_HPP_
#define RANDOM_DOUBLE_GENERATOR_HPP_

#include <random>
#include <cfloat> // DBL_MAX

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
    return std::uniform_real_distribution{-min_value_, std::nextafter(max_value_, DBL_MAX)}(gen);
  }
};

#endif  // RANDOM_DOUBLE_GENERATOR_HPP_
