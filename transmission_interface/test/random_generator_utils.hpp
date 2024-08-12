// Copyright 2020 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \author Adolfo Rodriguez Tsouroukdissian

#ifndef RANDOM_GENERATOR_UTILS_HPP_
#define RANDOM_GENERATOR_UTILS_HPP_

#include <cstdlib>
#include <ctime>
#include <vector>

using std::vector;

/// \brief Generator of pseudo-random double in the range [min_val, max_val].
// NOTE: Based on example code available at:
// http://stackoverflow.com/questions/2860673/initializing-a-c-vector-to-random-values-fast
// Use a user specified seed instead of system time for deterministic tests
struct RandomDoubleGenerator
{
public:
  RandomDoubleGenerator(double min_val, double max_val, unsigned int seed = 1234)
  : min_val_(min_val), max_val_(max_val)
  {
    srand(seed);
  }
  double operator()()
  {
    const double range = max_val_ - min_val_;
    return rand() / static_cast<double>(RAND_MAX) * range + min_val_;
  }

private:
  double min_val_;
  double max_val_;
};

/// \brief Generator of a vector of pseudo-random doubles.
vector<double> randomVector(const vector<double>::size_type size, RandomDoubleGenerator & generator)
{
  vector<double> out;
  out.reserve(size);
  for (vector<double>::size_type i = 0; i < size; ++i)
  {
    out.push_back(generator());
  }
  return out;
}

#endif  // RANDOM_GENERATOR_UTILS_HPP_
