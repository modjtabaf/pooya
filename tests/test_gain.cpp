/*
Copyright 2023 Mojtaba (Moji) Fathi

 Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the “Software”), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <cstddef>
#include <math.h>

#include <gtest/gtest.h>
#include <type_traits>

#include "src/core/helper.hpp"
#include "src/core/pooya.hpp"
#include "src/core/solver.hpp"
#include "src/misc/gp-ios.hpp"

template <typename T> class TestScalarGain : public testing::Test {
protected:
  void SetUp() override {}
};

typedef ::testing::Types<int, float, double> MyTypes;
TYPED_TEST_SUITE(TestScalarGain, MyTypes);

TYPED_TEST(TestScalarGain, checkScalarGain) {
  using Type = TypeParam;

  // model setup
  pooya::Model model;
  pooya::Gain gain("gain", 2.0);
  pooya::ScalarSignalId s_x = model.create_scalar_signal("x");
  pooya::ScalarSignalId s_y = model.create_scalar_signal("y");
  model.add_block(gain, s_x, s_y);

  Type x{};
  if (std::is_same<Type, int>::value) {
    x = 100;
  } else if (std::is_same<Type, float>::value) {
    x = 3.14;
  } else if (std::is_same<Type, double>::value) {
    x = 3.14159;
  }

  // simulator setup
  pooya::Simulator sim(
      model, [&](pooya::Model &, double /*t*/) -> void { s_x->set(x); });

  // do one step
  sim.init(0.0);

  // verify the results
  if (std::is_same<Type, int>::value) {
    EXPECT_EQ(gain.gain() * x, *s_y);
  } else if (std::is_same<Type, float>::value) {
    EXPECT_FLOAT_EQ(gain.gain() * x, *s_y);
  } else if (std::is_same<Type, double>::value) {
    EXPECT_DOUBLE_EQ(gain.gain() * x, *s_y);
  }
}

class TestArrayGain : public testing::Test {
protected:
  void SetUp() override {}
};

TEST_F(TestArrayGain, checkArrayGain) {
  // test parameters
  constexpr std::size_t N = 4;
  const pooya::ArrayN<N> x{3.7, -2.5, 10.45, 0.0};

  // model setup
  pooya::Model model;
  pooya::GainA gain("gain", -5.89);
  pooya::ArraySignalId s_x = model.create_array_signal("x", N);
  pooya::ArraySignalId s_y = model.create_array_signal("y", N);
  model.add_block(gain, s_x, s_y);

  // simulator setup
  pooya::Simulator sim(
      model, [&](pooya::Model &, double /*t*/) -> void { s_x->set(x); });

  // do one step
  sim.init(0.0);

  // verify the results
  auto des_y = gain.gain() * x;
  auto y = s_y->get();
  for (std::size_t k = 0; k < N; k++) {
    EXPECT_DOUBLE_EQ(des_y[k], y[k]);
  }
}
