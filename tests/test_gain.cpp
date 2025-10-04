/*
Copyright 2024 Mojtaba (Moji) Fathi

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

#include "src/block/extra/gain.hpp"
#include "src/signal/array_signal.hpp"
#include "src/signal/scalar_signal.hpp"
#include "src/solver/simulator.hpp"

class TestGain : public testing::Test
{
public:
    TestGain()
    {
        //
    }
};

TEST_F(TestGain, ScalarGain)
{
    // test parameters
    const double x = 3.7;
    constexpr double gain_value{2.0};

    // model setup
    pooya::Gain gain(gain_value, nullptr, "gain");
    pooya::ScalarSignal s_x("x");
    pooya::ScalarSignal s_y("y");
    gain.connect({s_x}, {s_y});

    // simulator setup
    pooya::Simulator sim(gain, [&](pooya::Block&, double /*t*/) -> void { s_x = x; });

    // do one step
    sim.init(0.0);

    // verify the results
    EXPECT_DOUBLE_EQ(gain_value * x, s_y);
}

#ifdef POOYA_INT_SIGNAL
TEST_F(TestGain, IntGain)
{
    // test parameters
    const int x = 3;
    constexpr int gain_value{2};

    // model setup
    pooya::GainT<int, int> gain(gain_value);
    pooya::IntSignal s_x;
    pooya::IntSignal s_y;
    gain.connect({s_x}, {s_y});

    // simulator setup
    pooya::Simulator sim(gain, [&](pooya::Block&, double /*t*/) -> void { s_x = x; });

    // do one step
    sim.init(0.0);

    // verify the results
    EXPECT_EQ(gain_value * x, s_y);
}
#endif // POOYA_INT_SIGNAL

#ifdef POOYA_ARRAY_SIGNAL
TEST_F(TestGain, ArrayGain)
{
    // test parameters
    constexpr std::size_t N = 4;
    const pooya::ArrayN<N> x{3.7, -2.5, 10.45, 0.0};
    constexpr double gain_value{-5.89};

    // model setup
    pooya::GainA gain(gain_value);
    pooya::ArraySignal s_x(N);
    pooya::ArraySignal s_y(N);
    gain.connect({s_x}, {s_y});

    // simulator setup
    pooya::Simulator sim(gain, [&](pooya::Block&, double /*t*/) -> void { s_x = x; });

    // do one step
    sim.init(0.0);

    // verify the results
    for (std::size_t k = 0; k < N; k++)
    {
        EXPECT_NEAR(gain_value * s_x[k], s_y[k], 1e-10);
    }
}
#endif // POOYA_ARRAY_SIGNAL
