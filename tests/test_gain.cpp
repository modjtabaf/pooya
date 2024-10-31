/*
Copyright 2024 Mojtaba (Moji) Fathi

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”),
to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <cstddef>
#include <math.h>

#include <gtest/gtest.h>

#include "src/block/gain.hpp"
#include "src/block/submodel.hpp"
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

    // model setup
    pooya::Submodel model("");
    pooya::Gain gain("gain", 2.0);
    pooya::ScalarSignal s_x("x");
    pooya::ScalarSignal s_y("y");
    model.add_block(gain, s_x, s_y);

    // simulator setup
    pooya::Simulator sim(model,
        [&](pooya::Block&, double /*t*/) -> void
        {
            s_x = x;
        });

    // do one step
    sim.init(0.0);

    // verify the results
    EXPECT_NEAR(gain.gain() * x, s_y, 1e-10);
}

TEST_F(TestGain, ArrayGain)
{
    // test parameters
    constexpr std::size_t N = 4;
    const pooya::ArrayN<N> x{3.7, -2.5, 10.45, 0.0};

    // model setup
    pooya::Submodel model("");
    pooya::GainA gain("gain", -5.89);
    pooya::ArraySignal s_x("x", N);
    pooya::ArraySignal s_y("y", N);
    model.add_block(gain, s_x, s_y);

    // simulator setup
    pooya::Simulator sim(model,
        [&](pooya::Block&, double /*t*/) -> void
        {
            s_x = x;
        });

    // do one step
    sim.init(0.0);

    // verify the results
    auto des_y = gain.gain() * x;
    // auto y = s_y->get();
    for (std::size_t k=0; k < N; k++)
    {
        EXPECT_NEAR(des_y[k], s_y[k], 1e-10);
    }
}
