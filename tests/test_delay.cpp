/*
Copyright 2024 Mojtaba (Moji) Fathi

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
documentation files (the “Software”), to deal in the Software without restriction, including without limitation the
rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit
persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
Software.

 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <cstddef>
#include <math.h>

#include <gtest/gtest.h>

#include "src/block/delay.hpp"
#include "src/block/submodel.hpp"
#include "src/signal/scalar_signal.hpp"
#include "src/solver/simulator.hpp"

class TestDelay : public testing::Test
{
public:
    TestDelay()
    {
        //
    }
};

TEST_F(TestDelay, ScalarDelay)
{
    // test parameters
    const double time_delay = 2.78;
    const double t_end      = 7.32;
    const double dt         = 0.01;
    const double initial    = 0.0;
    auto func               = [](double t) -> double { return -1.3 * t + 4.92; };

    // model setup
    pooya::Submodel model;
    pooya::Delay delay(&model, 10.0);
    pooya::ScalarSignal s_time_delay;
    pooya::ScalarSignal s_initial;
    pooya::ScalarSignal s_x;
    pooya::ScalarSignal s_y;
    model.add_block(delay, {{"delay", s_time_delay}, {"in", s_x}, {"initial", s_initial}}, s_y);

    // simulator setup
    pooya::Simulator sim(model,
                         [&](pooya::Block&, double t) -> void
                         {
                             s_time_delay = time_delay;
                             s_initial    = initial;
                             s_x          = func(t);
                         });

    double t;

    // initialize and run the simulation
    sim.init(0.0);
    for (t = dt; t < time_delay / 2; t += dt) sim.run(t);

    // verify the results
    EXPECT_EQ(initial, s_y);

    // continue running the simulation
    for (; t < t_end; t += dt) sim.run(t);

    // verify the results
    EXPECT_NEAR(func(t_end - time_delay), s_y, 1e-10);
}

TEST_F(TestDelay, ArrayDelay)
{
    // test parameters
    constexpr std::size_t N        = 5;
    const double time_delay        = 2.78;
    const double t_end             = 7.32;
    const double dt                = 0.01;
    const pooya::ArrayN<N> initial = pooya::ArrayN<N>::Zero();
    auto func                      = [](double t) -> pooya::ArrayN<N>
    {
        static const pooya::ArrayN<N> A{3.7, -2.5, 10.45, 0.0, -0.02};
        static const pooya::ArrayN<N> B{-10.56, 0.18, 7.24, -3.67, 0.876};
        return A * t + B;
    };

    // model setup
    pooya::Submodel model;
    pooya::DelayA delay(&model, 10.0);
    pooya::ScalarSignal s_time_delay;
    pooya::ArraySignal s_initial(N);
    pooya::ArraySignal s_x(N);
    pooya::ArraySignal s_y(N);
    model.add_block(delay, {{"delay", s_time_delay}, {"in", s_x}, {"initial", s_initial}}, s_y);

    // simulator setup
    pooya::Simulator sim(model,
                         [&](pooya::Block&, double t) -> void
                         {
                             s_time_delay = time_delay;
                             s_initial    = initial;
                             s_x          = func(t);
                         });

    double t;

    // initialize and run the simulation
    sim.init(0.0);
    for (t = dt; t < time_delay / 2; t += dt) sim.run(t);

    // verify the results
    EXPECT_EQ((initial - *s_y).abs().maxCoeff(), 0);

    // continue running the simulation
    for (; t < t_end; t += dt) sim.run(t);

    // verify the results
    EXPECT_NEAR((func(t_end - time_delay) - *s_y).abs().maxCoeff(), 0, 1e-10);
}
