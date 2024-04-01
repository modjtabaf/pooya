/*
Copyright 2023 Mojtaba (Moji) Fathi

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”),
to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <cstddef>
#include <iostream>
#include <math.h>
#include <vector>
#include <chrono>

#include <gtest/gtest.h>

#include "src/core/pooya.hpp"
#include "src/core/helper.hpp"
#include "src/core/solver.hpp"
#include "src/misc/gp-ios.hpp"

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
    const double t_end = 7.32;
    const double dt = 0.01;
    const double initial = 0.0;
    auto func = [](double t) -> double
    {
        return -1.3*t + 4.92;
    };

    // model setup
    pooya::Model model;
    pooya::Delay delay("delay");
    pooya::ScalarSignalId s_time_delay = model.create_scalar_signal("time_delay");
    pooya::ScalarSignalId s_initial = model.create_scalar_signal("initial");
    pooya::ScalarSignalId s_x = model.create_scalar_signal("x");
    pooya::ScalarSignalId s_y = model.create_scalar_signal("y");
    model.add_block(delay, {
        {"delay", s_time_delay},
        {"in", s_x},
        {"initial", s_initial}}, s_y);

    // simulator setup
    pooya::Simulator sim(model,
        [&](pooya::Model&, double t, pooya::Values& values) -> void
        {
            values[s_time_delay] = time_delay;
            values[s_initial] = initial;
            values[s_x] = func(t);
        });

    double t;

    // initialize and run the simulation
    sim.init(0.0);
    for (t = dt; t < time_delay/2; t += dt)
        sim.run(t);

    // verify the results
    EXPECT_EQ(initial, sim.values().get(s_y));

    // continue running the simulation
    for (; t < t_end; t += dt)
        sim.run(t);

    // verify the results
    EXPECT_NEAR(func(t_end - time_delay), sim.values().get(s_y), 1e-10);
}

TEST_F(TestDelay, ArrayDelay)
{
    // test parameters
    constexpr std::size_t N = 5;
    const double time_delay = 2.78;
    const double t_end = 7.32;
    const double dt = 0.01;
    const pooya::ArrayN<N> initial = pooya::ArrayN<N>::Zero();
    auto func = [](double t) -> pooya::ArrayN<N>
    {
        static const pooya::ArrayN<N> A{3.7, -2.5, 10.45, 0.0, -0.02};
        static const pooya::ArrayN<N> B{-10.56, 0.18, 7.24, -3.67, 0.876};
        return A*t + B;
    };

    // model setup
    pooya::Model model;
    pooya::DelayA delay("delay");
    pooya::ScalarSignalId s_time_delay = model.create_scalar_signal("time_delay");
    pooya::ArraySignalId s_initial = model.create_array_signal("initial", N);
    pooya::ArraySignalId s_x = model.create_array_signal("x", N);
    pooya::ArraySignalId s_y = model.create_array_signal("y", N);
    model.add_block(delay, {
        {"delay", s_time_delay},
        {"in", s_x},
        {"initial", s_initial}}, s_y);

    // simulator setup
    pooya::Simulator sim(model,
        [&](pooya::Model&, double t, pooya::Values& values) -> void
        {
            values[s_time_delay] = time_delay;
            values[s_initial] = initial;
            values[s_x] = func(t);
        });

    double t;

    // initialize and run the simulation
    sim.init(0.0);
    for (t = dt; t < time_delay/2; t += dt)
        sim.run(t);

    // verify the results
    EXPECT_EQ((initial - sim.values().get(s_y)).abs().maxCoeff(), 0);

    // continue running the simulation
    for (; t < t_end; t += dt)
        sim.run(t);

    // verify the results
    EXPECT_NEAR((func(t_end - time_delay) - sim.values().get(s_y)).abs().maxCoeff(), 0, 1e-10);
}
