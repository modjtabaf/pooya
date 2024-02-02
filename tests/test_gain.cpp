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

#include <iostream>
#include <math.h>
#include <vector>
#include <chrono>

#include <gtest/gtest.h>

#include "src/core/pooya.hpp"
#include "src/core/helper.hpp"
#include "src/core/solver.hpp"
#include "src/misc/gp-ios.hpp"

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
    constexpr double x = 3.7;
    pooya::Model model;
    pooya::Gain gain("gain", 2.0);
    pooya::ScalarSignal s_x = model.create_scalar_signal("x");
    pooya::ScalarSignal s_y = model.create_scalar_signal("y");
    model.add_block(gain, s_x, s_y);
    pooya::Simulator sim(model,
        [&](pooya::Model&, double /*t*/, pooya::Values& values) -> void
        {
            values.set(s_x, 3.7);
        });
    sim.init(0.0);
    EXPECT_NEAR(gain.gain() * x, sim.values().get(s_y), 1e-10);
}
