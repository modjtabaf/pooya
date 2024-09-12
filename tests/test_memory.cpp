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

#include "src/block/memory.hpp"
#include "src/block/model.hpp"
#include "src/signal/scalar_signal.hpp"
#include "src/solver/simulator.hpp"

class TestMemory : public testing::Test
{
public:
    TestMemory()
    {
        //
    }
};

TEST_F(TestMemory, ScalarMemory)
{
    // test parameters
    const double x = 3.7;
    const double x0 = -4.8;

    // model setup
    pooya::Model model;
    pooya::Memory memory("memory", x0);
    pooya::ScalarSignalId s_x = model.create_scalar_signal("x");
    pooya::ScalarSignalId s_y = model.create_scalar_signal("y");
    model.add_block(memory, s_x, s_y);

    // simulator setup
    pooya::Simulator sim(model,
        [&](pooya::Model&, double /*t*/) -> void
        {
            s_x->set(x);
        });

    // do one step
    sim.init(0.0);

    // verify the results
    EXPECT_EQ(x0, *s_y);
}

TEST_F(TestMemory, ArrayMemory)
{
    // test parameters
    constexpr std::size_t N = 4;
    const pooya::ArrayN<N> x{3.7, -2.5, 10.45, 0.0};
    const pooya::ArrayN<N> x0{-10.56, 0.18, 7.24, -3.67};

    // model setup
    pooya::Model model;
    pooya::MemoryA memory("memory", x0);
    pooya::ArraySignalId s_x = model.create_array_signal("x", N);
    pooya::ArraySignalId s_y = model.create_array_signal("y", N);
    model.add_block(memory, s_x, s_y);

    // simulator setup
    pooya::Simulator sim(model,
        [&](pooya::Model&, double /*t*/) -> void
        {
            s_x->set(x);
        });

    // do one step
    sim.init(0.0);

    // verify the results
    auto y = s_y->get();
    for (std::size_t k=0; k < N; k++)
    {
        EXPECT_EQ(y[k], x0[k]);
    }
}
