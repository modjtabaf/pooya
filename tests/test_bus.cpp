/*
Copyright 2025 Mojtaba (Moji) Fathi

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

#include <gtest/gtest.h>

#include "src/signal/bus.hpp"
#include "src/signal/scalar_signal.hpp"

class TestBus : public testing::Test
{
public:
    TestBus()
    {
        //
    }
};

TEST_F(TestBus, Bus)
{
    // signal setup
    pooya::ScalarSignal s_x("x");
    pooya::Bus bus1({{"s0", s_x}});

    EXPECT_EQ(bus1.size(), 1);
    EXPECT_EQ(&bus1[0].impl(), &s_x.impl());
    EXPECT_TRUE(dynamic_cast<pooya::ValueSignalImpl*>(&bus1[0].impl()));
    EXPECT_TRUE(std::dynamic_pointer_cast<pooya::ValueSignalImpl>(bus1[0]->shared_from_this()));

    pooya::Bus bus2({s_x, bus1});

    EXPECT_EQ(bus2.size(), 2);
    EXPECT_EQ(&bus2[0].impl(), &s_x.impl());
    EXPECT_TRUE(dynamic_cast<pooya::ValueSignalImpl*>(&bus2[0].impl()));
    EXPECT_TRUE(std::dynamic_pointer_cast<pooya::ValueSignalImpl>(bus2[0]->shared_from_this()));
}
