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

#include "src/signal/scalar_signal.hpp"

class TestScalarSignal : public testing::Test
{
public:
    TestScalarSignal()
    {
        //
    }
};

TEST_F(TestScalarSignal, ScalarSignal)
{
    // test parameters
    const double x = 3.7;
    double y;

    // signal setup
    pooya::ScalarSignal s_x;

    // get the value of an un-assigned signal
    EXPECT_THROW(y = s_x, std::runtime_error);
    EXPECT_THROW(y = s_x->get_value(), std::runtime_error);

    // assign a value to an un-assigned signal
    EXPECT_NO_THROW(s_x = x);

    // get the value of an assigned signal
    EXPECT_EQ(x, s_x);

    // clear an assigned signal
    EXPECT_NO_THROW(s_x->clear());

    // get the value of an un-assigned signal
    EXPECT_THROW(y = s_x, std::runtime_error);
    EXPECT_THROW(y = s_x->get_value(), std::runtime_error);

    // assign a value to an un-assigned signal
    EXPECT_NO_THROW(s_x->set_value(x));

    // get the value of an assigned signal
    EXPECT_EQ(x, s_x->get_value());

    // assign a value to an assigned signal
    EXPECT_THROW(s_x = x, std::runtime_error);
    EXPECT_THROW(s_x->set_value(x), std::runtime_error);
}
