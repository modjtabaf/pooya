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

#include "src/core/pooya.hpp"
#include "src/core/helper.hpp"
#include "src/core/solver.hpp"
#include "src/misc/gp-ios.hpp"

using namespace pooya;

class Pendulum : public Submodel
{
protected:
    Integrator _integ1{"dphi"};
    Integrator _integ2{ "phi", M_PI_4};
    // Function     _func{"sin(phi)",
    //     [](double /*t*/, const Value& x) -> Value
    //     {
    //         return x.sin();
    //     }},
    Sin           _sin{"sin(phi)"};
    MulDiv     _muldiv{   "-g\\l", "**/", -1};

public:
    Pendulum() : Submodel("pendulum") {}

    bool init(Parent& parent, const Signals&, const Signals&) override
    {
        if (!Submodel::init(parent))
            return false;

        // create signals
        auto   phi = signal(  "phi");
        auto  dphi = signal( "dphi");
        auto d2phi = signal("d2phi");

        auto s10 = signal(); // choose a random name for this internal signal

        auto g = parameter("g");
        auto l = parameter("l");

        // setup the submodel
        add_block(_integ1,       d2phi,  dphi);
        add_block(_integ2,        dphi,   phi);
        // add_block(  _func,         phi,   s10);
        add_block(   _sin,         phi,   s10);
        add_block(_muldiv, {s10, g, l}, d2phi);

        return true;
    }
};

int main()
{
    using milli = std::chrono::milliseconds;
    auto start = std::chrono::high_resolution_clock::now();

    // create raw blocks
    Model    model("test05");
    Pendulum pendulum;

    // setup the model
    model.add_block(pendulum);

    History history(model);

    auto l = model.signal("l");
    auto g = model.signal("g");

    Simulator sim(model,
        [&](Model&, double /*t*/, Values& values) -> void
        {
            values.set_scalar(l, 0.1);
            values.set_scalar(g, 9.81);
        },
        rk4);

    uint k = 0;
    double t;
    while (arange(k, t, 0, 5, 0.01))
    {
        sim.run(t);
        history.update(k, t, sim.values());
        k++;
    }

    auto finish = std::chrono::high_resolution_clock::now();
    std::cout << "It took "
              << std::chrono::duration_cast<milli>(finish - start).count()
              << " milliseconds\n";

    auto phi = model.find_signal("/pendulum.phi");

    history.shrink_to_fit();

    Gnuplot gp;
	gp << "set xrange [0:" << history.nrows() - 1 << "]\n";
    gp << "set yrange [-0.8:0.8]\n";
	gp << "plot" << gp.file1d(history[phi]) << "with lines title 'x'\n";

    return 0;
}
