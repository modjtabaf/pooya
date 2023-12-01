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
    Integrator _integ1{   "dphi", M_PI_4};
    Integrator _integ2{    "phi"};
    Function     _func{"sin_phi",
        [](double /*t*/, const Value& x) -> Value
        {
            return x.sin();
        }};
    // Sin _sin{"sin(phi)"};
    MulDiv   _muldiv1{    "g_l",  "**/"};
    MulDiv   _muldiv2{"tau_ml2", "*///"};
    Subtract     _sub{  "d2phi"};

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

        // choose random names for these internal signals
        auto s10 = signal();
        auto s20 = signal();
        auto s30 = signal();

        auto tau = parameter("tau");
        auto   m = parameter(  "m");
        auto   g = parameter(  "g");
        auto   l = parameter(  "l");

        // setup the submodel
        add_block( _integ1,          d2phi,  dphi);
        add_block( _integ2,           dphi,   phi);
        add_block(   _func,            phi,   s10);
        // add_block(    _sin,            phi,   s10);
        add_block(_muldiv1,    {s10, g, l},   s20);
        add_block(_muldiv2, {tau, m, l, l},   s30);
        add_block(    _sub,     {s30, s20}, d2phi);

        return true;
    }
};

int main()
{
    using milli = std::chrono::milliseconds;
    auto  start = std::chrono::high_resolution_clock::now();

    // create raw blocks
    Model    model("test06");
    Pendulum pendulum;

    // setup the model
    model.add_block(pendulum);

    History history(model);

    auto   m = model.signal(  "m");
    auto   l = model.signal(  "l");
    auto   g = model.signal(  "g");
    auto tau = model.signal("tau");

    Simulator sim(model,
        [&](double /*t*/, Values& values) -> void
        {
            values.set(  m,  0.2);
            values.set(  l,  0.1);
            values.set(  g, 9.81);
            values.set(tau, 0.13);
        },
        rkf45, true);

    uint k = 0;
    double t;
    while (arange(k, t, 0, 5, 0.1))
    {
        sim.run(t);
        history.update(k, t, sim.values());
        k++;
    }

    auto finish = std::chrono::high_resolution_clock::now();
    std::cout << "It took "
              << std::chrono::duration_cast<milli>(finish - start).count()
              << " milliseconds\n";

    auto  phi = model.find_signal("/pendulum.phi");
    auto dphi = model.find_signal(        ".dphi");

    history.shrink_to_fit();

    Gnuplot gp;
	gp << "set xrange [0:" << history.nrows() - 1 << "]\n";
    gp << "set yrange [-8:8]\n";
	gp << "plot" << gp.file1d(history[phi]) << "with lines title 'phi',"
	    << gp.file1d(history[dphi]) << "with lines title 'dphi'\n";

    return 0;
}
