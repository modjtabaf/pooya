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

class MassSpringDamper : public Base
{
protected:
    double   _m;
    double   _k;
    double   _c;
    double  _x0;
    double _xd0;

    Signal _s_tau;
    Signal   _s_x;
    Signal  _s_xd;
    Signal _s_xdd;

public:
    MassSpringDamper(std::string given_name, double m, double k, double c, double x0, double xd0) :
        Base(given_name), _m(m), _k(k), _c(c), _x0(x0), _xd0(xd0) {}

    bool init(Parent& parent, const Signals& iports, const Signals&) override
    {
        if (!Base::init(parent, iports))
            return false;

        _s_tau = iports[0];

        _s_x   = parent.signal(  "x");
        _s_xd  = parent.signal( "xd");
        _s_xdd = parent.signal("xdd");

        // it is not necessary to add these dependencies since both _x and _xd are states and so, are known always
        _add_dependecny(_s_x);
        _add_dependecny(_s_xd);

        return true;
    }

    void get_states(StatesInfo& states) override
    {
        states.add( _s_x,  _x0,  _s_xd);
        states.add(_s_xd, _xd0, _s_xdd);
    }

    void activation_function(double /*t*/, Values& values) override
    {
        // get states and input
        double   x = values[  _s_x][0];
        double  xd = values[ _s_xd][0];
        double tau = values[_s_tau][0];

        // calculate acceleration
        double xdd = tau/_m - _c/_m * xd - _k/_m * x;

        // assign acceleration
        values.set(_s_xdd, xdd);
    }
};

int main()
{
    using milli = std::chrono::milliseconds;
    auto start = std::chrono::high_resolution_clock::now();

    // create raw blocks
    Model          model("test10");
    MassSpringDamper msd(   "msd", 1, 1, 0.1, 0.1, -0.2);

    // create signals
    auto tau = model.signal("tau");

    // setup the model
    model.add_block(msd, tau);

    History history(model);

    Simulator sim(model,
    [&](double t, Values& values) -> void
    {
        values.set(tau, 0.01 * std::sin(t));
    }, rkf45);

    uint k = 0;
    double t;
    while (arange(k, t, 0, 50, 0.01))
    {
        sim.run(t);
        history.update(k, t, sim.values());
        k++;
    }

    auto finish = std::chrono::high_resolution_clock::now();
    std::cout << "It took "
              << std::chrono::duration_cast<milli>(finish - start).count()
              << " milliseconds\n";

    history.shrink_to_fit();

    auto x = model.find_signal(".x");

    Gnuplot gp;
	gp << "set xrange [0:" << history.nrows() - 1 << "]\n";
    gp << "set yrange [-0.4:0.5]\n";
	gp << "plot" << gp.file1d(history[x]) << "with lines title 'x'\n";

    return 0;
}
