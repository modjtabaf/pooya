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

class MassSpringDamper : public pooya::Block
{
protected:
    double  _m;
    double  _k;
    double  _c;
    double  _x;
    double _xd;

    pooya::ScalarSignal _s_tau;
    pooya::ScalarSignal   _s_x;
    pooya::ScalarSignal  _s_xd;
    pooya::ScalarSignal _s_xdd;

public:
    MassSpringDamper(std::string given_name, double m, double k, double c, double x0, double xd0) :
        pooya::Block(given_name, 1, 0), _m(m), _k(k), _c(c), _x(x0), _xd(xd0) {}

    bool init(pooya::Parent& parent, const pooya::LabelSignals& iports, const pooya::LabelSignals&) override
    {
        pooya_trace0;
        if (!pooya::Block::init(parent, iports))
            return false;

        _iports.bind(_s_tau);

        _s_x   = parent.signal(  "x");
        _s_xd  = parent.signal( "xd");
        _s_xdd = parent.signal("xdd");

        auto& model_ = model_ref();
        model_.register_state( _s_x,  _s_xd);
        model_.register_state(_s_xd, _s_xdd);

        // it is not necessary to add these dependencies since both _x and _xd are states and so, are known always
        _add_dependecny(_s_x);
        _add_dependecny(_s_xd);

        return true;
    }

    void activation_function(double /*t*/, pooya::Values& values) override
    {
        pooya_trace0;
        // get states and input
        double   x = values.get(  _s_x);
        double  xd = values.get( _s_xd);
        double tau = values.get(_s_tau);

        // calculate acceleration
        double xdd = tau/_m - _c/_m * xd - _k/_m * x;

        // assign acceleration
        values.set(_s_xdd, xdd);
    }

    void pre_step(double /*t*/, pooya::Values& values) override
    {
        values.set(_s_x, _x);
        values.set(_s_xd, _xd);
    }

    void post_step(double /*t*/, const pooya::Values& values) override
    {
        _x = values.get(_s_x);
        _xd = values.get(_s_xd);
    }
};

int main()
{
    pooya_trace0;
    using milli = std::chrono::milliseconds;
    auto start = std::chrono::high_resolution_clock::now();

    // create raw blocks
    pooya::Model model("test10");
    MassSpringDamper msd("msd", 1, 1, 0.1, 0.1, -0.2);

    // create signals
    auto tau = model.signal("tau");

    // setup the model
    model.add_block(msd, tau);

    pooya::Euler stepper(model);
    pooya::Simulator sim(model,
    [&](pooya::Model&, double t, pooya::Values& values) -> void
    {
        pooya_trace0;
        values.set(tau, 0.01 * std::sin(t));
    }, &stepper);

    pooya::History history(model);

    uint k = 0;
    double t;
    while (pooya::arange(k, t, 0, 50, 0.01))
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

    auto x = model.lookup_signal("~x");

    Gnuplot gp;
	gp << "set xrange [0:" << history.nrows() - 1 << "]\n";
    gp << "set yrange [-0.4:0.5]\n";
	gp << "plot" << gp.file1d(history[x]) << "with lines title 'x'\n";

    assert(pooya::util::pooya_trace_info.size() == 1);

    return 0;
}
