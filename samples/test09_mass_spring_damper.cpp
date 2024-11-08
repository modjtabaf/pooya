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

#include <iostream>
#include <chrono>

#include "src/helper/trace.hpp"
#include "src/block/submodel.hpp"
#include "src/solver/euler.hpp"
#include "src/solver/simulator.hpp"
#include "src/solver/history.hpp"
#include "src/misc/gp-ios.hpp"

class MassSpringDamper : public pooya::Block
{
protected:
    double _m;
    double _k;
    double _c;
    double _x;
    double _xd;

public:
    MassSpringDamper(const std::string& given_name, double m, double k, double c, double x0, double xd0) :
        pooya::Block(given_name, true, 1, 0), _m(m), _k(k), _c(c), _x(x0), _xd(xd0) {}

    pooya::ScalarSignal _s_tau{"tau"};
    pooya::ScalarSignal _s_x{"x"};
    pooya::ScalarSignal _s_xd{"xd"};
    pooya::ScalarSignal _s_xdd{"xdd"};

    bool init(pooya::Parent* parent, pooya::BusId ibus, pooya::BusId) override
    {
        pooya_trace0;

        if (!pooya::Block::init(parent, ibus))
            return false;

        _s_tau.reset(scalar_input_at(0));

        _s_x.set_deriv_signal(_s_xd);
        _s_xd.set_deriv_signal(_s_xdd);

        register_associated_signal(_s_x, SignalAssociationType::Input);
        register_associated_signal(_s_xd, SignalAssociationType::Input);
        register_associated_signal(_s_xdd, SignalAssociationType::Internal);

        return true;
    }

    void activation_function(double /*t*/) override
    {
        pooya_trace0;

        // calculate acceleration
        _s_xdd = _s_tau/_m - _c/_m * _s_xd - _k/_m * _s_x;
    }

    void pre_step(double /*t*/) override
    {
        _s_x = _x;
        _s_xd = _xd;
    }

    void post_step(double /*t*/) override
    {
        _x = _s_x;
        _xd = _s_xd;
    }
};

int main()
{
    pooya_trace0;

    using milli = std::chrono::milliseconds;
    auto start = std::chrono::high_resolution_clock::now();

    // create pooya blocks
    pooya::Submodel model("test10");
    MassSpringDamper msd("msd", 1, 1, 0.1, 0.1, -0.2);

    // create pooya signals
    pooya::ScalarSignal tau("tau");

    // setup the model
    model.add_block(msd, tau);

    pooya::Euler stepper;
    pooya::Simulator sim(model,
    [&](pooya::Block&, double t) -> void
    {
        pooya_trace0;
        tau = 0.01 * std::sin(t);
    }, &stepper);

    pooya::History history(model);
    history.track(msd._s_x);

    uint k = 0;
    double t;
    while (pooya::arange(k, t, 0, 50, 0.01))
    {
        sim.run(t);
        history.update(k, t);
        k++;
    }

    auto finish = std::chrono::high_resolution_clock::now();
    std::cout << "It took "
              << std::chrono::duration_cast<milli>(finish - start).count()
              << " milliseconds\n";

    history.shrink_to_fit();

    Gnuplot gp;
	gp << "set xrange [0:" << history.nrows() - 1 << "]\n";
    gp << "set yrange [-0.4:0.5]\n";
	gp << "plot" << gp.file1d(history[msd._s_x]) << "with lines title 'x'\n";

    assert(pooya::helper::pooya_trace_info.size() == 1);

    return 0;
}
