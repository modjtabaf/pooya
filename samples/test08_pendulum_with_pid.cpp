/*
Copyright 2024 Mojtaba (Moji) Fathi

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
documentation files (the “Software”), to deal in the Software without restriction, including without limitation the
rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit
persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
Software.

 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <chrono>
#include <iostream>

#include "src/block/add.hpp"
#include "src/block/derivative.hpp"
#include "src/block/gain.hpp"
#include "src/block/integrator.hpp"
#include "src/block/muldiv.hpp"
#include "src/block/sin.hpp"
#include "src/block/submodel.hpp"
#include "src/block/subtract.hpp"
#include "src/helper/trace.hpp"
#include "src/misc/gp-ios.hpp"
#include "src/solver/history.hpp"
#include "src/solver/rkf45.hpp"
#include "src/solver/simulator.hpp"

class Pendulum : public pooya::Submodel
{
protected:
    pooya::MulDiv _muldiv1{"tau_ml2", "*///"};
    pooya::Subtract _sub{"err"};
    pooya::Integrator _integ1{"dphi"};
    pooya::Integrator _integ2{"phi"};
    pooya::Sin _sin{"sin(phi)"};
    pooya::MulDiv _muldiv2{"g_l", "**/"};

public:
    Pendulum() : pooya::Submodel("pendulum") {}

    pooya::ScalarSignal _m{"m"};
    pooya::ScalarSignal _g{"g"};
    pooya::ScalarSignal _l{"l"};
    pooya::ScalarSignal _dphi{"dphi"};

    bool init(pooya::Submodel* parent, const pooya::Bus& ibus, const pooya::Bus& obus) override
    {
        pooya_trace0;

        if (!pooya::Submodel::init(parent, ibus, obus)) return false;

        pooya::ScalarSignal s10;
        pooya::ScalarSignal s20;
        pooya::ScalarSignal s30;
        pooya::ScalarSignal s40;

        auto tau = scalar_input_at(0);
        auto phi = scalar_output_at(0);

        // setup the submodel
        add_block(_muldiv1, {tau, _m, _l, _l}, s10);
        add_block(_sub, {s10, s20}, s30);
        add_block(_integ1, s30, _dphi);
        add_block(_integ2, _dphi, phi);
        add_block(_sin, phi, s40);
        add_block(_muldiv2, {s40, _g, _l}, s20);

        return true;
    }
};

class PID : public pooya::Submodel
{
protected:
    pooya::Gain _gain_p;
    pooya::Integrator _integ;
    pooya::Gain _gain_i;
    pooya::Add _add;
    pooya::Derivative _deriv;
    pooya::Gain _gain_d;

public:
    PID(double Kp, double Ki, double Kd, double x0 = 0.0)
        : pooya::Submodel("PI"), _gain_p("Kp", Kp), _integ("ix", x0), _gain_i("Ki", Ki), _gain_d("Kd", Kd)
    {
    }

    bool init(pooya::Submodel* parent, const pooya::Bus& ibus, const pooya::Bus& obus) override
    {
        pooya_trace0;

        if (!pooya::Submodel::init(parent, ibus, obus)) return false;

        pooya::ScalarSignal s10;
        pooya::ScalarSignal s20;
        pooya::ScalarSignal s30;
        pooya::ScalarSignal s40;
        pooya::ScalarSignal s50;

        auto x = scalar_input_at(0);
        auto y = scalar_output_at(0);

        // blocks
        add_block(_gain_p, x, s10);
        add_block(_integ, x, s20);
        add_block(_gain_i, s20, s30);
        add_block(_deriv, x, s40);
        add_block(_gain_d, s40, s50);
        add_block(_add, {s10, s30, s50}, y);

        return true;
    }
};

class PendulumWithPID : public pooya::Submodel
{
protected:
    pooya::Subtract _sub{"Sub"};
    PID _pid{40.0, 20.0, 0.05};

public:
    PendulumWithPID() : pooya::Submodel("pendulum_with_PID") {}

    Pendulum _pend;
    pooya::ScalarSignal _des_phi{"des_phi"};
    pooya::ScalarSignal _phi{"phi"};
    pooya::ScalarSignal _tau{"tau"};
    pooya::ScalarSignal _err{"err"};

    bool init(pooya::Submodel* parent, const pooya::Bus&, const pooya::Bus&) override
    {
        pooya_trace0;

        if (!pooya::Submodel::init(parent)) return false;

        // blocks
        add_block(_sub, {_des_phi, _phi}, _err);
        add_block(_pid, _err, _tau);
        add_block(_pend, _tau, _phi);

        return true;
    }
};

int main()
{
    pooya_trace0;

    using milli = std::chrono::milliseconds;
    auto start  = std::chrono::high_resolution_clock::now();

    // create pooya blocks
    pooya::Submodel model("test08");
    PendulumWithPID pendulum_with_pid;

    // setup the model
    model.add_block(pendulum_with_pid);

    pooya::Rkf45 stepper;
    pooya::Simulator sim(
        model,
        [&](pooya::Block&, double /*t*/) -> void
        {
            pendulum_with_pid._pend._m = 0.2;
            pendulum_with_pid._pend._l = 0.1;
            pendulum_with_pid._pend._g = 9.81;
            pendulum_with_pid._des_phi = M_PI_4;
        },
        &stepper);

    pooya::History history(model);
    history.track(pendulum_with_pid._phi);
    history.track(pendulum_with_pid._pend._dphi);
    history.track(pendulum_with_pid._tau);

    uint k = 0;
    double t;
    while (pooya::arange(k, t, 0, 5, 0.02))
    {
        sim.run(t);
        history.update(k, t);
        k++;
    }

    auto finish = std::chrono::high_resolution_clock::now();
    std::cout << "It took " << std::chrono::duration_cast<milli>(finish - start).count() << " milliseconds\n";

    history.shrink_to_fit();

    Gnuplot gp;
    gp << "set xrange [0:" << history.nrows() - 1 << "]\n";
    gp << "set yrange [-80:80]\n";
    gp << "plot" << gp.file1d((history[pendulum_with_pid._phi] * (180 / M_PI)).eval()) << "with lines title 'phi',"
       << gp.file1d(history[pendulum_with_pid._pend._dphi]) << "with lines title 'dphi',"
       << gp.file1d(history[pendulum_with_pid._tau]) << "with lines title 'tau'\n";

    assert(pooya::helper::pooya_trace_info.size() == 1);

    return 0;
}
