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
#include "src/block/divide.hpp"
#include "src/block/gain.hpp"
#include "src/block/integrator.hpp"
#include "src/block/multiply.hpp"
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
    pooya::Multiply _mul1{this};
    pooya::Divide _div1{this};
    pooya::Subtract _sub{this};
    pooya::Integrator _integ1{this};
    pooya::Integrator _integ2{this};
    pooya::Sin _sin{this};
    pooya::Multiply _mul2{this};
    pooya::Divide _div2{this};

public:
    Pendulum(pooya::Submodel* parent) : pooya::Submodel(parent)
    {
        rename("pendulum");
        _mul1.rename("tau");
        _div1.rename("_ml2");
        _sub.rename("err");
        _integ1.rename("dphi");
        _integ2.rename("phi");
        _sin.rename("sin(phi)");
        _mul2.rename("g");
        _div2.rename("_l");
    }

    pooya::ScalarSignal _m{"m"};
    pooya::ScalarSignal _g{"g"};
    pooya::ScalarSignal _l{"l"};
    pooya::ScalarSignal _dphi{"dphi"};

    bool connect(const pooya::Bus& ibus, const pooya::Bus& obus) override
    {
        pooya_trace0;

        if (!pooya::Submodel::connect(ibus, obus)) return false;

        pooya::ScalarSignal s05;
        pooya::ScalarSignal s10;
        pooya::ScalarSignal s15;
        pooya::ScalarSignal s20;
        pooya::ScalarSignal s30;
        pooya::ScalarSignal s40;

        auto tau = scalar_input_at(0);
        auto phi = scalar_output_at(0);

        // setup the submodel
        _mul1.connect({_m, _l, _l}, s05);
        _div1.connect({tau, s05}, s10);
        _sub.connect({s10, s20}, s30);
        _integ1.connect(s30, _dphi);
        _integ2.connect(_dphi, phi);
        _sin.connect(phi, s40);
        _mul2.connect({s40, _g}, s15);
        _div2.connect({s15, _l}, s20);

        return true;
    }
};

class PID : public pooya::Submodel
{
protected:
    pooya::Gain _gain_p;
    pooya::Integrator _integ;
    pooya::Gain _gain_i;
    pooya::Add _add{this};
    pooya::Derivative _deriv{this};
    pooya::Gain _gain_d;

public:
    PID(pooya::Submodel* parent, double Kp, double Ki, double Kd, double x0 = 0.0)
        : pooya::Submodel(parent), _gain_p(this, Kp), _integ(this, x0), _gain_i(this, Ki), _gain_d(this, Kd)
    {
        rename("PI");
        _gain_p.rename("Kp");
        _integ.rename("ix");
        _gain_i.rename("Ki");
        _gain_d.rename("Kd");
    }

    bool connect(const pooya::Bus& ibus, const pooya::Bus& obus) override
    {
        pooya_trace0;

        if (!pooya::Submodel::connect(ibus, obus)) return false;

        pooya::ScalarSignal s10;
        pooya::ScalarSignal s20;
        pooya::ScalarSignal s30;
        pooya::ScalarSignal s40;
        pooya::ScalarSignal s50;

        auto x = scalar_input_at(0);
        auto y = scalar_output_at(0);

        // blocks
        _gain_p.connect(x, s10);
        _integ.connect(x, s20);
        _gain_i.connect(s20, s30);
        _deriv.connect(x, s40);
        _gain_d.connect(s40, s50);
        _add.connect({s10, s30, s50}, y);

        return true;
    }
};

class PendulumWithPID : public pooya::Submodel
{
protected:
    pooya::Subtract _sub{this};
    PID _pid{this, 40.0, 20.0, 0.05};

public:
    Pendulum _pend{this};
    pooya::ScalarSignal _des_phi{"des_phi"};
    pooya::ScalarSignal _phi{"phi"};
    pooya::ScalarSignal _tau{"tau"};
    pooya::ScalarSignal _err{"err"};

    PendulumWithPID()
    {
        rename("pendulum_with_PID");

        _sub.connect({_des_phi, _phi}, _err);
        _pid.connect(_err, _tau);
        _pend.connect(_tau, _phi);
    }
};

int main()
{
    pooya_trace0;

    using milli = std::chrono::milliseconds;
    auto start  = std::chrono::high_resolution_clock::now();

    // create pooya blocks
    PendulumWithPID pendulum_with_pid;

    pooya::Rkf45 stepper;
    pooya::Simulator sim(
        pendulum_with_pid,
        [&](pooya::Block&, double /*t*/) -> void
        {
            pendulum_with_pid._pend._m = 0.2;
            pendulum_with_pid._pend._l = 0.1;
            pendulum_with_pid._pend._g = 9.81;
            pendulum_with_pid._des_phi = M_PI_4;
        },
        &stepper);

    pooya::History history;
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
