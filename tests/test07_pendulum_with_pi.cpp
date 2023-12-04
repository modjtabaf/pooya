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
    MulDiv     _muldiv1{"tau_ml2", "*///"};
    Subtract       _sub{    "err"};
    Integrator  _integ1{   "dphi"};
    Integrator  _integ2{    "phi"};
    Sin            _sin{"sin_phi"};
    MulDiv     _muldiv2{    "g_l", "**/"};

public:
    Pendulum() : Submodel("pendulum") {}

    bool init(Parent& parent, const Signals& iports, const Signals& oports) override
    {
        if (!Submodel::init(parent))
            return false;

        // create signals
        auto dphi = signal("dphi");

        // choose random names for these internal signals
        auto s10 = signal();
        auto s20 = signal();
        auto s30 = signal();
        auto s40 = signal();

        auto m = parameter("m");
        auto g = parameter("g");
        auto l = parameter("l");

        auto& tau = iports[0];
        auto& phi = oports[0];

        // setup the submodel
        add_block(_muldiv1, {tau, m, l, l},  s10);
        add_block(    _sub,     {s10, s20},  s30);
        add_block( _integ1,            s30, dphi);
        add_block( _integ2,           dphi,  phi);
        add_block(    _sin,            phi,  s40);
        add_block(_muldiv2,    {s40, g, l},  s20);

        return true;
    }
};

class PI : public Submodel
{
protected:
    Gain      _gain_p;
    Integrator _integ;
    Gain      _gain_i;
    Add          _add{"Add"};

public:
    PI(double Kp, double Ki, double x0=0.0) :
        Submodel("PI"),
        _gain_p("Kp", Kp),
        _integ ("ix", x0),
        _gain_i("Ki", Ki)
    {}

    bool init(Parent& parent, const Signals& iports, const Signals& oports) override
    {
        if (!Submodel::init(parent))
            return false;

        // choose random names for these internal signals
        auto s10 = signal();
        auto s20 = signal();
        auto s30 = signal();

        auto& x = iports[0];
        auto& y = oports[0];

        // blocks
        add_block(_gain_p,          x, s10);
        add_block( _integ,          x, s20);
        add_block(_gain_i,        s20, s30);
        add_block(   _add, {s10, s30},   y);

        return true;
    }
};

class PendulumWithPI : public Submodel
{
protected:
    Subtract  _sub{"Sub"};
    PI         _pi{40.0, 20.0};
    Pendulum _pend;

public:
    PendulumWithPI() : Submodel("pendulum_with_PI") {}

    bool init(Parent& parent, const Signals&, const Signals&) override
    {
        if (!Submodel::init(parent))
            return false;

        // signals
        auto phi = signal("phi");
        auto tau = signal("tau");
        auto err = signal("err");

        auto des_phi = parameter("des_phi");

        // blocks
        add_block(_sub, {des_phi, phi}, err);
        add_block( _pi,            err, tau);
        add_block(_pend,           tau, phi);

        return true;
    }
};

int main()
{
    using milli = std::chrono::milliseconds;
    auto  start = std::chrono::high_resolution_clock::now();

    // create raw blocks
    Model          model("test07");
    PendulumWithPI pendulum_with_pi;

    // setup the model
    model.add_block(pendulum_with_pi);

    History history(model);

    auto       m = model.signal(      "m");
    auto       l = model.signal(      "l");
    auto       g = model.signal(      "g");
    auto des_phi = model.signal("des_phi");

    Simulator sim(model,
        [&](Model&, double /*t*/, Values& values) -> void
        {
            values.set(      m,    0.2);
            values.set(      l,    0.1);
            values.set(      g,   9.81);
            values.set(des_phi, M_PI_4);
        },
        rkf45); // try rk4 with h = 0.01 to see the difference

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

    auto phi = model.find_signal(".phi");

    history.shrink_to_fit();

    Gnuplot gp;
	gp << "set xrange [0:" << history.nrows() - 1 << "]\n";
    gp << "set yrange [-50:150]\n";
	gp << "plot" << gp.file1d((history[phi] * (180/M_PI)).eval()) << "with lines title 'phi'"
        ","
	    // << gp.file1d(history[model.find_signal(".dphi")]) << "with lines title 'dphi',"
	    // << gp.file1d(history[model.find_signal(".tau")]) << "with lines title 'tau'"
        "\n";

    return 0;
}
