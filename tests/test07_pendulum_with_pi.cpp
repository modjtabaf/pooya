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
public:
    Pendulum(Parent& parent, const Signal& tau, const Signal& phi) : Submodel(parent, "pendulum", {tau}, {phi})
    {
        // signals
        auto dphi = signal( "dphi");

        // choose random names for these internal signals
        auto s10 = signal();
        auto s20 = signal();
        auto s30 = signal();
        auto s40 = signal();

        auto m = parameter("m");
        auto g = parameter("g");
        auto l = parameter("l");

        // blocks
        new MulDiv(*this, "tau\\ml2", "*///", {tau, m, l, l}, s10);
        new Subtract(*this, "err", {s10, s20}, s30);
        new Integrator(*this, "dphi", s30, dphi);
        new Integrator(*this, "phi", dphi, phi);
        // new Function(*this, "sin(phi)",
        //     [](double /*t*/, const Value& x) -> Value
        //     {
        //         return x.sin();
        //     }, phi, s40);
        new Sin(*this, "sin(phi)", phi, s40);
        new MulDiv(*this, "g\\l", "**/", {s40, g, l}, s20);
    }
};

class PI : public Submodel
{
public:
    PI(Parent& parent, double Kp, double Ki, const Signal& x, const Signal& y, double x0=0.0) :
        Submodel(parent, "PI", x, y)
    {
        // choose random names for these internal signals
        auto s10 = signal();
        auto s20 = signal();
        auto s30 = signal();

        // blocks
        new Gain(*this, "Kp", Kp, x, s10);
        new Integrator(*this, "ix", x, s20, x0);
        new Gain(*this, "Ki", Ki, s20, s30);
        new Add(*this, "Add", {s10, s30}, y);
    }
};

class MyModel : public Model
{
public:
    MyModel() : Model("pendulum_with_PI")
    {
        // signals
        auto phi = signal("phi");
        auto tau = signal("tau");
        auto err = signal("err");

        auto des_phi = parameter("des_phi");

        // blocks
        new Subtract(*this, "Sub", {des_phi, phi}, err);
        new PI(*this, 40.0, 20.0, err, tau);
        new Pendulum(*this, tau, phi);
    }
};

int main()
{
    using milli = std::chrono::milliseconds;
    auto start = std::chrono::high_resolution_clock::now();

    auto model = MyModel();

    auto m = model.signal("m");
    auto l = model.signal("l");
    auto g = model.signal("g");
    auto des_phi = model.signal("des_phi");

    History history(model);

    Simulator sim(model,
        [&](double /*t*/, Values& values) -> void
        {
            values.set(m, 0.2);
            values.set(l, 0.1);
            values.set(g, 9.81);
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

    auto  phi = model.find_signal(".phi");

    Gnuplot gp;
	gp << "set xrange [0:" << history[phi].size() - 1 << "]\n";
    gp << "set yrange [-50:150]\n";
	gp << "plot" << gp.file1d((history[phi] * (180/M_PI)).eval()) << "with lines title 'phi'"
        ","
	    // << gp.file1d(history[model.find_signal(".dphi")]) << "with lines title 'dphi',"
	    // << gp.file1d(history[model.find_signal(".tau")]) << "with lines title 'tau'"
        "\n";

    return 0;
}
