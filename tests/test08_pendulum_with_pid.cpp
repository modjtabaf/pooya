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

        auto m = parameter("m");
        auto g = parameter("g");
        auto l = parameter("l");

        // blocks
        new MulDiv(*this, "tau\\ml2", "*///", {tau, m, l, l}, signal(10));
        new AddSub(*this, "err", "+-", {signal(10), signal(20)}, signal(30));
        new Integrator(*this, "dphi", signal(30), dphi);
        new Integrator(*this, "phi", dphi, phi);
        // new Function(*this, "sin(phi)",
        //     [](double t, const Value& x) -> Value
        //     {
        //         return x.sin();
        //     }, phi, signal(40));
        new Sin(*this, "sin(phi)", phi, signal(40));
        new MulDiv(*this, "g\\l", "**/", {signal(40), g, l}, signal(20));
    }
};

class PID : public Submodel
{
public:
    PID(Parent& parent, double Kp, double Ki, double Kd, Signal& x, Signal& y, double x0=0.0) :
        Submodel(parent, "PID", x, y)
    {
        // blocks
        new Gain(*this, "Kp", Kp, x, signal(10));
        new Integrator(*this, "ix", x, signal(20), x0);
        new Gain(*this, "Ki", Ki, signal(20), signal(30));
        new Derivative(*this, "dx", x, signal(40));
        new Gain(*this, "Kd", Kd, signal(40), signal(50));
        new AddSub(*this, "AddSub", "+++", {signal(10), signal(30), signal(50)}, y);
    }
};

class MyModel : public Model
{
public:
    MyModel() : Model("pendulum_with_PID")
    {
        // signals
        auto phi = signal("phi");
        auto tau = signal("tau");
        auto err = signal("err");

        auto des_phi = parameter("des_phi");

        // blocks
        new AddSub(*this, "AddSub", "+-", {des_phi, phi}, err);
        new PID(*this, 40.0, 20.0, 0.05, err, tau);
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
        rkf45);

    uint k = 0;
    double t;
    while (arange(k, t, 0, 5, 0.02))
    {
        sim.run(t);
        history.update(k, t, sim.values());
        k++;
    }

    auto finish = std::chrono::high_resolution_clock::now();
    std::cout << "It took "
              << std::chrono::duration_cast<milli>(finish - start).count()
              << " milliseconds\n";

    auto  phi = model.find_signal("/pendulum_with_PID.phi", true); // find using the exact name
    auto dphi = model.find_signal(".dphi"); // find using the partial name
    auto  tau = model.find_signal("tau"); // find using the partial name

    Gnuplot gp;
	gp << "set xrange [0:" << history[phi].size() - 1 << "]\n";
    gp << "set yrange [-80:80]\n";
	gp << "plot" << gp.file1d((history[phi] * (180/M_PI)).eval()) << "with lines title 'phi',"
	    << gp.file1d(history[dphi]) << "with lines title 'dphi',"
	    << gp.file1d(history[tau]) << "with lines title 'tau'\n";

    return 0;
}
