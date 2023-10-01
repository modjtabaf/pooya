
#include <iostream>
#include <math.h>
#include <vector>
#include <chrono>

#include "blocks.hpp"
#include "helper.hpp"
#include "solver.hpp"
#include "gp-ios.hpp"

using namespace blocks;

class Pendulum : public Submodel
{
public:
    Pendulum(Submodel* parent, const Signal& tau, const Signal& phi) : Submodel(parent, "pendulum", {tau}, {phi})
    {
        // signals
        auto dphi = signal( "dphi");

        auto m = parameter("m");
        auto g = parameter("g");
        auto l = parameter("l");

        // blocks
        new MulDiv(this, "tau\\ml2", "*///", {tau, m, l, l}, signal(10));
        new AddSub(this, "err", "+-", {signal(10), signal(20)}, signal(30));
        new Integrator(this, "dphi", signal(30), dphi);
        new Integrator(this, "phi", dphi, phi);
        // new Function(this, "sin(phi)",
        //     [](double t, const Value& x) -> Value
        //     {
        //         return x.sin();
        //     }, phi, signal(40));
        new Sin(this, "sin(phi)", phi, signal(40));
        new MulDiv(this, "g\\l", "**/", {signal(40), g, l}, signal(20));
    }
};

class PID : public Submodel
{
public:
    PID(Submodel* parent, double Kp, double Ki, double Kd, Signal& x, Signal& y, double x0=0.0) :
        Submodel(parent, "PID", x, y)
    {
        // blocks
        new Gain(this, "Kp", Kp, x, signal(10));
        new Integrator(this, "ix", x, signal(20), x0);
        new Gain(this, "Ki", Ki, signal(20), signal(30));
        new Derivative(this, "dx", x, signal(40));
        new Gain(this, "Kd", Kd, signal(40), signal(50));
        new AddSub(this, "AddSub", "+++", {signal(10), signal(30), signal(50)}, y);
    }
};

class SSModel : public Submodel
{
public:
    SSModel(Submodel* parent) : Submodel(parent, "pendulum_with_PID")
    {
        // signals
        auto phi = signal("phi");
        auto tau = signal("tau");
        auto err = signal("err");

        auto des_phi = parameter("des_phi");

        // blocks
        new AddSub(this, "AddSub", "+-", {des_phi, phi}, err);
        new PID(this, 40.0, 20.0, 0.05, err, tau);
        new Pendulum(this, tau, phi);
    }
};

int main()
{
    using milli = std::chrono::milliseconds;
    auto start = std::chrono::high_resolution_clock::now();

    auto model = Model("test08");
    auto m = model.signal("m");
    auto l = model.signal("l");
    auto g = model.signal("g");
    auto des_phi = model.signal("des_phi");
    auto ss_model = SSModel(&model);
    auto history = run(model,
        [](uint k, double& t) -> bool
        {
            return arange(k, t, 0, 2, 0.01);
        },
        [&](double /*t*/, Values& values) -> void
        {
            values.set(m, 0.2);
            values.set(l, 0.1);
            values.set(g, 9.81);
            values.set(des_phi, M_PI_4);
        },
        rk4);

    auto finish = std::chrono::high_resolution_clock::now();
    std::cout << "It took "
              << std::chrono::duration_cast<milli>(finish - start).count()
              << " milliseconds\n";

    auto  phi = history[model.find_signal("/test08/pendulum_with_PID.phi")];
    auto dphi = history[model.find_signal("/test08/pendulum_with_PID/pendulum.dphi")];
    auto  tau = history[model.find_signal("/test08/pendulum_with_PID.tau")];
 
    Gnuplot gp;
	gp << "set xrange [0:" << phi.size() << "]\n";
    gp << "set yrange [-80:80]\n";
	gp << "plot" << gp.file1d((phi * (180/M_PI)).eval()) << "with lines title 'phi',"
	    << gp.file1d(dphi) << "with lines title 'dphi',"
	    << gp.file1d(tau) << "with lines title 'tau'\n";

    return 0;
}
