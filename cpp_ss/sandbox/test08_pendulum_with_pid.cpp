
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
    Pendulum(Submodel* parent, const Node& tau, const Node& phi) : Submodel(parent, "pendulum", {tau}, {phi})
    {
        // nodes
        Node dphi("dphi");
        Node m("m");
        Node g("g");
        Node l("l");

        // blocks
        new MulDiv(this, "tau/ml2", "*///", {tau, m, l, l});
        new AddSub(this, "err", "+-", {"", -1});
        new Integrator(this, "dphi", "", dphi);
        new Integrator(this, "phi", dphi, phi);
        // new Function(this, "sin(phi)",
        //     [](double t, const Value& x) -> Value
        //     {
        //         return x.sin();
        //     }, phi);
        new Sin(this, "sin(phi)", {phi});
        new MulDiv(this, "g/l", "**/", {"", g, l}, -1);
    }
};

class PID : public Submodel
{
public:
    PID(Submodel* parent, double Kp, double Ki, double Kd, Node& iport, Node& oport, double x0=0.0) :
        Submodel(parent, "PID", iport, oport)
    {
        // nodes
        auto& x = iport;

        // blocks
        new Gain(this, "Kp", Kp, x, {-1});
        new Integrator(this, "ix", x, "", x0);
        new Gain(this, "Ki", Ki, {""}, {-2});
        new Derivative(this, "dx", x, "");
        new Gain(this, "Kd", Kd, {""}, {-3});
        new AddSub(this, "", "+++", {-1, -2, -3}, oport);
    }
};

class SSModel : public Submodel
{
public:
    SSModel(Submodel* parent) : Submodel(parent, "pendulum_with_PID")
    {
        // nodes
        Node phi("phi");
        Node tau("tau");
        Node err("err");

        // blocks
        new AddSub(this, "", "+-", {"des_phi", phi}, err);
        new PID(this, 40.0, 20.0, 0.05, err, tau);
        new Pendulum(this, tau, phi);
    }
};

int main()
{
    using milli = std::chrono::milliseconds;
    auto start = std::chrono::high_resolution_clock::now();

    auto model = Model();

    NodeIdValues parameters({
        {      "m", 0.2   },
        {      "l", 0.1   },
        {      "g", 9.81  },
        {"des_phi", M_PI_4},
        }, model);

    auto phi  = Node("phi",  model);
    auto dphi = Node("dphi", model);
    auto tau  = Node("tau",  model);

    auto ss_model = SSModel(&model);
    auto history = run(model,
        [](uint k, double& t) -> bool
        {
            return arange(k, t, 0, 5, 0.01);
        },
        nullptr, parameters, rk4);

    auto finish = std::chrono::high_resolution_clock::now();
    std::cout << "It took "
              << std::chrono::duration_cast<milli>(finish - start).count()
              << " milliseconds\n";

    Gnuplot gp;
	gp << "set xrange [0:500]\n";
    gp << "set yrange [-80:80]\n";
	gp << "plot" << gp.file1d((history[phi] * (180/M_PI)).eval()) << "with lines title 'phi',"
	    << gp.file1d(history[dphi]) << "with lines title 'dphi',"
	    << gp.file1d(history[tau]) << "with lines title 'tau'\n";

    return 0;
}
