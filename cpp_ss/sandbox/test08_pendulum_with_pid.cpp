
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
        auto dphi = get_model()->create_node( "dphi");
        auto    m = get_model()->create_node(    "m");
        auto    g = get_model()->create_node(    "g");
        auto    l = get_model()->create_node(    "l");

        // blocks
        new MulDiv(this, "tau\\ml2", "*///", {tau, m, l, l}, 10);
        new AddSub(this, "err", "+-", {10, 20}, 30);
        new Integrator(this, "dphi", 30, dphi);
        new Integrator(this, "phi", dphi, phi);
        // new Function(this, "sin(phi)",
        //     [](double t, const Value& x) -> Value
        //     {
        //         return x.sin();
        //     }, phi, 40);
        new Sin(this, "sin(phi)", phi, 40);
        new MulDiv(this, "g\\l", "**/", {40, g, l}, 20);
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
        new Gain(this, "Kp", Kp, x, 10);
        new Integrator(this, "ix", x, 20, x0);
        new Gain(this, "Ki", Ki, 20, 30);
        new Derivative(this, "dx", x, 40);
        new Gain(this, "Kd", Kd, 40, 50);
        new AddSub(this, "AddSub", "+++", {10, 30, 50}, oport);
    }
};

class SSModel : public Submodel
{
public:
    SSModel(Submodel* parent) : Submodel(parent, "pendulum_with_PID")
    {
        // nodes
        auto phi = get_model()->create_node("phi");
        auto tau = get_model()->create_node("tau");
        auto err = get_model()->create_node("err");
        auto des_phi = get_model()->create_node("des_phi");

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

    NodeValues parameters({
        {      "m", 0.2   },
        {      "l", 0.1   },
        {      "g", 9.81  },
        {"des_phi", M_PI_4},
        }, model);

    auto  phi = model.create_node( "phi");
    auto dphi = model.create_node("dphi");
    auto  tau = model.create_node( "tau");

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
