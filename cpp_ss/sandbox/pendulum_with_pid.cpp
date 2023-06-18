
#include <iostream>
#include <math.h>
#include <vector>

#include "blocks.hpp"
#include "helper.hpp"
#include "solver.hpp"
#include "gp-ios.hpp"

using namespace blocks;

class Pendulum : public Submodel
{
public:
    Pendulum(const Node& tau, const Node& phi) : Submodel("pendulum", {tau}, {phi})
    {
        // nodes
        auto dphi  = Node("dphi");
        auto m     = Node("m");
        auto g     = Node("g");
        auto l     = Node("l");

        // blocks
        enter();
        {
            new MulDiv("tau/ml2", "*///", {tau, m, l, l});
            new AddSub("err", "+-", {Node(), Node(-1)});
            new Integrator("dphi", Node(), dphi);
            new Integrator("phi", dphi, phi);
            // new Function("sin(phi)",
            //     [](double t, const Value& x) -> Value
            //     {
            //         return x.sin();
            //     }, phi);
            new Sin("sin(phi)", {phi});
            new MulDiv("g/l", "**/", {Node(), g, l}, Node(-1));
        }
        exit();
    }
};

class PID : public Submodel
{
public:
    PID(double Kp, double Ki, double Kd, Node& iport, Node& oport, double x0=0.0) :
        Submodel("PID", iport, oport)
    {
        // nodes
        auto& x = iport;

        // blocks
        enter();
        {
            new Gain("Kp", Kp, x, Node(-1));
            new Integrator("ix", x, Node(), x0);
            new Gain("Ki", Ki, Node(), Node(-2));
            new Derivative("dx", x, Node());
            new Gain("Kd", Kd, Node(), Node(-3));
            new AddSub("", "+++", {Node(-1), Node(-2), Node(-3)}, oport);
        }
        exit();
    }
};

class SSModel : public Submodel
{
public:
    SSModel() : Submodel("pendulum_with_PID")
    {
        // nodes
        Node phi("phi");
        Node tau("tau");
        Node err("err");

        // blocks
        enter();
        {
            new AddSub("", "+-", {Node("des_phi"), phi}, err);
            new PID(40.0, 20.0, 0.05, err, tau);
            new Pendulum(tau, phi);
        }
        exit();
    }
};

int main()
{
    NodeValues parameters = {
        {"m", 0.2},
        {"l", 0.1},
        {"g", 9.81},
        {"des_phi", M_PI_4},
        };

    auto model = SSModel();
    auto history = run(model,
        [](uint k, double& t) -> bool
        {
            return arange(k, t, 0, 5, 0.01);
        },
        nullptr, parameters, rk4);

    Gnuplot gp;
	gp << "set xrange [0:500]\n";
    gp << "set yrange [-80:80]\n";
	gp << "plot" << gp.file1d((history["phi"] * (180/M_PI)).eval()) << "with lines title 'phi',"
	    << gp.file1d(history["dphi"]) << "with lines title 'dphi',"
	    << gp.file1d(history["tau"]) << "with lines title 'tau'\n";

    return 0;
}
