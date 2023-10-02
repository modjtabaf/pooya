
#include <iostream>
#include <math.h>
#include <vector>
#include <chrono>

#include "blocks.hpp"
#include "helper.hpp"
#include "solver.hpp"
#include "gp-ios.hpp"

using namespace blocks;

class MyModel : public Model
{
public:
    MyModel() : Model("pendulum")
    {
        auto   phi = signal(  "phi");
        auto  dphi = signal( "dphi");
        auto d2phi = signal("d2phi");

        auto g = parameter("g");
        auto l = parameter("l");

        new Integrator(this, "dphi", d2phi, dphi);
        new Integrator(this, "phi", dphi, phi, M_PI_4);
        // new Function(this, "sin(phi)",
        //     [](double /*t*/, const Value& x) -> Value
        //     {
        //         return x.sin();
        //     }, phi, 1);
        new Sin(this, "sin(phi)", phi, signal(10));
        new MulDiv(this, "-g\\l", "**/", {signal(10), g, l}, d2phi, -1);
    }
};

int main()
{
    using milli = std::chrono::milliseconds;
    auto start = std::chrono::high_resolution_clock::now();

    auto model = MyModel();

    History history(model);

    auto l = model.signal("l");
    auto g = model.signal("g");

    Simulator sim(model,
        [&](double /*t*/, Values& values) -> void
        {
            values.set(l, 0.1);
            values.set(g, 9.81);
        },
        rk4);

    uint k = 0;
    double t;
    while (arange(k, t, 0, 5, 0.01))
    {
        sim.run(t);
        history.update(k, t, sim.values());
        k++;
    }

    auto finish = std::chrono::high_resolution_clock::now();
    std::cout << "It took "
              << std::chrono::duration_cast<milli>(finish - start).count()
              << " milliseconds\n";

    auto phi = model.find_signal("/pendulum.phi", true);

    Gnuplot gp;
	gp << "set xrange [0:500]\n";
    gp << "set yrange [-0.8:0.8]\n";
	gp << "plot" << gp.file1d(history[phi]) << "with lines title 'x'\n";

    return 0;
}
