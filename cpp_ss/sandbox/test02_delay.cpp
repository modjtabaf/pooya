
#include <iostream>
#include <math.h>
#include <vector>
#include <chrono>

#include "blocks.hpp"
#include "helper.hpp"
#include "solver.hpp"
#include "gp-ios.hpp"

using namespace blocks;

class MyModel : public Submodel
{
public:
    MyModel(Submodel* parent, const Signal& x, const Signal& y) : Submodel(parent, "MyModel", x, y)
    {
        auto time_delay = signal("time_delay");
        auto initial = signal("initial");

        new Const(this, "TimeDelay", 2.7435, time_delay);
        new Const(this, "Initial", 0.0, initial);
        new Delay(this, "Delay", {x, time_delay, initial}, y);
    }
};

int main()
{
    using milli = std::chrono::milliseconds;
    auto start = std::chrono::high_resolution_clock::now();

    auto model = Model("test02");
    auto x = model.signal("x");
    auto y = model.signal("y");
    new MyModel(&model, x, y);

    History history(model);

    Simulator sim(model,
        [&](double t, Values& values) -> void
        {
            values.set(x, std::sin(M_PI * t / 5));
        });

    uint k = 0;
    double t;
    while (arange(k, t, 0, 10, 0.1))
    {
        sim.run(t);
        history.update(k, t, sim.values());
        k++;
    }

    auto finish = std::chrono::high_resolution_clock::now();
    std::cout << "It took "
              << std::chrono::duration_cast<milli>(finish - start).count()
              << " milliseconds\n";

    Gnuplot gp;
	gp << "set xrange [0:100]\n";
    gp << "set yrange [-1:1]\n";
	gp << "plot" << gp.file1d(history[x]) << "with lines title 'x',"
		<< gp.file1d(history[y]) << "with lines title 'xd'\n";

    return 0;
}
