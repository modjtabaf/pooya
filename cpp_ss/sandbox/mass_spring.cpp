
#include <iostream>
#include <math.h>
#include <vector>

#include "blocks.hpp"
#include "helper.hpp"
#include "solver.hpp"
#include "gp-ios.hpp"

using namespace blocks;

class SSModel : public Submodel
{
public:
    SSModel() : Submodel("")
    {
        auto x   = N("x"  );
        auto xd  = N("xd" );
        auto xdd = N("xdd");

        enter();
        {
            new Integrator("xd", xdd, xd, 0.1);
            new Integrator("x", xd, x);
            new Gain("-k/m", -1.0/1.0, Ports({x}), Ports({xdd}));
        }
        exit();
    }
};

int main()
{
    auto model = SSModel();
    auto history = run(model,
        [](uint k, double& t) -> bool
        {
            return arange(k, t, 0, 5, 0.01);
        },
        nullptr, NamedValues(), rk4);

    Gnuplot gp;
	gp << "set xrange [0:500]\n";
    gp << "set yrange [-0.15:0.15]\n";
	gp << "plot" << gp.file1d(history["x"]) << "with lines title 'x',"
		<< gp.file1d(history["xd"]) << "with lines title 'xd'\n";

    return 0;
}
