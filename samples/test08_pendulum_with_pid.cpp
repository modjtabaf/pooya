
#include <iostream>

#include "src/block/add.hpp"
#include "src/block/derivative.hpp"
#include "src/block/gain.hpp"
#include "src/block/integrator.hpp"
#include "src/block/muldiv.hpp"
#include "src/block/sin.hpp"
#include "src/block/submodel.hpp"
#include "src/block/subtract.hpp"
#include "src/helper/trace.hpp"
#include "src/misc/gp-ios.hpp"
#include "src/solver/history.hpp"
#include "src/solver/rkf45.hpp"
#include "src/solver/simulator.hpp"

#define deg2rad(d) (d * (M_PI / 180))
#define rad2deg(d) (d * (180 / M_PI))

class Pendulum : public pooya::Submodel
{
protected:
    double _m{0.2}, _l{0.1}, _g{9.81};

    pooya::Gain _gain1{1 / (_m * _l * _l)};
    pooya::Sin _sin;
    pooya::Gain _gain2{_g / _l};
    pooya::Subtract _sub;
    pooya::Integrator _integ1;
    pooya::Integrator _integ2;

public:
    pooya::ScalarSignal _dphi;

    bool connect(const pooya::Bus& ibus, const pooya::Bus& obus) override
    {
        if (!pooya::Submodel::connect(ibus, obus)) return false;

        pooya::ScalarSignal tau_scaled, sin_phi, sin_phi_scaled, d2phi;

        auto tau = scalar_input_at(0);
        auto phi = scalar_output_at(0);

        // setup the submodel
        add_block(_gain1, tau, tau_scaled);
        add_block(_sin, phi, sin_phi);
        add_block(_gain2, sin_phi, sin_phi_scaled);
        add_block(_sub, {tau_scaled, sin_phi_scaled}, d2phi);
        add_block(_integ1, d2phi, _dphi);
        add_block(_integ2, _dphi, phi);

        return true;
    }
};

class PID : public pooya::Submodel
{
protected:
    pooya::Gain _gain_p;
    pooya::Integrator _integ;
    pooya::Gain _gain_i;
    pooya::Derivative _deriv;
    pooya::Gain _gain_d;
    pooya::Add _add;

public:
    PID(double Kp, double Ki, double Kd, double x0 = 0.0)
        : _gain_p(Kp), _integ(x0), _gain_i(Ki), _gain_d(Kd) {}

    bool connect(const pooya::Bus& ibus, const pooya::Bus& obus) override
    {
        if (!pooya::Submodel::connect(ibus, obus)) return false;

        pooya::ScalarSignal kp_x, xi, ki_xi, xd, kd_xd;

        auto x = scalar_input_at(0);  // e
        auto y = scalar_output_at(0); // tau

        // blocks
        add_block(_gain_p, x, kp_x);
        add_block(_integ, x, xi);
        add_block(_gain_i, xi, ki_xi);
        add_block(_deriv, x, xd);
        add_block(_gain_d, xd, kd_xd);
        add_block(_add, {kp_x, ki_xi, kd_xd}, y);

        return true;
    }
};

class PendulumWithPID : public pooya::Submodel
{
protected:
    pooya::Subtract _sub;
    PID _pid{40.0, 20.0, 0.05};

public:
    Pendulum _pend;
    pooya::ScalarSignal _des_phi, _phi, _tau, _err;

    PendulumWithPID()
    {
        add_block(_sub, {_des_phi, _phi}, _err);
        add_block(_pid, _err, _tau);
        add_block(_pend, _tau, _phi);
    }
};

int main()
{
    // instantiate the model
    PendulumWithPID pendulum_with_pid;

    // instantiate simulation helper objects
    pooya::Rkf45 stepper;
    pooya::Simulator sim(
        pendulum_with_pid,
        [&](pooya::Block&, double t) -> void // input callback
        {
            if (t < 1.0)
                pendulum_with_pid._des_phi = deg2rad(45);
            else if (t < 2.0)
                pendulum_with_pid._des_phi = deg2rad(60);
            else if (t < 3.0)
                pendulum_with_pid._des_phi = deg2rad(75);
            else if (t < 4.0)
                pendulum_with_pid._des_phi = deg2rad(25);
            else
                pendulum_with_pid._des_phi = deg2rad(45);
        },
        &stepper);

    // setup the history
    pooya::History history;
    history.track(pendulum_with_pid._des_phi);
    history.track(pendulum_with_pid._phi);
    history.track(pendulum_with_pid._pend._dphi);
    history.track(pendulum_with_pid._tau);

    // main simulation loop
    uint k = 0;
    double t;
    while (pooya::arange(k, t, 0, 5, 0.01))
    {
        sim.run(t);
        history.update(k, t);
        k++;
    }

    // plot some results
    Gnuplot gp1;
    gp1 << "set xrange [0:" << history.nrows() - 1 << "]\n";
    gp1 << "set yrange [0:90]\n";
    gp1 << "set grid\n";
    gp1 << "plot"
        << gp1.file1d(rad2deg(history[pendulum_with_pid._des_phi]).eval()) << "with lines title 'phi_{des}',"
        << gp1.file1d(rad2deg(history[pendulum_with_pid._phi]).eval()) << "with lines title 'phi'\n";

    Gnuplot gp2;
    gp2 << "set xrange [0:" << history.nrows() - 1 << "]\n";
    gp2 << "set yrange [-100:100]\n";
    gp2 << "set grid\n";
    gp2 << "plot" << gp2.file1d(rad2deg(history[pendulum_with_pid._des_phi]).eval()) << "with lines title 'phi_{des}',"
       << gp2.file1d(history[pendulum_with_pid._pend._dphi]) << "with lines title 'dphi',"
       << gp2.file1d(history[pendulum_with_pid._tau]) << "with lines title 'tau'\n";

    return 0;
}
