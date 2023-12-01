
#include <iostream>
#include <math.h>
#include <vector>
#include <chrono>

#include "3rdparty/eigen/Eigen/Dense"

#include "src/core/pooya.hpp"
#include "src/core/helper.hpp"
#include "src/core/solver.hpp"
#include "src/misc/gp-ios.hpp"

using namespace pooya;

struct Parameters
{
    // primary parameters
    bool   has_trailer{true};
    double tractor_m   = 8760;
    double tractor_Izz = 55649;
    double trailer_m   = 6114;
    double trailer_Izz = 107407;
    double tractor_lf  = 2.585;
    double tractor_lr  = 3.2475;
    double trailer_lf  = 8.1400;
    double trailer_lr  = 3.425;
    double payload_m_max   = 21413;
    double payload_Izz_max = 261401;
    double payload_factor  = 1.0;
    double tractor_rear_axle_to_hitch = 0.15;
    double tractor_lk  = tractor_lr - tractor_rear_axle_to_hitch;
    double payload_cog_to_trailer_axle = 3.425;

    // secondary parameters
    double m1{0};
    double m2{0};
    double I1{0};
    double I2{0};
    double lf{0};
    double lr{0};
    double lk{0};
    double ltf{0};
    double ltr{0};

    Parameters() {update();}

    void update()
    {
        m1 = tractor_m;
        I1 = tractor_Izz;

        lf = tractor_lf;
        lr = tractor_lr;
        lk = tractor_lk;

        if (has_trailer)
        {
            double payload_m = payload_factor * payload_m_max;
            double payload_Izz = payload_factor * payload_Izz_max;
            double lp = payload_cog_to_trailer_axle;

            m2  = trailer_m + payload_m;
            ltr = (lp * payload_m + trailer_lr * trailer_m) / m2;
            ltf = trailer_lf + trailer_lr - ltr;
            I2  = trailer_Izz + trailer_m * std::pow(ltr - trailer_lr, 2)
                + payload_Izz + payload_m * std::pow(ltr - lp, 2);
        }
        else
        {
            m2  = 0;
            ltr = 0;
            ltf = 0;
            I2  = 0;
        }
    }
};

class Forces : public Base
{
public:
    Forces() : Base("Forces") {}

    void activation_function(double /*t*/, Values& values) override
    {
        double         delta = scalar_input(0);
        double engine_torque = scalar_input(1);
        const Value&      dq =  array_input(2);

        double     F = engine_torque * (0.5 * 5 * 2.41 / 0.5); // efficiency * gear_ratio * diff_ratio / R_tire
        double F_lon = F * cos(delta) - 500 * dq(0);

        Value F_lat(3);
        F_lat << F * sin(delta) - 500 * dq(1), 0, 0;

        array_output(0, F_lon);
        array_output(1, F_lat);
    }
};

class EquationsOfMotion : public Base
{
protected:
    Parameters& _p;

    // input signals
    Signal _s_delta;
    Signal _s_F_lon;
    Signal _s_F_lat;
    Signal     _s_q;
    Signal    _s_dq;

    // output signal
    Signal _s_d2q;

public:
    EquationsOfMotion(Parameters& params) : Base("EquationsOfMotion"), _p(params) {}

    bool init(Parent& parent, const Signals& iports, const Signals& oports) override
    {
        if (!Base::init(parent, iports, oports))
            return false;

        _s_delta = iports[0];
        _s_F_lon = iports[1];
        _s_F_lat = iports[2];
        _s_q     = iports[3];
        _s_dq    = iports[4];

        _s_d2q   = oports[0];

        return true;
    }

    void activation_function(double /*t*/, Values& values) override
    {
        // inputs
        double       delta = values[_s_delta][0];
        double       F_lon = values[_s_F_lon][0];
        const Value& F_lat = values[_s_F_lat];
        const Value&     q = values[    _s_q];
        const Value&    dq = values[   _s_dq];

        // lateral force components of axles
        double Fyf = F_lat[0];
        double Fyr = F_lat[1];
        double Fyt = F_lat[2];

        // yaw angles and rates
        double  psi1 =  q[2];
        double  psi2 =  q[3];
        double Dpsi1 = dq[2];
        double Dpsi2 = dq[3];

        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        // % Solve Equations of Motion
        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        // second derivatives
        Value d2q(4);

        double s1 = sin(psi1);
        double c1 = cos(psi1);

        if (_p.has_trailer) // Truck-Trailer Combination
        {
            double   s2 = sin(psi2);
            double   c2 = cos(psi2);
            double s1_2 = sin(psi1 - psi2);
            double c1_2 = cos(psi1 - psi2);

            // % Generalized Mass Matrix
            // % -----------------------
            Eigen::Matrix4d M;
            M  <<   _p.m1 + _p.m2 ,                0 ,            _p.lk*_p.m2*s1 ,             _p.ltf*_p.m2*s2
                ,               0 ,    _p.m1 + _p.m2 ,           -_p.lk*_p.m2*c1 ,            -_p.ltf*_p.m2*c2
                ,  _p.lk*_p.m2*s1 ,  -_p.lk*_p.m2*c1 , _p.m2*_p.lk*_p.lk + _p.I1 ,     _p.lk*_p.ltf*_p.m2*c1_2
                , _p.ltf*_p.m2*s2 , -_p.ltf*_p.m2*c2 ,   _p.lk*_p.ltf*_p.m2*c1_2 , _p.m2*_p.ltf*_p.ltf + _p.I2;

            // % Vector of Generalized Forces (gyroscopic, Coriolis, centrifugal)
            // % ----------------------------------------------------------------
            Eigen::Vector4d k;
            k << _p.m2*(_p.lk*c1*Dpsi1*Dpsi1 + _p.ltf*c2*Dpsi2*Dpsi2)
               , _p.m2*(_p.lk*s1*Dpsi1*Dpsi1 + _p.ltf*s2*Dpsi2*Dpsi2)
               ,                  Dpsi2*Dpsi2*_p.lk*_p.ltf*_p.m2*s1_2
               ,                 -Dpsi1*Dpsi1*_p.lk*_p.ltf*_p.m2*s1_2;

            // % Vector of Generalized Applied Forces
            // % ------------------------------------
            Eigen::Vector4d qe;
            qe << F_lon*c1 - Fyf*sin(delta + psi1) - Fyr*s1 - Fyt*s2
                , Fyf*cos(delta + psi1) + Fyr*c1 + Fyt*c2 + F_lon*s1
                ,  Fyf*_p.lf*cos(delta) - Fyt*_p.lk*c1_2 - Fyr*_p.lr
                ,                             -Fyt*(_p.ltf + _p.ltr);
            
            // % comput new states
            // % -----------------
            d2q = M.colPivHouseholderQr().solve(qe - k);
        }
        else // % Truck without trailer (bobtail)
        {
            // % comput new states
            // % -----------------
            d2q << -(Fyf*sin(delta + psi1) - F_lon*c1 + Fyr*s1)/_p.m1
                 ,  (Fyf*cos(delta + psi1) + Fyr*c1 + F_lon*s1)/_p.m1
                 ,          -(Fyr*_p.lr - Fyf*_p.lf*cos(delta))/_p.I1
                 ,                                                  0;
        }

        values.set(_s_d2q, d2q);
    }
};

class PT : public Submodel
{
protected:
    Subtract     _sub{"+-"};
    Const      _const;
    Divide       _div{"*\\"};
    Integrator _integ{"Int"};
    InitialValue  _iv{"IV"};
    Add          _add{"++"};

public:
    PT(double tau) : Submodel("PT"), _const("tau", tau) {}

    bool init(Parent& parent, const Signals& iports, const Signals& oports) override
    {
        if (!Submodel::init(parent, iports, oports))
            return false;

        // choose random names for these internal signals
        auto s10 = signal();
        auto s15 = signal();
        auto s20 = signal();
        auto s30 = signal();
        auto s40 = signal();

        auto&  y_in = iports[0];
        auto&    y0 = iports[1];
        auto& y_out = oports[0];

        // blocks
        add_block(  _sub, {y_in, y_out},   s10);
        add_block(_const,            {},   s15);
        add_block(  _div,    {s10, s15},   s20);
        add_block(_integ,          s20 ,   s30);
        add_block(   _iv,           y0 ,   s40);
        add_block(  _add,    {s30, s40}, y_out);

        return true;
    }
};

class ChassisDynamics : public Submodel
{
protected:
    PT                 _pt{0.1};
    Forces         _forces;
    EquationsOfMotion _eom;
    Integrator     _integ1{"d2q->dq", Value::Zero(4)};
    Integrator     _integ2{  "dq->q", Value::Zero(4)};

public:
    ChassisDynamics(Parameters& params) : Submodel("ChassisDynamics"), _eom(params) {}

    bool init(Parent& parent, const Signals& iports, const Signals& oports) override
    {
        if (!Submodel::init(parent, iports, oports))
            return false;

        // signals
        auto s_delta = signal("front_wheel_angle");
        auto     s_q = signal(  "q");
        auto    s_dq = signal( "dq");
        auto   s_d2q = signal("d2q");
        auto s_F_lon = signal("F_lon");
        auto s_F_lat = signal("F_lat");

        auto&      s_delta_Rq = iports[0];
        auto& s_engine_torque = iports[1];

        add_block(    _pt,               {s_delta_Rq, s_delta_Rq},           s_delta );
        add_block(_forces,       {s_delta, s_engine_torque, s_dq}, {s_F_lon, s_F_lat});
        add_block(   _eom, {s_delta, s_F_lon, s_F_lat, s_q, s_dq},             s_d2q );
        add_block(_integ1,                                 s_d2q ,              s_dq );
        add_block(_integ2,                                  s_dq ,               s_q );

        return true;
    }
};

int main()
{
    using milli = std::chrono::milliseconds;
    auto  start = std::chrono::high_resolution_clock::now();

    // parameters
    Parameters params;

    // create raw blocks
    Model           model("SimpleTruck");
    ChassisDynamics chdyn(params);

    // inputs
    auto      s_delta_Rq = model.signal("front_wheel_angle_Rq");
    auto s_engine_torque = model.signal("engine_torque");

    // setup the model
    model.add_block(chdyn, {s_delta_Rq, s_engine_torque});

    History history(model);

    Simulator sim(model,
        [&](double t, Values& values) -> void
        {
            double delta_Rq;
            if (t < 2)
                delta_Rq = 0;
            else if (t < 3)
                delta_Rq = 0.1;
            else if (t < 5)
                delta_Rq = 0;
            else if (t < 6)
                delta_Rq = -0.1;
            else
                delta_Rq = 0;

            values.set(     s_delta_Rq, delta_Rq);
            values.set(s_engine_torque,     5000);
        }, rk4);

    uint k = 0;
    double t;
    while (arange(k, t, 0, 100, 0.01))
    {
        sim.run(t);
        history.update(k, t, sim.values());
        k++;
    }

    history.shrink_to_fit();

    auto finish = std::chrono::high_resolution_clock::now();
    std::cout << "It took "
              << std::chrono::duration_cast<milli>(finish - start).count()
              << " milliseconds\n";

    auto T = history.at(History::time_id);
    // auto q = history.at(model.find_signal(".q"));
    auto dq = history.at(model.find_signal(".dq"));
    auto a = history.at(model.find_signal("_angle"));

    const auto& vx = dq.col(0);
    const auto& vy = dq.col(1);
    // const auto& psi1 = q.col(2);
    // const auto& psi2 = q.col(3);

    Gnuplot gp;
	gp << "set xrange [0:" << T.size() - 1 << "]\n";
    gp << "set yrange [-20:50]\n";
	gp << "plot"
        << gp.file1d(vx) << "with lines title 'vx', "
        << gp.file1d(vy) << "with lines title 'vy', "
        // << gp.file1d(psi1) << "with lines title 'psi1', "
	    // << gp.file1d(psi2) << "with lines title 'psi2', "
	    // << gp.file1d(a) << "with lines title 'angle', "
        "\n";

    return 0;
}
