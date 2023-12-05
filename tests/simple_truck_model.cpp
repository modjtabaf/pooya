
#include <iostream>
#include <math.h>
#include <vector>
#include <chrono>

// #include "3rdparty/eigen/Eigen/Dense"

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
        // const Value&      dq =  array_input(2);
        double           dq0 = scalar_input(2);
        double           dq1 = scalar_input(3);

        double     F = engine_torque * (0.5 * 5 * 2.41 / 0.5); // efficiency * gear_ratio * diff_ratio / R_tire
        // double F_lon = F * cos(delta) - 500 * dq(0);
        double F_lon = F * cos(delta) - 500 * dq0;

        // Value F_lat(3);
        // F_lat << F * sin(delta) - 500 * dq1, 0, 0;
        double F_lat = F * sin(delta) - 500 * dq1;

        array_output(0, F_lon);
        array_output(1, F_lat);
        array_output(2, 0);
        array_output(3, 0);
    }
};

class EquationsOfMotion : public Base
{
protected:
    Parameters& _p;

    // input signals
    Signal _s_delta;
    Signal _s_F_lon;
    // Signal _s_F_lat;
    Signal _s_F_lat0;
    Signal _s_F_lat1;
    Signal _s_F_lat2;
    // Signal     _s_q;
    Signal     _s_q0;
    Signal     _s_q1;
    Signal     _s_q2;
    Signal     _s_q3;
    // Signal    _s_dq;
    Signal    _s_dq0;
    Signal    _s_dq1;
    Signal    _s_dq2;
    Signal    _s_dq3;

    // output signal
    // Signal _s_d2q;
    Signal _s_d2q0;
    Signal _s_d2q1;
    Signal _s_d2q2;
    Signal _s_d2q3;

public:
    EquationsOfMotion(Parameters& params) : Base("EquationsOfMotion"), _p(params) {}

    bool init(Parent& parent, const Signals& iports, const Signals& oports) override
    {
        if (!Base::init(parent, iports, oports))
            return false;

        _s_delta = iports[0];
        _s_F_lon = iports[1];
        // _s_F_lat = iports[2];
        _s_F_lat0 = iports[2];
        _s_F_lat1 = iports[3];
        _s_F_lat2 = iports[4];
        // _s_q     = iports[3];
        _s_q0    = iports[5];
        _s_q1    = iports[6];
        _s_q2    = iports[7];
        _s_q3    = iports[8];
        // _s_dq    = iports[4];
        _s_dq0   = iports[9];
        _s_dq1   = iports[10];
        _s_dq2   = iports[11];
        _s_dq3   = iports[12];

        // _s_d2q   = oports[0];
        _s_d2q0  = oports[0];
        _s_d2q1  = oports[1];
        _s_d2q2  = oports[2];
        _s_d2q3  = oports[3];

        return true;
    }

    void activation_function(double /*t*/, Values& values) override
    {
        // inputs
        // double       delta = values[_s_delta][0];
        // double       F_lon = values[_s_F_lon][0];
        double       delta = values[_s_delta];
        double       F_lon = values[_s_F_lon];
        // const Value& F_lat = values[_s_F_lat];
        double Fyf = values[_s_F_lat0];
        double Fyr = values[_s_F_lat1];
        double Fyt = values[_s_F_lat2];
        // const Value&     q = values[    _s_q];
        // double     q0 = values[    _s_q0];
        // double     q1 = values[    _s_q1];
        double     psi1 = values[    _s_q2];
        double     psi2 = values[    _s_q3];
        // const Value&    dq = values[   _s_dq];
        // double    dq0 = values[   _s_dq0];
        // double    dq1 = values[   _s_dq1];
        double    Dpsi1 = values[   _s_dq2];
        double    Dpsi2 = values[   _s_dq3];

        // lateral force components of axles
        // double Fyf = F_lat[0];
        // double Fyr = F_lat[1];
        // double Fyt = F_lat[2];

        // yaw angles and rates
        // double  psi1 =  q[2];
        // double  psi2 =  q[3];
        // double Dpsi1 = dq[2];
        // double Dpsi2 = dq[3];

        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        // % Solve Equations of Motion
        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        // second derivatives
        // Value d2q(4);
        double d2q0, d2q1, d2q2, d2q3;

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
            // Eigen::Matrix4d M;
            // M  <<   _p.m1 + _p.m2 ,                0 ,            _p.lk*_p.m2*s1 ,             _p.ltf*_p.m2*s2
            //     ,               0 ,    _p.m1 + _p.m2 ,           -_p.lk*_p.m2*c1 ,            -_p.ltf*_p.m2*c2
            //     ,  _p.lk*_p.m2*s1 ,  -_p.lk*_p.m2*c1 , _p.m2*_p.lk*_p.lk + _p.I1 ,     _p.lk*_p.ltf*_p.m2*c1_2
            //     , _p.ltf*_p.m2*s2 , -_p.ltf*_p.m2*c2 ,   _p.lk*_p.ltf*_p.m2*c1_2 , _p.m2*_p.ltf*_p.ltf + _p.I2;
            double m00 = _p.m1 + _p.m2;
            double m02 = _p.lk*_p.m2*s1;
            double m03 = _p.ltf*_p.m2*s2;
            double m11 = _p.m1 + _p.m2;
            double m12 = -_p.lk*_p.m2*c1;
            double m13 = -_p.ltf*_p.m2*c2;
            double m20 = _p.lk*_p.m2*s1;
            double m21 = -_p.lk*_p.m2*c1;
            double m22 = _p.m2*_p.lk*_p.lk + _p.I1;
            double m23 = _p.lk*_p.ltf*_p.m2*c1_2;
            double m30 = _p.ltf*_p.m2*s2;
            double m31 = -_p.ltf*_p.m2*c2;
            double m32 = _p.lk*_p.ltf*_p.m2*c1_2;
            double m33 = _p.m2*_p.ltf*_p.ltf + _p.I2;

            // % Vector of Generalized Forces (gyroscopic, Coriolis, centrifugal)
            // % ----------------------------------------------------------------
            // Eigen::Vector4d k;
            // k << _p.m2*(_p.lk*c1*Dpsi1*Dpsi1 + _p.ltf*c2*Dpsi2*Dpsi2)
            //    , _p.m2*(_p.lk*s1*Dpsi1*Dpsi1 + _p.ltf*s2*Dpsi2*Dpsi2)
            //    ,                  Dpsi2*Dpsi2*_p.lk*_p.ltf*_p.m2*s1_2
            //    ,                 -Dpsi1*Dpsi1*_p.lk*_p.ltf*_p.m2*s1_2;
            double k0 = _p.m2*(_p.lk*c1*Dpsi1*Dpsi1 + _p.ltf*c2*Dpsi2*Dpsi2);
            double k1 = _p.m2*(_p.lk*s1*Dpsi1*Dpsi1 + _p.ltf*s2*Dpsi2*Dpsi2);
            double k2 =  Dpsi2*Dpsi2*_p.lk*_p.ltf*_p.m2*s1_2;
            double k3 = -Dpsi1*Dpsi1*_p.lk*_p.ltf*_p.m2*s1_2;

            // % Vector of Generalized Applied Forces
            // % ------------------------------------
            // Eigen::Vector4d qe;
            // qe << F_lon*c1 - Fyf*sin(delta + psi1) - Fyr*s1 - Fyt*s2
            //     , Fyf*cos(delta + psi1) + Fyr*c1 + Fyt*c2 + F_lon*s1
            //     ,  Fyf*_p.lf*cos(delta) - Fyt*_p.lk*c1_2 - Fyr*_p.lr
            //     ,                             -Fyt*(_p.ltf + _p.ltr);
            double qe0 = F_lon*c1 - Fyf*sin(delta + psi1) - Fyr*s1 - Fyt*s2;
            double qe1 = Fyf*cos(delta + psi1) + Fyr*c1 + Fyt*c2 + F_lon*s1;
            double qe2 = Fyf*_p.lf*cos(delta) - Fyt*_p.lk*c1_2 - Fyr*_p.lr;
            double qe3 = -Fyt*(_p.ltf + _p.ltr);;
            
            // % comput new states
            // % -----------------
            // if (0)
            //     d2q = M.colPivHouseholderQr().solve(qe - k);
            // else
            {
                // Eigen::Matrix<double, 4, 5> A;
                // A.block<4, 4>(0, 0) = M;
                // A.block<4, 1>(0, 4) = qe - k;

                // A.row(0) /= A(0, 0);
                m02 /= m00;
                m03 /= m00;
                double qe_k0 = (qe0 - k0)/m00;
                m00 = 1;

                // A.row(1) /= A(1, 1);
                m12 /= m11;
                m13 /= m11;
                double qe_k1 = (qe1 - k1)/m11;
                m11 = 1;

                // A.row(2) -= A.row(0) * A(2, 0);
                m22 -= m02*m20;
                m23 -= m03*m20;
                double qe_k2 = (qe2 - k2) - qe_k0 * m20;
                m20 = 0;

                // A.row(3) -= A.row(0) * A(3, 0);
                m32 -= m02*m30;
                m33 -= m03*m30;
                double qe_k3 = (qe3 - k3) - qe_k0 * m30;
                m30 = 0;

                // A.row(2) -= A.row(1) * A(2, 1);
                m22 -= m12*m21;
                m23 -= m13*m21;
                qe_k2 -= qe_k1*m21;
                m21 = 0;

                // A.row(3) -= A.row(1) * A(3, 1);
                m32 -= m12*m31;
                m33 -= m13*m31;
                qe_k3 -= qe_k1*m31;
                m31 = 0;

                // auto B = A.block<2, 2>(2, 2);
                double b00 = m22;
                double b01 = m23;
                double b10 = m32;
                double b11 = m33;

                // Eigen::Matrix<double, 2, 2> B_inv;
                // B_inv << B(1, 1), -B(0, 1), -B(1, 0), B(0, 0);
                // B_inv /= B(0, 0) * B(1, 1) - B(0, 1) * B(1, 0);
                double det = b00 * b11 - b01 * b10;
                double bi00 =  b11 / det;
                double bi01 = -b01 / det;
                double bi10 = -b10 / det;
                double bi11 =  b00 / det;

                // auto foo = B_inv * A.block<2, 1>(2, 4);
                // d2q.tail(2) = foo;
                // d2q2 = foo(0);
                // d2q3 = foo(1);
                d2q2 = bi00 * qe_k2 + bi01 * qe_k3;
                d2q3 = bi10 * qe_k2 + bi11 * qe_k3;

                // d2q.head(2) = A.block<2, 1>(0, 4) - A.block<2, 2>(0, 2) * foo;
                // auto foo2 = A.block<2, 1>(0, 4) - A.block<2, 2>(0, 2) * foo;
                // d2q0 = foo2(0);
                // d2q1 = foo2(1);
                d2q0 = qe_k0 - m02 * d2q2 - m03 * d2q3;
                d2q1 = qe_k1 - m12 * d2q2 - m13 * d2q3;
            }
        }
        else // % Truck without trailer (bobtail)
        {
            // % comput new states
            // % -----------------
            // d2q << -(Fyf*sin(delta + psi1) - F_lon*c1 + Fyr*s1)/_p.m1
            //      ,  (Fyf*cos(delta + psi1) + Fyr*c1 + F_lon*s1)/_p.m1
            //      ,          -(Fyr*_p.lr - Fyf*_p.lf*cos(delta))/_p.I1
            //      ,                                                  0;
        }

        // values.set(_s_d2q, d2q);
        values.set(_s_d2q0, d2q0);
        values.set(_s_d2q1, d2q1);
        values.set(_s_d2q2, d2q2);
        values.set(_s_d2q3, d2q3);
    }
};

class PT : public Submodel
{
protected:
    Subtract     _sub{"sub"};
    Const      _const;
    Divide       _div{"div"};
    Integrator _integ{"Int"};
    InitialValue  _iv{"IV"};
    Add          _add{"add"};

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
    // Integrator     _integ1{"d2q->dq", Value::Zero(4)};
    Integrator     _integ10{"d2q0->dq0"};
    Integrator     _integ11{"d2q1->dq1"};
    Integrator     _integ12{"d2q2->dq2"};
    Integrator     _integ13{"d2q3->dq3"};
    // Integrator     _integ2{  "dq->q", Value::Zero(4)};
    Integrator     _integ20{  "dq0->q0"};
    Integrator     _integ21{  "dq1->q1"};
    Integrator     _integ22{  "dq2->q2"};
    Integrator     _integ23{  "dq3->q3"};

public:
    ChassisDynamics(Parameters& params) : Submodel("ChassisDynamics"), _eom(params) {}

    bool init(Parent& parent, const Signals& iports, const Signals& oports) override
    {
        if (!Submodel::init(parent, iports, oports))
            return false;

        // signals
        auto s_delta = signal("front_wheel_angle");
        // auto     s_q = signal(  "q");
        auto     s_q0 = signal(  "q0");
        auto     s_q1 = signal(  "q1");
        auto     s_q2 = signal(  "q2");
        auto     s_q3 = signal(  "q3");
        // auto    s_dq = signal( "dq");
        auto    s_dq0 = signal( "dq0");
        auto    s_dq1 = signal( "dq1");
        auto    s_dq2 = signal( "dq2");
        auto    s_dq3 = signal( "dq3");
        // auto   s_d2q = signal("d2q");
        auto   s_d2q0 = signal("d2q0");
        auto   s_d2q1 = signal("d2q1");
        auto   s_d2q2 = signal("d2q2");
        auto   s_d2q3 = signal("d2q3");
        auto s_F_lon = signal("F_lon");
        // auto s_F_lat = signal("F_lat");
        auto s_F_lat0 = signal("F_lat0");
        auto s_F_lat1 = signal("F_lat1");
        auto s_F_lat2 = signal("F_lat2");

        auto&      s_delta_Rq = iports[0];
        auto& s_engine_torque = iports[1];

        add_block(    _pt,               {s_delta_Rq, s_delta_Rq},           s_delta );
        // add_block(_forces,       {s_delta, s_engine_torque, s_dq}, {s_F_lon, s_F_lat});
        add_block(_forces,       {s_delta, s_engine_torque, s_dq0, s_dq1}, {s_F_lon, s_F_lat0, s_F_lat1, s_F_lat2});
        // add_block(   _eom, {s_delta, s_F_lon, s_F_lat, s_q, s_dq},             s_d2q );
        add_block(   _eom, {s_delta, s_F_lon, s_F_lat0, s_F_lat1, s_F_lat2, s_q0, s_q1, s_q2, s_q3, s_dq0, s_dq1, s_dq2, s_dq3}, {s_d2q0, s_d2q1, s_d2q2, s_d2q3} );
        // add_block(_integ1,                                 s_d2q ,              s_dq );
        add_block(_integ10,                                 s_d2q0 ,              s_dq0 );
        add_block(_integ11,                                 s_d2q1 ,              s_dq1 );
        add_block(_integ12,                                 s_d2q2 ,              s_dq2 );
        add_block(_integ13,                                 s_d2q3 ,              s_dq3 );
        // add_block(_integ2,                                  s_dq ,               s_q );
        add_block(_integ20,                                  s_dq0 ,               s_q0 );
        add_block(_integ21,                                  s_dq1 ,               s_q1 );
        add_block(_integ22,                                  s_dq2 ,               s_q2 );
        add_block(_integ23,                                  s_dq3 ,               s_q3 );

        return true;
    }
};

class SimpleTruckModel : public Model
{
protected:
    // parameters
    Parameters _params;
    ChassisDynamics _chdyn;

    // input signals
    Signal _s_delta_Rq;
    Signal _s_engine_torque;

public:
    SimpleTruckModel() : Model("SimpleTruck"), _chdyn(_params)
    {
        // input signals
        _s_delta_Rq      = signal("front_wheel_angle_Rq");
        _s_engine_torque = signal("engine_torque");

        // setup the model
        add_block(_chdyn, {_s_delta_Rq, _s_engine_torque});
    }

    void input_cb(double t, Values& values) override
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

        values.set(     _s_delta_Rq, delta_Rq);
        values.set(_s_engine_torque,     5000);
    }
};

int main()
{
    using milli = std::chrono::milliseconds;
    auto  start = std::chrono::high_resolution_clock::now();

    SimpleTruckModel model;

    History history(model);

    Simulator sim(model, nullptr, rk4);

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
    // auto dq = history.at(model.find_signal(".dq"));
    auto a = history.at(model.find_signal("_angle"));

    // const auto& vx = dq.col(0);
    const auto& vx = history.at(model.find_signal(".dq0"));
    // const auto& vy = dq.col(1);
    const auto& vy = history.at(model.find_signal(".dq1"));
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
