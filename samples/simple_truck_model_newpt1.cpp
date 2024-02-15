
#include <chrono>
#include <iostream>
#include <math.h>
#include <vector>

#include "3rdparty/eigen/Eigen/Dense"

#include "src/core/helper.hpp"
#include "src/core/pooya.hpp"
#include "src/core/solver.hpp"
#include "src/misc/gp-ios.hpp"

struct Parameters {
  // primary parameters
  bool has_trailer{true};
  double tractor_m = 8760;
  double tractor_Izz = 55649;
  double trailer_m = 6114;
  double trailer_Izz = 107407;
  double tractor_lf = 2.585;
  double tractor_lr = 3.2475;
  double trailer_lf = 8.1400;
  double trailer_lr = 3.425;
  double payload_m_max = 21413;
  double payload_Izz_max = 261401;
  double payload_factor = 1.0;
  double tractor_rear_axle_to_hitch = 0.15;
  double tractor_lk = tractor_lr - tractor_rear_axle_to_hitch;
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

  Parameters() { update(); }

  void update() {
    m1 = tractor_m;
    I1 = tractor_Izz;

    lf = tractor_lf;
    lr = tractor_lr;
    lk = tractor_lk;

    if (has_trailer) {
      double payload_m = payload_factor * payload_m_max;
      double payload_Izz = payload_factor * payload_Izz_max;
      double lp = payload_cog_to_trailer_axle;

      m2 = trailer_m + payload_m;
      ltr = (lp * payload_m + trailer_lr * trailer_m) / m2;
      ltf = trailer_lf + trailer_lr - ltr;
      I2 = trailer_Izz + trailer_m * std::pow(ltr - trailer_lr, 2) +
           payload_Izz + payload_m * std::pow(ltr - lp, 2);
    } else {
      m2 = 0;
      ltr = 0;
      ltf = 0;
      I2 = 0;
    }
  }
};

class EquationsOfMotion : public pooya::Block
{
protected:
  Parameters &_p;

  // input signals
  pooya::ScalarSignalId _s_delta_rq;      // [0]
  pooya::ScalarSignalId _s_engine_torque; // [1]
  pooya::ArraySignalId _s_q;              // [2]
  pooya::ArraySignalId _s_dq;             // [3]

  // output signal
  pooya::ArraySignalId _s_d2q;

  // internal signal of interest
  pooya::ScalarSignalId _s_delta;

  pooya::util::PT1 _pt;

public:
  EquationsOfMotion(Parameters &params)
      : pooya::Block("EquationsOfMotion", 4, 1), _p(params), _pt(0.1) {}

  bool init(pooya::Parent &parent, pooya::BusId ibus, pooya::BusId obus) override
  {
    if (!pooya::Block::init(parent, ibus, obus))
      return false;

    // input signals
    _s_delta_rq = scalar_input_at("delta_rq");
    _s_engine_torque = scalar_input_at("engine_torque");
    _s_q = array_input_at("q");
    _s_dq = array_input_at("dq");

    // output signal
    _s_d2q = array_output_at("d2q");

    // internal signal of interest
    _s_delta = parent.create_scalar_signal("front_wheel_angle");

    return true;
  }

  void activation_function(double t, pooya::Values &values) override {
    // get inputs
    double delta_rq = values.get(_s_delta_rq);
    double engine_torque = values.get(_s_engine_torque);
    const pooya::Array &q = values.get(_s_q);
    const pooya::Array &dq = values.get(_s_dq);

    // do the math

    double delta = _pt(t, delta_rq);
    values.set(_s_delta, delta);

    // forces

    double F =
        engine_torque *
        (0.5 * 5 * 2.41 / 0.5); // efficiency * gear_ratio * diff_ratio / R_tire
    double F_lon = F * cos(delta) - 500 * dq(0);

    // lateral force components of axles
    double Fyf = F * sin(delta) - 500 * dq(1);
    double Fyr = 0.0;
    double Fyt = 0.0;

    // yaw angles and rates
    double psi1 = q[2];
    double psi2 = q[3];
    double Dpsi1 = dq[2];
    double Dpsi2 = dq[3];

    // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // % Solve Equations of Motion
    // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    // second derivatives
    pooya::Array4 d2q;

    double s1 = sin(psi1);
    double c1 = cos(psi1);

    if (_p.has_trailer) // Truck-Trailer Combination
    {
      double s2 = sin(psi2);
      double c2 = cos(psi2);
      double s1_2 = sin(psi1 - psi2);
      double c1_2 = cos(psi1 - psi2);

      // % Generalized Mass Matrix
      // % -----------------------
      Eigen::Matrix4d M;
      M << _p.m1 + _p.m2, 0, _p.lk * _p.m2 * s1, _p.ltf * _p.m2 * s2, 0,
          _p.m1 + _p.m2, -_p.lk * _p.m2 * c1, -_p.ltf * _p.m2 * c2,
          _p.lk * _p.m2 * s1, -_p.lk * _p.m2 * c1,
          _p.m2 * _p.lk * _p.lk + _p.I1, _p.lk * _p.ltf * _p.m2 * c1_2,
          _p.ltf * _p.m2 * s2, -_p.ltf * _p.m2 * c2,
          _p.lk * _p.ltf * _p.m2 * c1_2, _p.m2 * _p.ltf * _p.ltf + _p.I2;

      // % Vector of Generalized Forces (gyroscopic, Coriolis, centrifugal)
      // % ----------------------------------------------------------------
      Eigen::Vector4d k;
      k << _p.m2 * (_p.lk * c1 * Dpsi1 * Dpsi1 + _p.ltf * c2 * Dpsi2 * Dpsi2),
          _p.m2 * (_p.lk * s1 * Dpsi1 * Dpsi1 + _p.ltf * s2 * Dpsi2 * Dpsi2),
          Dpsi2 * Dpsi2 * _p.lk * _p.ltf * _p.m2 * s1_2,
          -Dpsi1 * Dpsi1 * _p.lk * _p.ltf * _p.m2 * s1_2;

      // % Vector of Generalized Applied Forces
      // % ------------------------------------
      Eigen::Vector4d qe;
      qe << F_lon * c1 - Fyf * sin(delta + psi1) - Fyr * s1 - Fyt * s2,
          Fyf * cos(delta + psi1) + Fyr * c1 + Fyt * c2 + F_lon * s1,
          Fyf * _p.lf * cos(delta) - Fyt * _p.lk * c1_2 - Fyr * _p.lr,
          -Fyt * (_p.ltf + _p.ltr);

      // % comput new states
      // % -----------------
      if (1)
        d2q = M.colPivHouseholderQr().solve(qe - k);
      else {
        Eigen::Matrix<double, 4, 5> A;
        A.block<4, 4>(0, 0) = M;
        A.block<4, 1>(0, 4) = qe - k;

        A.row(0) /= A(0, 0);
        A.row(1) /= A(1, 1);
        A.row(2) -= A.row(0) * A(2, 0);
        A.row(3) -= A.row(0) * A(3, 0);
        A.row(2) -= A.row(1) * A(2, 1);
        A.row(3) -= A.row(1) * A(3, 1);
        auto B = A.block<2, 2>(2, 2);
        Eigen::Matrix<double, 2, 2> B_inv;
        B_inv << B(1, 1), -B(0, 1), -B(1, 0), B(0, 0);
        B_inv /= B(0, 0) * B(1, 1) - B(0, 1) * B(1, 0);
        auto foo = B_inv * A.block<2, 1>(2, 4);
        d2q.tail(2) = foo;
        d2q.head(2) = A.block<2, 1>(0, 4) - A.block<2, 2>(0, 2) * foo;
      }
    } else // % Truck without trailer (bobtail)
    {
      // % comput new states
      // % -----------------
      d2q << -(Fyf * sin(delta + psi1) - F_lon * c1 + Fyr * s1) / _p.m1,
          (Fyf * cos(delta + psi1) + Fyr * c1 + F_lon * s1) / _p.m1,
          -(Fyr * _p.lr - Fyf * _p.lf * cos(delta)) / _p.I1, 0;
    }

    // set output
    values.set(_s_d2q, d2q);
  }

  void post_step(double t, const pooya::Values &values) override
  {
    Block::post_step(t, values);
    _pt.step(t, values.get(_s_delta_rq));
  }
};

class ChassisDynamics : public pooya::Submodel {
protected:
  EquationsOfMotion _eom;
  pooya::IntegratorA _integ1{"dq", pooya::Array4::Zero()};
  pooya::IntegratorA _integ2{"q", pooya::Array4::Zero()};

public:
  ChassisDynamics(Parameters &params)
      : pooya::Submodel("ChassisDynamics"), _eom(params) {}

  bool init(pooya::Parent &parent, pooya::BusId ibus,
            pooya::BusId obus) override {
    if (!pooya::Submodel::init(parent, ibus, obus))
      return false;

    // signals
    auto s_q   = create_array_signal("q", 4);
    auto s_dq  = create_array_signal("dq", 4);
    auto s_d2q = create_array_signal("d2q", 4);

    add_block(_eom,
      {
        {"delta_rq", scalar_input_at("delta_rq")},
        {"engine_torque", scalar_input_at("engine_torque")},
        {"q", s_q},
        {"dq", s_dq},
      }, {{"d2q", s_d2q}});
    add_block(_integ1, s_d2q, s_dq);
    add_block(_integ2, s_dq, s_q);

    return true;
  }
};

class SimpleTruckModel : public pooya::Model {
protected:
  // parameters
  Parameters _params;
  ChassisDynamics _chdyn;

  // input signals
  pooya::ScalarSignalId _s_delta_Rq;
  pooya::ScalarSignalId _s_engine_torque;

public:
  SimpleTruckModel() : pooya::Model("SimpleTruck"), _chdyn(_params) {
    // input signals
    _s_delta_Rq = create_scalar_signal("front_wheel_angle_Rq");
    _s_engine_torque = create_scalar_signal("engine_torque");

    // setup the model
    add_block(_chdyn, {{"delta_rq", _s_delta_Rq}, {"engine_torque", _s_engine_torque}});
  }

  void input_cb(double t, pooya::Values &values) override {
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

    values.set(_s_delta_Rq, delta_Rq);
    values.set(_s_engine_torque, 5000.0);
  }
};

int main() {
  using milli = std::chrono::milliseconds;
  auto start = std::chrono::high_resolution_clock::now();

  SimpleTruckModel model;

  pooya::Rk4 stepper(model);
  pooya::Simulator sim(model, nullptr, &stepper);

  pooya::History history(model);

  uint k = 0;
  double t;
  while (pooya::arange(k, t, 0, 100, 0.01)) {
    sim.run(t);
    history.update(k, t, sim.values());
    k++;
  }

  history.shrink_to_fit();

  auto finish = std::chrono::high_resolution_clock::now();
  std::cout << "It took "
            << std::chrono::duration_cast<milli>(finish - start).count()
            << " milliseconds\n";

  history.export_csv("simple_truck_model_newpt1.csv");

  const auto &T = history.time();
  auto dq = history.at(model.lookup_signal("~dq"));
  auto a = history.at(model.lookup_signal("_angle"));

  const auto &vx = dq.col(0);
  const auto &vy = dq.col(1);

  Gnuplot gp;
  gp << "set xrange [0:" << T.size() - 1 << "]\n";
  gp << "set yrange [-20:50]\n";
  gp << "plot" << gp.file1d(vx) << "with lines title 'vx', " << gp.file1d(vy)
     << "with lines title 'vy', "
        "\n";

  return 0;
}
