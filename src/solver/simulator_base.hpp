/*
Copyright 2024 Mojtaba (Moji) Fathi

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
documentation files (the “Software”), to deal in the Software without restriction, including without limitation the
rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit
persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
Software.

 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __POOYA_SOLVER_SIMULATOR_BASE_HPP__
#define __POOYA_SOLVER_SIMULATOR_BASE_HPP__

#include <functional>
#include <memory>
#include <vector>

#include "src/helper/defs.hpp" // should be the first include
#include "src/signal/array.hpp"
#include "stepper_base.hpp"

#if !defined(VERIFY_SIMULATOR)
#define VERIFY_SIMULATOR POOYA_DEBUG
#endif // !defined(VERIFY_SIMULATOR)

namespace pooya
{

class ValueSignalImpl;
class ScalarSignalImpl;
class ArraySignalImpl;
class Block;

class SimulatorBase
{
public:
    using InputCallback = std::function<void(Block&, double)>;

    explicit SimulatorBase(Block& model, InputCallback inputs_cb = nullptr, StepperBase* stepper = nullptr);
    SimulatorBase(const SimulatorBase&) = delete; // no copy constructor
    virtual ~SimulatorBase()            = default;

    virtual void init(double t0 = 0.0);
    virtual void run(double t, double min_time_step = 1e-3, double max_time_step = 1);

protected:
    Block& _model;
    double _t_prev{0};
    InputCallback _inputs_cb;
    std::vector<std::shared_ptr<ValueSignalImpl>> value_signals_;
    std::vector<std::shared_ptr<ScalarSignalImpl>> scalar_state_signals_;
#if DEFINE_ARRAY_SIGNAL != 0
    std::vector<std::shared_ptr<ArraySignalImpl>> array_state_signals_;
#endif // DEFINE_ARRAY_SIGNAL != 0
    Array _state_variables;
    Array _state_variables_orig;
    Array _state_variable_derivs;
    StepperBase* _stepper{nullptr};
    bool _initialized{false};

    void reset_with_state_variables(const Array& state_variables);
    void get_state_variables(Array& state_variables);

    virtual void process_model(double t, bool call_pre_step, bool call_post_step) = 0;
};

} // namespace pooya

#endif // __POOYA_SOLVER_SIMULATOR_BASE_HPP__
