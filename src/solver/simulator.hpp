/*
Copyright 2024 Mojtaba (Moji) Fathi

 Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the “Software”), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __POOYA_SOLVER_SIMULATOR_HPP__
#define __POOYA_SOLVER_SIMULATOR_HPP__

#include <functional>
#include <unordered_set>
#include <vector>

#include "src/block/block.hpp"
#include "src/signal/array.hpp"
#include "stepper_base.hpp"

namespace pooya
{

using InputCallback = std::function<void(Block&, double)>;

class Simulator
{
protected:
    Block& _model;
    double _t_prev{0};
    InputCallback _inputs_cb;
    std::vector<ValueSignalId> value_signals_;
    std::vector<ScalarSignalId> scalar_state_signals_;
    std::vector<ArraySignalId> array_state_signals_;
    Array _state_variables;
    Array _state_variables_orig;
    Array _state_variable_derivs;
#if defined(POOYA_USE_SMART_PTRS)
    std::optional<std::reference_wrapper<StepperBase>> _stepper;
#else  // defined(POOYA_USE_SMART_PTRS)
    StepperBase* _stepper{nullptr};
#endif // defined(POOYA_USE_SMART_PTRS)
    bool _initialized{false};

    const bool _reuse_order;
#if defined(POOYA_USE_SMART_PTRS)
    using ProcessingOrder = std::vector<std::reference_wrapper<Block>>;
#else  // defined(POOYA_USE_SMART_PTRS)
    using ProcessingOrder = std::vector<Block*>;
#endif // defined(POOYA_USE_SMART_PTRS)
    ProcessingOrder _processing_order1;
    ProcessingOrder _processing_order2;
#if defined(POOYA_USE_SMART_PTRS)
    std::reference_wrapper<ProcessingOrder> _current_po{_processing_order1};
    std::reference_wrapper<ProcessingOrder> _new_po{_processing_order2};
#else  // defined(POOYA_USE_SMART_PTRS)
    ProcessingOrder* _current_po{nullptr};
    ProcessingOrder* _new_po{nullptr};
#endif // defined(POOYA_USE_SMART_PTRS)

    uint _process(double t);
    void reset_with_state_variables(const Array& state_variables);
    void get_state_variables(Array& state_variables);

public:
    Simulator(Block& model, InputCallback inputs_cb = nullptr, StepperBase* stepper = nullptr,
              bool reuse_order = false);
    Simulator(const Simulator&) = delete; // no copy constructor

    void init(double t0 = 0.0);
    void run(double t, double min_time_step = 1e-3, double max_time_step = 1);
};

bool arange(uint k, double& t, double t_init, double t_end, double dt);

} // namespace pooya

#endif // __POOYA_SOLVER_SIMULATOR_HPP__
