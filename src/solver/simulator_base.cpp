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

#include <unordered_set>

#include "simulator_base.hpp"
#include "src/block/block.hpp"
#include "src/helper/util.hpp"

namespace pooya
{

SimulatorBase::SimulatorBase(Block& model, InputCallback inputs_cb, StepperBase* stepper)
    : _model(model), _inputs_cb(inputs_cb), _stepper(stepper)
{
    pooya_trace("model: " + model.full_name().str());
}

void SimulatorBase::init(double t0)
{
    pooya_trace("t0: " + std::to_string(t0));

    std::size_t state_variables_size{0};

    std::unordered_set<ValueSignalImpl*> value_signals;
    std::unordered_set<ScalarSignalImpl*> scalar_state_signals;
#ifdef POOYA_ARRAY_SIGNAL
    std::unordered_set<ArraySignalImpl*> array_state_signals;
#endif // POOYA_ARRAY_SIGNAL

    _model.visit(
        [&](Block& block, uint32_t /*level*/) -> bool
        {
            const auto& signals = block.linked_signals();
            for (auto& sig : signals)
            {
                value_signals.insert(sig.first.get());

                if (auto* ps = dynamic_cast<ScalarSignalImpl*>(sig.first.get()); ps)
                {
                    if (ps->state_variable() &&
                        scalar_state_signals
                            .insert(std::static_pointer_cast<ScalarSignalImpl>(ps->shared_from_this()).get())
                            .second)
                    {
                        state_variables_size++;
                    }
                }
#ifdef POOYA_ARRAY_SIGNAL
                else if (auto* pa = dynamic_cast<ArraySignalImpl*>(sig.first.get()); pa)
                {
                    if (pa->state_variable() &&
                        array_state_signals
                            .insert(std::static_pointer_cast<ArraySignalImpl>(pa->shared_from_this()).get())
                            .second)
                    {
                        state_variables_size += pa->size();
                    }
                }
#endif // POOYA_ARRAY_SIGNAL
            }

            return true;
        },
        0);

    value_signals_.reserve(value_signals.size());
    for (auto* sig : value_signals)
    {
        value_signals_.emplace_back(std::static_pointer_cast<ValueSignalImpl>(sig->shared_from_this()));
    }
    scalar_state_signals_.reserve(scalar_state_signals.size());
    for (auto* sig : scalar_state_signals)
    {
        scalar_state_signals_.emplace_back(std::static_pointer_cast<ScalarSignalImpl>(sig->shared_from_this()));
    }
#ifdef POOYA_ARRAY_SIGNAL
    array_state_signals_.reserve(array_state_signals.size());
    for (auto* sig : array_state_signals)
    {
        array_state_signals_.emplace_back(std::static_pointer_cast<ArraySignalImpl>(sig->shared_from_this()));
    }
#endif // POOYA_ARRAY_SIGNAL

    _state_variables.resize(state_variables_size);
    _state_variables_orig.resize(state_variables_size);
    _state_variable_derivs.resize(state_variables_size);

    _t_prev = t0;

    if (state_variables_size == 0 && _stepper)
    {
        helper::pooya_show_warning(
            __FILE__, __LINE__, "A stepper is provided but no state variable is defined! The stepper will be ignored.");
    }

    for (auto& sig : value_signals_)
    {
        sig->clear();
    }

    _initialized = true;
}

void SimulatorBase::run(double t, double min_time_step, double max_time_step)
{
    pooya_trace("t: " + std::to_string(t));

    auto minor_solver_step_cb = [&](double t, const Array& state_variables) -> const Array&
    {
        pooya_trace("t: " + std::to_string(t));

        reset_with_state_variables(state_variables);
        process_model(t, false, false);

        double* data = _state_variable_derivs.data();
        for (auto& sig : scalar_state_signals_)
        {
            *data = sig->deriv_signal()->get_value();
            data++;
        }
#ifdef POOYA_ARRAY_SIGNAL
        for (auto& sig : array_state_signals_)
        {
            auto* deriv_sig                            = sig->deriv_signal();
            Eigen::Map<Array>(data, deriv_sig->size()) = deriv_sig->get_value();
            data += deriv_sig->size();
        }
#endif // POOYA_ARRAY_SIGNAL

        return _state_variable_derivs;
    };

    if (!_initialized)
    {
        // treat the first call as the initialization call if init(t0) was not called explicitely
        init(t);
        return;
    }

    if (_state_variables.size() > 0)
    {
        if (t < _t_prev)
        {
            helper::pooya_throw_exception(__FILE__, __LINE__,
                                          "Simulation cannot go back in time. Aborting!\n  "
                                          "- current time  = " +
                                              std::to_string(t) +
                                              "\n  "
                                              "- previous time = " +
                                              std::to_string(_t_prev) + "\n");
        }

        if (t == _t_prev)
        {
            helper::pooya_show_warning(__FILE__, __LINE__,
                                       "Repeated simulation step:\n  "
                                       "- time = " +
                                           std::to_string(t) + "\n");

            for (auto& sig : value_signals_)
            {
                sig->clear();
            }
            if (_inputs_cb)
            {
                _inputs_cb(_model, t);
            }
            _model.input_cb(t);
            _model.pre_step(t);
            get_state_variables(_state_variables);
        }
        else
        {
            pooya_debug_verify0(_stepper);

            pooya_debug_verify0(min_time_step > 0);
            pooya_debug_verify0(max_time_step > min_time_step);

            double new_h;
            double t1         = _t_prev;
            double t2         = t;
            bool force_accept = false;
            while (t1 < t)
            {
                for (auto& sig : value_signals_)
                {
                    sig->clear();
                }
                if (_inputs_cb)
                {
                    _inputs_cb(_model, t1);
                }
                _model.input_cb(t1);
                _model.pre_step(t1);
                get_state_variables(_state_variables_orig);

                _stepper->step(minor_solver_step_cb, t1, _state_variables_orig, t2, _state_variables, new_h);

                double h = t2 - t1;
                if (force_accept || (new_h >= h) || (h <= min_time_step))
                {
                    // accept this step
                    force_accept = false;
                    new_h        = std::max(min_time_step, std::min(new_h, max_time_step));
                    t1           = t2;
                    t2           = std::min(t1 + new_h, t);

                    if (t1 < t)
                    {
                        reset_with_state_variables(_state_variables);
                        process_model(t1, false, true);
                    }
                }
                else
                {
                    // redo this step
                    force_accept = new_h <= min_time_step;
                    new_h        = std::max(min_time_step, std::min(new_h, max_time_step));
                    t2           = t1 + new_h;
                }
            }
        }
    }

    reset_with_state_variables(_state_variables);
    process_model(t, _state_variables.size() == 0, true);

    _t_prev = t;
}

void SimulatorBase::reset_with_state_variables(const Array& state_variables)
{
    pooya_trace0;
    for (auto& sig : value_signals_)
    {
        sig->clear();
    }
    const double* data = state_variables.data();
    for (auto& sig : scalar_state_signals_)
    {
        sig->set_value(*data);
        data++;
    }
#ifdef POOYA_ARRAY_SIGNAL
    for (auto& sig : array_state_signals_)
    {
        sig->set_value(Eigen::Map<const Array>(data, sig->size()));
        data += sig->size();
    }
#endif // POOYA_ARRAY_SIGNAL
}

void SimulatorBase::get_state_variables(Array& state_variables)
{
    pooya_trace0;
    pooya_debug_verify(state_variables.size() == _state_variables.size(), "Incorrect output array size!");
    double* data = state_variables.data();
    for (auto& sig : scalar_state_signals_)
    {
        *data = sig->get_value();
        data++;
    }
#ifdef POOYA_ARRAY_SIGNAL
    for (auto& sig : array_state_signals_)
    {
        Eigen::Map<Array>(data, sig->size()) = sig->get_value();
        data += sig->size();
    }
#endif // POOYA_ARRAY_SIGNAL
}

} // namespace pooya
