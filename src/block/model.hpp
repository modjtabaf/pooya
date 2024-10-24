/*
Copyright 2024 Mojtaba (Moji) Fathi

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”),
to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __POOYA_BLOCK_MODEL_HPP__
#define __POOYA_BLOCK_MODEL_HPP__

#include "src/signal/scalar_signal.hpp"
#include "src/signal/int_signal.hpp"
#include "src/signal/bool_signal.hpp"
#include "src/signal/array_signal.hpp"
#include "parent.hpp"

namespace pooya
{

class Model : public Parent
{
    friend class Parent;

public:
    // using SignalInfos = std::vector<SignalId>;

protected:
    // SignalInfos _signal_infos;

    bool init(Parent&, BusId, BusId) override;

    // template<typename T, typename... Ts>
    // typename Types<T>::SignalId register_signal(const std::string& name, Ts... args)
    // {
    //     pooya_trace("block: " + full_name() + ", given name: " + name);
    //     if (name.empty()) {return typename Types<T>::SignalId();}

    //     pooya_verify(!lookup_signal(name, true), "Re-registering a signal is not allowed!");

    //     auto index = _signal_infos.size();
    //     typename Types<T>::SignalId sig;
    //     sig = Types<T>::SignalInfo::create_new(name, index, args...);
    //     _signal_infos.emplace_back(sig);

    //     return Types<T>::as_signal_id(_signal_infos.back());
    // }

    // ScalarSignalId register_scalar_signal(const std::string& name)
    // {
    //     pooya_trace("block: " + full_name() + ", given name: " + name);
    //     return register_signal<double>(name);
    // }
    // IntSignalId register_int_signal(const std::string& name)
    // {
    //     pooya_trace("block: " + full_name() + ", given name: " + name);
    //     return register_signal<int>(name);
    // }
    // BoolSignalId register_bool_signal(const std::string& name)
    // {
    //     pooya_trace("block: " + full_name() + ", given name: " + name);
    //     return register_signal<bool>(name);
    // }
    // ArraySignalId register_array_signal(const std::string& name, std::size_t size)
    // {
    //     pooya_trace("block: " + full_name() + ", given name: " + name);
    //     return register_signal<Array, std::size_t>(name, size);
    // }
    // BusId register_bus(const std::string& name, const BusSpec& spec, LabelSignals::const_iterator begin_, LabelSignals::const_iterator end_)
    // {
    //     pooya_trace("block: " + full_name() + ", given name: " + name);
    //     return register_signal<BusSpec, const BusSpec&, LabelSignals::const_iterator, LabelSignals::const_iterator>(name, spec, begin_, end_);
    // }

public:
    Model(const std::string& given_name="model");

    virtual void input_cb(double /*t*/) {}

    // Model* model() override {return this;}
    // const SignalInfos& signals() const {return _signal_infos;}

    // void register_state_variable(FloatSignalId sig, FloatSignalId deriv_sig);
    // SignalId lookup_signal(const std::string& name, bool exact_match=false) const;
    // void invalidate();
};

}

#endif // __POOYA_BLOCK_MODEL_HPP__
