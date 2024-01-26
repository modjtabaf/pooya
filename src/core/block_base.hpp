/*
Copyright 2023 Mojtaba (Moji) Fathi

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”),
to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __POOYA_BLOCK_BASE_HPP__
#define __POOYA_BLOCK_BASE_HPP__

#include <functional>

#include "util.hpp"
#include "signal.hpp"

namespace pooya
{

class Block;
class Parent;
class Model;

using TraverseCallback = std::function<bool(Block&, uint32_t level)>;

class Block
{
    friend class Parent;

public:
    static constexpr uint16_t NoIOLimit = uint16_t(-1);

protected:
    bool _initialized{false};
    LabelSignals _iports;
    LabelSignals _oports;
    std::vector<Signal> _dependencies;
    Parent* _parent{nullptr};
    std::string _given_name;
    std::string _full_name;
    uint16_t _num_iports{NoIOLimit};
    uint16_t _num_oports{NoIOLimit};
    std::size_t _unnamed_signal_counter{0};

    bool _processed{false};
    void _assign_valid_given_name(std::string given_name);
    bool _add_dependecny(Signal signal);

    Block(std::string given_name, uint16_t num_iports=NoIOLimit, uint16_t num_oports=NoIOLimit) :
        _given_name(given_name), _num_iports(num_iports), _num_oports(num_oports) {}

    virtual bool init(Parent& parent, const LabelSignals& iports=LabelSignals(), const LabelSignals& oports=LabelSignals());
    virtual void post_init() {}

public:
    virtual ~Block() = default;

    virtual void step(double /*t*/, const Values& /*states*/) {}
    virtual void activation_function(double /*t*/, Values& /*x*/) {}
    virtual Model* model();

    Model& model_ref()
    {
        pooya_trace("block: " + full_name());
        auto* mdl = model();
        verify(mdl, _full_name + ": a model is necessary but none is defined!");
        return *mdl;
    }

    Parent* parent() {return _parent;}
    bool processed() const {return _processed;}
    bool is_initialized() const {return _initialized;}
    const std::string& given_name() const {return _given_name;}
    const std::string& full_name() const {return _full_name;}
    const LabelSignals& iports() const {return _iports;}
    const LabelSignals& oports() const {return _oports;}

    virtual void _mark_unprocessed();
    virtual uint _process(double t, Values& values, bool go_deep = true);

    virtual bool traverse(TraverseCallback cb, uint32_t level, uint32_t max_level=std::numeric_limits<uint32_t>::max())
    {
        pooya_trace("block: " + full_name());
        return (level > max_level) || cb(*this, level);
    }
}; // class Block

class Parent : public Block
{
protected:
    std::vector<Block*> _components;

    Parent(std::string given_name, uint16_t num_iports=NoIOLimit, uint16_t num_oports=NoIOLimit) :
        Block(given_name, num_iports, num_oports) {}

    template<typename Iter>
    BusSignal create_bus(const std::string& given_name, const BusSpec& spec, Iter begin_, Iter end_);

public:
    bool add_block(Block& component, const LabelSignals& iports={}, const LabelSignals& oports={})
    {
        pooya_trace("block: " + full_name());
        if (!component.init(*this, iports, oports))
            return false;

        _components.push_back(&component);
        component.post_init();
        return true;
    }

    void step(double t, const Values& values) override
    {
        pooya_trace("block: " + full_name());
        for (auto* component: _components)
            component->step(t, values);
    }

    // retrieve an existing signal
    Signal get_generic_signal(const std::string& given_name);
    template<typename T=double>
    typename Types<T>::Signal get_signal(const std::string& given_name);
    ArraySignal get_signal(const std::string& given_name, std::size_t size);
    BusSignal get_bus(const std::string& given_name, const BusSpec& spec);

    // get if exists, create otherwise
    ArraySignal signal(const std::string& given_name, std::size_t size);
    BusSignal bus(const std::string& given_name, const BusSpec& spec);

    template<typename T=double>
    typename Types<T>::Signal signal(const std::string& given_name="")
    {
        pooya_trace("block: " + full_name());
        auto sig = get_signal<T>(given_name);
        return sig ? sig : create_signal<T>(given_name);
    }

    // create with a unique name
    template<typename T=double>
    typename Types<T>::Signal create_signal(const std::string& given_name="");
    ArraySignal create_signal(const std::string& given_name, std::size_t size);
    BusSignal create_bus(const std::string& given_name, const BusSpec& spec, const std::initializer_list<LabelSignal>& l);
    BusSignal create_bus(const std::string& given_name, const BusSpec& spec, const std::initializer_list<Signal>& l={});

    // clone (make a new copy)
    Signal clone_signal(const std::string& given_name, Signal sig);
    ScalarSignal clone_signal(const std::string& given_name, ScalarSignal sig)
    {
        pooya_trace("block: " + full_name());
        return clone_signal(given_name, Signal(sig))->as_scalar();
    }
    IntegerSignal clone_signal(const std::string& given_name, IntegerSignal sig)
    {
        pooya_trace("block: " + full_name());
        return clone_signal(given_name, Signal(sig))->as_integer();
    }
    ArraySignal clone_signal(const std::string& given_name, ArraySignal sig)
    {
        pooya_trace("block: " + full_name());
        return clone_signal(given_name, Signal(sig))->as_array();
    }
    BusSignal clone_bus(const std::string& given_name, BusSignal sig)
    {
        pooya_trace("block: " + full_name());
        return clone_signal(given_name, Signal(sig))->as_bus();
    }

    std::string make_signal_name(const std::string& given_name, bool make_new=false);
    void _mark_unprocessed() override;
    uint _process(double t, Values& values, bool go_deep = true) override;
    bool traverse(TraverseCallback cb, uint32_t level, uint32_t max_level=std::numeric_limits<uint32_t>::max()) override;
}; // class Parent

template<>
typename Types<double>::Signal Parent::get_signal<double>(const std::string& given_name);

template<>
typename Types<int>::Signal Parent::get_signal<int>(const std::string& given_name);

template<>
typename Types<double>::Signal Parent::create_signal<double>(const std::string& given_name);

template<>
typename Types<int>::Signal Parent::create_signal<int>(const std::string& given_name);

class Submodel : public Parent
{
public:
    Submodel(std::string given_name, uint16_t num_iports=NoIOLimit, uint16_t num_oports=NoIOLimit) : Parent(given_name, num_iports, num_oports) {}

}; // class Submodel

class Model : public Parent
{
    friend class Parent;

public:
    using SignalInfos = std::vector<Signal>;

protected:
    SignalInfos _signal_infos;
    std::size_t _vi_index{0};

    ValueSignalInfo* _register_state(Signal sig, Signal deriv_sig);

    bool init(Parent&, const LabelSignals&, const LabelSignals&) override;

    template<typename T>
    typename Types<T>::Signal register_signal(const std::string& name);
    ArraySignal register_signal(const std::string& name, std::size_t size);
    BusSignal register_bus(const std::string& name, const BusSpec& spec, LabelSignals::const_iterator begin_, LabelSignals::const_iterator end_);

public:
    Model(std::string given_name="model");
    ~Model();

    Model* model() override {return this;}

    virtual void input_cb(double /*t*/, Values& /*values*/) {}

    const SignalInfos& signals() const {return _signal_infos;}

    void register_state(Signal sig, Signal deriv_sig, double iv)
    {
        pooya_trace("block: " + full_name());
        _register_state(sig, deriv_sig)->_scalar->_iv = iv;
    }

    void register_state(Signal sig, Signal deriv_sig, const Array& iv)
    {
        pooya_trace("block: " + full_name());
        _register_state(sig, deriv_sig)->_array->_iv = iv;
    }

    Signal lookup_signal(const std::string& name, bool exact_match=false) const;
};

template<>
typename Types<double>::Signal Model::register_signal<double>(const std::string& name);

template<>
typename Types<int>::Signal Model::register_signal<int>(const std::string& name);

}

#endif // __POOYA_BLOCK_BASE_HPP__
