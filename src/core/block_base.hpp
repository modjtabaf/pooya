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
    std::vector<SignalId> _dependencies;
    Parent* _parent{nullptr};
    std::string _given_name;
    std::string _full_name;
    uint16_t _num_iports{NoIOLimit};
    uint16_t _num_oports{NoIOLimit};
    std::size_t _unnamed_signal_counter{0};

    bool _processed{false};
    void _assign_valid_given_name(std::string given_name);
    bool _add_dependecny(SignalId sig);
    bool _remove_dependecny(SignalId sig);

    Block(std::string given_name, uint16_t num_iports=NoIOLimit, uint16_t num_oports=NoIOLimit) :
        _given_name(given_name), _num_iports(num_iports), _num_oports(num_oports) {}

    virtual bool init(Parent& parent, const LabelSignals& iports=LabelSignals(), const LabelSignals& oports=LabelSignals());
    virtual void post_init() {}

public:
    virtual ~Block() = default;

    virtual void pre_step(double /*t*/, Values& /*states*/) {}
    virtual void post_step(double /*t*/, const Values& /*states*/) {}
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
    BusId create_bus(const std::string& given_name, const BusSpec& spec, Iter begin_, Iter end_);

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

    void pre_step(double t, Values& values) override
    {
        pooya_trace("block: " + full_name());
        for (auto* component: _components)
            component->pre_step(t, values);
    }

    void post_step(double t, const Values& values) override
    {
        pooya_trace("block: " + full_name());
        for (auto* component: _components)
            component->post_step(t, values);
    }

    // retrieve an existing signal
    SignalId get_generic_signal(const std::string& given_name);
    template<typename T, typename... Ts>
    typename Types<T>::SignalId get_signal(const std::string& given_name, Ts... args)
    {
        pooya_trace("block: " + full_name() + ", given name: " + given_name);
        SignalId sig = get_generic_signal(given_name);
        if (!sig)
            return nullptr;
        Types<T>::verify_signal_type(sig, args...);
        return Types<T>::as_type(sig);
    }

    ScalarSignalId get_scalar_signal(const std::string& given_name)
    {
        pooya_trace("block: " + full_name() + ", given name: " + given_name);
        return get_signal<double>(given_name);
    }
    IntSignalId get_int_signal(const std::string& given_name)
    {
        pooya_trace("block: " + full_name() + ", given name: " + given_name);
        return get_signal<int>(given_name);
    }
    BoolSignalId get_bool_signal(const std::string& given_name)
    {
        pooya_trace("block: " + full_name() + ", given name: " + given_name);
        return get_signal<bool>(given_name);
    }
    ArraySignalId get_array_signal(const std::string& given_name, std::size_t size)
    {
        pooya_trace("block: " + full_name() + ", given name: " + given_name);
        return get_signal<Array>(given_name, size);
    }
    BusId get_bus(const std::string& given_name, const BusSpec& spec)
    {
        pooya_trace("block: " + full_name() + ", given name: " + given_name);
        return get_signal<BusSpec>(given_name, spec);
    }

    // get if exists, create otherwise
    template<typename T, typename... Ts>
    typename Types<T>::SignalId signal(const std::string& given_name="", Ts... args)
    {
        pooya_trace("block: " + full_name() + ", given name: " + given_name);
        auto sig = get_signal<T, Ts...>(given_name, args...);
        return sig ? sig : create_signal<T, Ts...>(given_name, args...);
    }

    ScalarSignalId scalar_signal(const std::string& given_name="")
    {
        pooya_trace("block: " + full_name() + ", given name: " + given_name);
        return signal<double>(given_name);
    }
    IntSignalId int_signal(const std::string& given_name="")
    {
        pooya_trace("block: " + full_name() + ", given name: " + given_name);
        return signal<int>(given_name);
    }
    BoolSignalId bool_signal(const std::string& given_name="")
    {
        pooya_trace("block: " + full_name() + ", given name: " + given_name);
        return signal<bool>(given_name);
    }
    BusId bus(const std::string& given_name, const BusSpec& spec)
    {
        pooya_trace("block: " + full_name() + ", given name: " + given_name);
        auto sig = get_bus(given_name, spec);
        return sig ? sig : create_bus(given_name, spec);
    }

    // create with a unique name
    template<typename T, typename... Ts>
    typename Types<T>::SignalId create_signal(const std::string& given_name="", Ts... args);
    BusId create_bus(const std::string& given_name, const BusSpec& spec, const std::initializer_list<LabelSignalId>& l);
    BusId create_bus(const std::string& given_name, const BusSpec& spec, const std::initializer_list<SignalId>& l={});

    ScalarSignalId create_scalar_signal(const std::string& given_name="")
    {
        pooya_trace("block: " + full_name() + ", given name: " + given_name);
        return create_signal<double>(given_name);
    }
    IntSignalId create_int_signal(const std::string& given_name="")
    {
        pooya_trace("block: " + full_name() + ", given name: " + given_name);
        return create_signal<int>(given_name);
    }
    BoolSignalId create_bool_signal(const std::string& given_name="")
    {
        pooya_trace("block: " + full_name() + ", given name: " + given_name);
        return create_signal<bool>(given_name);
    }
    ArraySignalId create_array_signal(const std::string& given_name, std::size_t size)
    {
        pooya_trace("block: " + full_name() + ", given name: " + given_name);
        return create_signal<Array, std::size_t>(given_name, size);
    }

    // clone (make a new copy)
    SignalId clone_signal(const std::string& given_name, SignalId sig);
    ScalarSignalId clone_signal(const std::string& given_name, ScalarSignalId sig)
    {
        pooya_trace("block: " + full_name());
        return clone_signal(given_name, SignalId(sig))->as_scalar();
    }
    IntSignalId clone_signal(const std::string& given_name, IntSignalId sig)
    {
        pooya_trace("block: " + full_name());
        return clone_signal(given_name, SignalId(sig))->as_int();
    }
    BoolSignalId clone_signal(const std::string& given_name, BoolSignalId sig)
    {
        pooya_trace("block: " + full_name());
        return clone_signal(given_name, SignalId(sig))->as_bool();
    }
    ArraySignalId clone_signal(const std::string& given_name, ArraySignalId sig)
    {
        pooya_trace("block: " + full_name());
        return clone_signal(given_name, SignalId(sig))->as_array();
    }
    BusId clone_bus(const std::string& given_name, BusId sig)
    {
        pooya_trace("block: " + full_name());
        return clone_signal(given_name, SignalId(sig))->as_bus();
    }

    std::string make_signal_name(const std::string& given_name, bool make_new=false);
    void _mark_unprocessed() override;
    uint _process(double t, Values& values, bool go_deep = true) override;
    bool traverse(TraverseCallback cb, uint32_t level, uint32_t max_level=std::numeric_limits<uint32_t>::max()) override;
}; // class Parent

class Submodel : public Parent
{
public:
    Submodel(std::string given_name, uint16_t num_iports=NoIOLimit, uint16_t num_oports=NoIOLimit) : Parent(given_name, num_iports, num_oports) {}

}; // class Submodel

class Model : public Parent
{
    friend class Parent;

public:
    using SignalInfos = std::vector<SignalId>;

protected:
    SignalInfos _signal_infos;
    std::size_t _vi_index{0};

    bool init(Parent&, const LabelSignals&, const LabelSignals&) override;

    template<typename T, typename... Ts>
    typename Types<T>::SignalId register_signal(const std::string& name, Ts... args)
    {
        pooya_trace("block: " + full_name() + ", given name: " + name);
        if (name.empty()) return nullptr;

        verify(!lookup_signal(name, true), "Re-registering a signal is not allowed!");

        auto index = _signal_infos.size();
        typename Types<T>::SignalId sig;
        if constexpr (std::is_base_of_v<ValueSignalInfo, typename Types<T>::SignalInfo>)
            sig = new typename Types<T>::SignalInfo(name, index, _vi_index++, args...);
        else
            sig = new typename Types<T>::SignalInfo(name, index, args...);
        _signal_infos.push_back(sig);

        return sig;
    }

    ScalarSignalId register_scalar_signal(const std::string& name)
    {
        pooya_trace("block: " + full_name() + ", given name: " + name);
        return register_signal<double>(name);
    }
    IntSignalId register_int_signal(const std::string& name)
    {
        pooya_trace("block: " + full_name() + ", given name: " + name);
        return register_signal<int>(name);
    }
    BoolSignalId register_bool_signal(const std::string& name)
    {
        pooya_trace("block: " + full_name() + ", given name: " + name);
        return register_signal<bool>(name);
    }
    ArraySignalId register_array_signal(const std::string& name, std::size_t size)
    {
        pooya_trace("block: " + full_name() + ", given name: " + name);
        return register_signal<Array, std::size_t>(name, size);
    }
    BusId register_bus(const std::string& name, const BusSpec& spec, LabelSignals::const_iterator begin_, LabelSignals::const_iterator end_)
    {
        pooya_trace("block: " + full_name() + ", given name: " + name);
        return register_signal<BusSpec, const BusSpec&, LabelSignals::const_iterator, LabelSignals::const_iterator>(name, spec, begin_, end_);
    }

public:
    Model(std::string given_name="model");
    ~Model();

    Model* model() override {return this;}

    virtual void input_cb(double /*t*/, Values& /*values*/) {}

    const SignalInfos& signals() const {return _signal_infos;}

    void register_state(SignalId sig, SignalId deriv_sig);
    SignalId lookup_signal(const std::string& name, bool exact_match=false) const;
};

template<typename T, typename... Ts>
typename Types<T>::SignalId Parent::create_signal(const std::string& given_name, Ts... args)
{
    pooya_trace("block: " + full_name() + ", given name: " + given_name);
    return Types<T>::as_type(model_ref().register_signal<T, Ts...>(make_signal_name(given_name, true), args...));
}

}

#endif // __POOYA_BLOCK_BASE_HPP__
