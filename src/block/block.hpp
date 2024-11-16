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

#ifndef __POOYA_BLOCK_BLOCK_HPP__
#define __POOYA_BLOCK_BLOCK_HPP__

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <unordered_set>

#include "src/signal/trait.hpp"
#include "src/signal/scalar_signal.hpp"
#include "src/signal/int_signal.hpp"
#include "src/signal/bool_signal.hpp"
#include "src/signal/array_signal.hpp"
#include "src/signal/bus.hpp"

namespace pooya
{

class Block;
class Submodel;

using TraverseCallback = std::function<bool(Block&, uint32_t level)>;
using ConstTraverseCallback = std::function<bool(const Block&, uint32_t level)>;

class Block
{
    friend class Submodel;
    friend class Simulator; // a dirty workaround so Simulator::init can call Block::init. Should be resolved!

public:
    static constexpr uint16_t NoIOLimit = uint16_t(-1);

    enum class SignalAssociationType
    {
        Input,
        Output,
        Internal,
        Unknown,
    };

    using SignalAssociationPair = std::pair<ValueSignalId, SignalAssociationType>;

protected:
    bool _initialized{false};
    BusId _ibus{nullptr};
    BusId _obus{nullptr};
    std::vector<SignalAssociationPair> _associated_signals;
    Submodel* _parent{nullptr};
    std::string _given_name;
    std::string _full_name;
    uint16_t _num_iports{NoIOLimit};
    uint16_t _num_oports{NoIOLimit};
    std::size_t _unnamed_signal_counter{0};

    bool _processed{false};
    bool register_associated_signal(ValueSignalId sig, SignalAssociationType type);

    Block(const std::string& given_name, uint16_t num_iports=NoIOLimit, uint16_t num_oports=NoIOLimit) :
        _given_name(given_name), _num_iports(num_iports), _num_oports(num_oports) {}

    virtual bool init(Submodel* parent=nullptr, BusId ibus=BusId(), BusId obus=BusId());
    virtual void post_init() {}

public:
    virtual ~Block() = default;

    virtual void input_cb(double /*t*/) {}
    virtual void pre_step(double /*t*/) {}
    virtual void post_step(double /*t*/) {}
    virtual void activation_function(double /*t*/) {}

    auto parent() -> auto {return _parent;}
    bool processed() const {return _processed;}
    bool is_initialized() const {return _initialized;}
    const std::string& given_name() const {return _given_name;}
    const std::string& full_name() const {return _full_name;}
    BusId ibus() const {return _ibus;}
    BusId obus() const {return _obus;}
    auto associated_signals() const -> const auto& {return _associated_signals;}

    template<typename T, typename Key>
    typename Types<T>::SignalId io_at(BusId bus, Key key, std::optional<SignalAssociationType> type)
    {
        pooya_verify_bus(bus);
        SignalId sig;
        if constexpr (std::is_same_v<Key, std::size_t>)
            {sig = bus->at(key).second;}
        else
            {sig = bus->at(key);}
#if defined (POOYA_DEBUG)
        Types<T>::verify_signal_type(sig);
#endif // defined (POOYA_DEBUG)
        if (type.has_value())
        {
            pooya_verify_value_signal(sig);
            register_associated_signal(std::static_pointer_cast<ValueSignalInfo>(sig->shared_from_this()), *type);
        }
        return Types<T>::as_signal_id(sig);
    }

    ScalarSignalId scalar_input_at(const std::string& label)
    {
        return io_at<double, const std::string&>(_ibus, label, SignalAssociationType::Input);
    }
    IntSignalId int_input_at(const std::string& label)
    {
        return io_at<int, const std::string&>(_ibus, label, SignalAssociationType::Input);
    }
    BoolSignalId bool_input_at(const std::string& label)
    {
        return io_at<bool, const std::string&>(_ibus, label, SignalAssociationType::Input);
    }
    ArraySignalId array_input_at(const std::string& label)
    {
        return io_at<Array, const std::string&>(_ibus, label, SignalAssociationType::Input);
    }
    ScalarSignalId scalar_input_at(std::size_t index)
    {
        return io_at<double, std::size_t>(_ibus, index, SignalAssociationType::Input);
    }
    IntSignalId int_input_at(std::size_t index)
    {
        return io_at<int, std::size_t>(_ibus, index, SignalAssociationType::Input);
    }
    BoolSignalId bool_input_at(std::size_t index)
    {
        return io_at<bool, std::size_t>(_ibus, index, SignalAssociationType::Input);
    }
    ArraySignalId array_input_at(std::size_t index)
    {
        return io_at<Array, std::size_t>(_ibus, index, SignalAssociationType::Input);
    }
    ScalarSignalId scalar_output_at(const std::string& label)
    {
        return io_at<double, const std::string&>(_obus, label, SignalAssociationType::Output);
    }
    IntSignalId int_output_at(const std::string& label)
    {
        return io_at<int, const std::string&>(_obus, label, SignalAssociationType::Output);
    }
    BoolSignalId bool_output_at(const std::string& label)
    {
        return io_at<bool, const std::string&>(_obus, label, SignalAssociationType::Output);
    }
    ArraySignalId array_output_at(const std::string& label)
    {
        return io_at<Array, const std::string&>(_obus, label, SignalAssociationType::Output);
    }
    ScalarSignalId scalar_output_at(std::size_t index)
    {
        return io_at<double, std::size_t>(_obus, index, SignalAssociationType::Output);
    }
    IntSignalId int_output_at(std::size_t index)
    {
        return io_at<int, std::size_t>(_obus, index, SignalAssociationType::Output);
    }
    BoolSignalId bool_output_at(std::size_t index)
    {
        return io_at<bool, std::size_t>(_obus, index, SignalAssociationType::Output);
    }
    ArraySignalId array_output_at(std::size_t index)
    {
        return io_at<Array, std::size_t>(_obus, index, SignalAssociationType::Output);
    }

    virtual void _mark_unprocessed();
    virtual uint _process(double t, bool go_deep = true);

    virtual bool traverse(TraverseCallback cb, uint32_t level, uint32_t max_level=std::numeric_limits<uint32_t>::max())
    {
        pooya_trace("block: " + full_name());
        return (level > max_level) || cb(*this, level);
    }
    virtual bool const_traverse(ConstTraverseCallback cb, uint32_t level, uint32_t max_level=std::numeric_limits<uint32_t>::max()) const
    {
        pooya_trace("block: " + full_name());
        return (level > max_level) || cb(*this, level);
    }

    std::string make_valid_given_name(const std::string& given_name);
}; // class Block

}

#endif // __POOYA_BLOCK_BLOCK_HPP__
