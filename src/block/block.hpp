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

#ifndef __POOYA_BLOCK_BLOCK_HPP__
#define __POOYA_BLOCK_BLOCK_HPP__

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>

#include "src/shared/named_object.hpp"
#include "src/signal/array_signal.hpp"
#include "src/signal/bool_signal.hpp"
#include "src/signal/bus.hpp"
#include "src/signal/int_signal.hpp"
#include "src/signal/scalar_signal.hpp"
#include "src/signal/trait.hpp"

namespace pooya
{

class Block;
class Submodel;

using VisitorCallback      = std::function<bool(Block&, uint32_t level)>;
using ConstVisitorCallback = std::function<bool(const Block&, uint32_t level)>;

class Block : public NamedObject
{
public:
    static constexpr uint16_t NoIOLimit = uint16_t(-1);

    enum class SignalLinkType
    {
        Input,
        Output,
        Internal,
        Unknown,
    };

    using SignalLinkPair = std::pair<ValueSignalImplPtr, SignalLinkType>;

protected:
    bool _initialized{false};
    Bus _ibus;
    Bus _obus;
    std::vector<SignalLinkPair> _linked_signals;
    Submodel* _parent{nullptr};
    uint16_t _num_iports{NoIOLimit};
    uint16_t _num_oports{NoIOLimit};

    bool _processed{false};
    bool link_signal(const ValueSignalImplPtr& sig, SignalLinkType type);

    Block(uint16_t num_iports = NoIOLimit, uint16_t num_oports = NoIOLimit)
        : _num_iports(num_iports), _num_oports(num_oports)
    {
    }
    Block(const ValidName& name, uint16_t num_iports = NoIOLimit, uint16_t num_oports = NoIOLimit)
        : NamedObject(name), _num_iports(num_iports), _num_oports(num_oports)
    {
    }

public:
    virtual ~Block() = default;

    virtual bool init(Submodel* parent = nullptr, const Bus& ibus = Bus(), const Bus& obus = Bus());
    virtual void post_init() {}
    virtual void input_cb(double /*t*/) {}
    virtual void pre_step(double /*t*/) {}
    virtual void post_step(double /*t*/) {}

    auto parent() -> auto { return _parent; }
    bool processed() const { return _processed; }
    bool is_initialized() const { return _initialized; }
    const Bus& ibus() const { return _ibus; }
    const Bus& obus() const { return _obus; }
    auto linked_signals() const -> const auto& { return _linked_signals; }

    template<typename T, typename Key>
    typename Types<T>::Signal io_at(const Bus& bus, Key key, std::optional<SignalLinkType> type)
    {
        SignalImplPtr ptr;
        if constexpr (std::is_same_v<Key, std::size_t>)
        {
            ptr = bus->at(key).second;
        }
        else
        {
            ptr = bus->at(key);
        }
        typename Types<T>::Signal sig(ptr);
        if (type.has_value())
        {
            pooya_verify_value_signal(ptr);
            link_signal(std::static_pointer_cast<ValueSignalImpl>(sig->shared_from_this()), *type);
        }
        return Types<T>::as_signal_id(sig);
    }

    ScalarSignal scalar_input_at(const std::string& label)
    {
        return io_at<double, const std::string&>(_ibus, label, SignalLinkType::Input);
    }
    IntSignal int_input_at(const std::string& label)
    {
        return io_at<int, const std::string&>(_ibus, label, SignalLinkType::Input);
    }
    BoolSignal bool_input_at(const std::string& label)
    {
        return io_at<bool, const std::string&>(_ibus, label, SignalLinkType::Input);
    }
    ArraySignal array_input_at(const std::string& label)
    {
        return io_at<Array, const std::string&>(_ibus, label, SignalLinkType::Input);
    }
    ScalarSignal scalar_input_at(std::size_t index)
    {
        return io_at<double, std::size_t>(_ibus, index, SignalLinkType::Input);
    }
    IntSignal int_input_at(std::size_t index) { return io_at<int, std::size_t>(_ibus, index, SignalLinkType::Input); }
    BoolSignal bool_input_at(std::size_t index)
    {
        return io_at<bool, std::size_t>(_ibus, index, SignalLinkType::Input);
    }
    ArraySignal array_input_at(std::size_t index)
    {
        return io_at<Array, std::size_t>(_ibus, index, SignalLinkType::Input);
    }
    ScalarSignal scalar_output_at(const std::string& label)
    {
        return io_at<double, const std::string&>(_obus, label, SignalLinkType::Output);
    }
    IntSignal int_output_at(const std::string& label)
    {
        return io_at<int, const std::string&>(_obus, label, SignalLinkType::Output);
    }
    BoolSignal bool_output_at(const std::string& label)
    {
        return io_at<bool, const std::string&>(_obus, label, SignalLinkType::Output);
    }
    ArraySignal array_output_at(const std::string& label)
    {
        return io_at<Array, const std::string&>(_obus, label, SignalLinkType::Output);
    }
    ScalarSignal scalar_output_at(std::size_t index)
    {
        return io_at<double, std::size_t>(_obus, index, SignalLinkType::Output);
    }
    IntSignal int_output_at(std::size_t index) { return io_at<int, std::size_t>(_obus, index, SignalLinkType::Output); }
    BoolSignal bool_output_at(std::size_t index)
    {
        return io_at<bool, std::size_t>(_obus, index, SignalLinkType::Output);
    }
    ArraySignal array_output_at(std::size_t index)
    {
        return io_at<Array, std::size_t>(_obus, index, SignalLinkType::Output);
    }

    virtual void _mark_unprocessed();
    virtual uint _process(double t, bool go_deep = true) = 0;

    virtual bool visit(VisitorCallback cb, uint32_t level, uint32_t max_level = std::numeric_limits<uint32_t>::max())
    {
        pooya_trace("block: " + full_name().str());
        return (level > max_level) || cb(*this, level);
    }
    virtual bool const_visit(ConstVisitorCallback cb, uint32_t level,
                             uint32_t max_level = std::numeric_limits<uint32_t>::max()) const
    {
        pooya_trace("block: " + full_name().str());
        return (level > max_level) || cb(*this, level);
    }

    ValidName full_name() const;
}; // class Block

} // namespace pooya

#endif // __POOYA_BLOCK_BLOCK_HPP__
