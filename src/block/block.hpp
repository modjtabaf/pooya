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

#include "src/helper/defs.hpp"
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

    enum SignalLinkType : uint32_t
    {
        Input    = 1U << 0,
        Output   = 1U << 1,
        Internal = 1U << 2,
        Required = 1U << 3,
        Unknown  = 1U << (8 * sizeof(SignalLinkType) - 1),
    };

    using SignalLinkPair = std::pair<std::shared_ptr<ValueSignalImpl>, uint32_t>;

    virtual ~Block() = default;

    virtual bool set_parent(Submodel& parent);
    virtual bool connect(const Bus& ibus = Bus(), const Bus& obus = Bus());
    virtual void input_cb(double /*t*/) {}
    virtual void pre_step(double /*t*/) {}
    virtual void post_step(double /*t*/) {}

    auto parent() -> auto { return _parent; }
    bool processed() const { return _processed; }
    bool is_connected() const { return _connected; }
    const Bus& ibus() const { return _ibus; }
    const Bus& obus() const { return _obus; }
    auto linked_signals() const -> const auto& { return _linked_signals; }

    template<typename Key>
    Signal io_at(const Bus& bus, Key key, std::optional<uint32_t> types)
    {
        Signal sig(bus.at(key));
        if (types.has_value())
        {
            link_signal(sig, *types);
        }
        return sig;
    }

    template<typename Key = std::size_t>
    Signal input(Key key)
    {
        return io_at<Key>(_ibus, key, SignalLinkType::Input | SignalLinkType::Required);
    }

    template<typename Key = std::size_t>
    Signal output(Key key)
    {
        return io_at<Key>(_obus, key, SignalLinkType::Output);
    }

    virtual void _mark_unprocessed();
    virtual uint process(double t, bool go_deep = true) = 0;

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

protected:
    bool _connected{false};
    Bus _ibus;
    Bus _obus;
    std::vector<SignalLinkPair> _linked_signals;
    Submodel* _parent{nullptr};
    uint16_t _num_iports{NoIOLimit};
    uint16_t _num_oports{NoIOLimit};

    bool _processed{false};
    void link_signal(const Signal& sig, uint32_t types);
    SignalLinkPair* find_linked_signal(SignalImpl& impl);

    explicit Block(Submodel* parent = nullptr, std::string_view name = "", uint16_t num_iports = NoIOLimit,
                   uint16_t num_oports = NoIOLimit);
}; // class Block

} // namespace pooya

#endif // __POOYA_BLOCK_BLOCK_HPP__
