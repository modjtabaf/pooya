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

#ifndef __POOYA_SIGNAL_ARRAY_SIGNAL_HPP__
#define __POOYA_SIGNAL_ARRAY_SIGNAL_HPP__

#include "src/helper/trace.hpp"
#include "src/helper/verify.hpp"
#include "array.hpp"
#include "float_signal.hpp"

#define pooya_verify_array_signal(sig) \
    pooya_verify_valid_signal(sig); \
    pooya_verify((sig)->is_array(), (sig)->_full_name + ": array signal expected!");

#define pooya_verify_array_signal_size(sig, size_) \
    pooya_verify_array_signal(sig); \
    pooya_verify((sig)->as_array()->size() == size_, (sig)->_full_name + ": array size mismatch!");

namespace pooya
{

class ArraySignalInfo : public FloatSignalInfo
{
    friend class Model;

protected:
    const std::size_t _size;
    MappedArray _array_value{nullptr, 0};
    MappedArray _deriv_array_value{nullptr, 0};

    static ArraySignalId create_new(const std::string& full_name, std::size_t index, std::size_t size)
    {
#if defined(POOYA_USE_SMART_PTRS)
        return std::make_shared<ArraySignalInfo>(Protected(), full_name, index, size);
#else // defined(POOYA_USE_SMART_PTRS)
        return new ArraySignalInfo(full_name, index, size);
#endif // defined(POOYA_USE_SMART_PTRS)
    } 

public:
#if defined(POOYA_USE_SMART_PTRS)
    ArraySignalInfo(Protected, const std::string& full_name, std::size_t index, std::size_t size)
#else // defined(POOYA_USE_SMART_PTRS)
    ArraySignalInfo(const std::string& full_name, std::size_t index, std::size_t size)
#endif // defined(POOYA_USE_SMART_PTRS)
        : FloatSignalInfo(full_name, index), _size(size)
    {
        _array = true;
    }

    const MappedArray& get() const
    {
        pooya_trace0;
        pooya_verify(_array_value.rows() == int(_size), _full_name + ": attempting to retrieve the value of an uninitialized array signal!");
        pooya_verify(is_assigned(), _full_name + ": attempting to access an unassigned value!");
        return _array_value;
    }

    void set(const Array& value)
    {
        pooya_verify(_array_value.rows() == int(_size), _full_name + ": attempting to assign the value of an uninitialized array signal!");
        pooya_verify(!is_assigned(), _full_name + ": re-assignment is prohibited!");
        pooya_verify(value.rows() == int(_size),
            std::string("size mismatch (id=") + _full_name + ")(" + std::to_string(_size) +
            " vs " + std::to_string(value.rows()) + ")!");
        _array_value = value;
        if (_deriv_array_value.size() == int(_size))
        {
            _deriv_array_value = value;
        }
        _assigned = true;
    }

    std::size_t size() const {return _size;}
};

#if defined(POOYA_USE_SMART_PTRS)

inline ArraySignalId SignalInfo::as_array() {return _array ? std::static_pointer_cast<ArraySignalInfo>(shared_from_this()) : ArraySignalId();}
inline ReadOnlyArraySignalId SignalInfo::as_array() const {return _array ? std::static_pointer_cast<const ArraySignalInfo>(shared_from_this()) : ReadOnlyArraySignalId();}

#else // defined(POOYA_USE_SMART_PTS)

inline ArraySignalId SignalInfo::as_array() {return _array ? static_cast<ArraySignalId>(this) : nullptr;}
inline ReadOnlyArraySignalId SignalInfo::as_array() const {return _array ? static_cast<ReadOnlyArraySignalId>(this) : nullptr;}

#endif // defined(POOYA_USE_SMART_PTS)

}

#endif // __POOYA_SIGNAL_ARRAY_SIGNAL_HPP__
