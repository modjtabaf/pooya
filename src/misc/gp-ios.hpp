
#ifndef __GP_IOS_HPP__
#define __GP_IOS_HPP__

#include "src/core/pooya.hpp"

#include "3rdparty/eigen/Eigen/Core"
#include "3rdparty/misc/gnuplot-iostream.h"

using namespace pooya;
using namespace Eigen;

namespace gnuplotio
{

template<>
// class ArrayTraits<Value>
class ArrayTraits<ArrayXd>
{
public:
    static constexpr int depth = 1;

	// typedef IteratorRange<typename Value::const_iterator, typename Value::value_type> range_type;
	typedef IteratorRange<typename ArrayXd::const_iterator, typename ArrayXd::value_type> range_type;

	// static range_type get_range(const Value& arg)
	static range_type get_range(const ArrayXd& arg)
    {
		return range_type(arg.begin(), arg.end());
	}
};

template<> std::string Gnuplot::file1d(const MatrixXd &arg, const std::string &filename)
{
    // return file1d(Value{arg.reshaped()}, filename);
    return file1d(ArrayXd{arg.reshaped()}, filename);
}

}

#endif // __GP_IOS_HPP__
