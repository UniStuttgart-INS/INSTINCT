#include "CatchMatchers.hpp"

namespace Catch::Matchers
{

auto WithinAbs(long double target, long double margin) -> WithinAbsMatcher
{
    return WithinAbsMatcher{ static_cast<double>(target), static_cast<double>(margin) };
}

} // namespace Catch::Matchers