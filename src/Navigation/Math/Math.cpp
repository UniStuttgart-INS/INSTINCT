#include "Math.hpp"

namespace NAV
{

uint64_t factorial(uint64_t n)
{
    // uint64_t is required to calculate factorials of n > 12 (Limit of uint32_t). The limit of uint64_t is at n = 20
    constexpr std::array factorials = {
        uint64_t(1),                   // 0
        uint64_t(1),                   // 1
        uint64_t(2),                   // 2
        uint64_t(6),                   // 3
        uint64_t(24),                  // 4
        uint64_t(120),                 // 5
        uint64_t(720),                 // 6
        uint64_t(5040),                // 7
        uint64_t(40320),               // 8
        uint64_t(362880),              // 9
        uint64_t(3628800),             // 10
        uint64_t(39916800),            // 11
        uint64_t(479001600),           // 12
        uint64_t(6227020800),          // 13
        uint64_t(87178291200),         // 14
        uint64_t(1307674368000),       // 15
        uint64_t(20922789888000),      // 16
        uint64_t(355687428096000),     // 17
        uint64_t(6402373705728000),    // 18
        uint64_t(121645100408832000),  // 19
        uint64_t(2432902008176640000), // 20
    };

    if (n < factorials.size())
    {
        return factorials.at(n);
    }
    return n * factorial(n - 1);
}

} // namespace NAV