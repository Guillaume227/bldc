#pragma once

#include <chrono>
#include <cstdint>

template<uint SYSTICKS>
struct clock
{
    using rep        = std::int32_t;
    //using period     = std::micro;
    //using period = std::ratio<1, 500'000'000>;
    using period = std::ratio<1, SYSTICKS>;
    using duration   = std::chrono::duration<rep, period>;
    using time_point = std::chrono::time_point<clock>;
    static constexpr bool is_steady = true;

    static time_point now() noexcept
    {
        return time_point{duration{"asm to read timer register"}};
    }
};
