#pragma once

//#define USE_UNITS


#ifdef USE_UNITS
#define volatil_

#define UNIT_LIB_DISABLE_IOSTREAM
#define UNIT_LIB_DEFAULT_TYPE float
#include "units.h"

using units::traits::unit_t_traits;

using ampere_t  = float;
using volt_t    = float;
using celsius_t = float;
using watt_t    = float;
using radian_t  = float;
using degree_t  = float;


using kv_t        = float;
using rpm_t       = float;
using dutycycle_t = float;

using namespace units::frequency;
using namespace units::time;
using namespace units::dimensionless;

//using namespace units::literals;
//using units::literals::operator""_degC;

using units::literals::operator""_us;
using units::literals::operator""_ms;
using units::literals::operator""_s;
using units::literals::operator""_Hz;
using units::literals::operator""_MHz;

constexpr celsius_t operator"" _degC(long double f){
  return static_cast<float>(f);
}

constexpr ampere_t operator"" _A(long double f){
  return static_cast<float>(f);
}

constexpr volt_t operator"" _V(long double f){
  return static_cast<float>(f);
}

/*
using namespace units::temperature;
using namespace units::frequency
using namespace units::current;
using namespace units::voltage;
using namespace units::power;
using namespace units::impedance;
using namespace units::capacitance;
using namespace units::inductance;
using namespace units::magnetic_flux;
using namespace units::torque;
using namespace units::angle;
using namespace units::angular_velocity;

using rpm_t     = unit_t<inverse<seconds>>;
using kv_t      = compound_unit<rpm_t, inverse<volts>>;
using dutycycle_t = float;
*/

using units::unit_cast;


//using namespace std::chrono_literals;
//auto u = 2s;
//auto t = 4.0_N;


#else

#define volatil_ volatile

using ampere_t  = float;
using volt_t    = float;
using celsius_t = float;
using watt_t    = float;
using microsecond_t = float;
using millisecond_t = int;
using second_t  = float;
using minute_t  = float;
using hertz_t   = float;
using radian_t  = float;
using degree_t  = float;

constexpr microsecond_t operator"" _us(long double f){
  return static_cast<float>(f);
}

constexpr microsecond_t operator"" _us(unsigned long long f){
  return static_cast<float>(f);
}

constexpr millisecond_t operator"" _ms(unsigned long long f){
  return static_cast<float>(f);
}

constexpr second_t operator"" _s(unsigned long long f){
  return static_cast<float>(f);
}

constexpr second_t operator"" _s(long double f){
  return static_cast<float>(f);
}

constexpr celsius_t operator"" _degC(long double f){
  return static_cast<float>(f);
}

constexpr ampere_t operator"" _A(long double f){
  return static_cast<float>(f);
}

constexpr volt_t operator"" _V(long double f){
  return static_cast<float>(f);
}

constexpr hertz_t operator"" _Hz(long double f){
  return static_cast<float>(f);
}

constexpr hertz_t operator"" _Hz(unsigned long long f){
  return static_cast<float>(f);
}

using kv_t        = float;
using rpm_t       = float;
using dutycycle_t = float;
using scalar_t    = float;

namespace units{
  template<typename T>
  struct inverse;

  template<typename T, typename U>
  struct compound_unit;

  namespace detail{

    template<typename T, typename U, typename ...Args>
    struct compound_impl{};

    template<typename T>
    struct inverse_impl{};
  }
}

template <typename T>
struct unit_t_traits;


template <typename T, typename U>
constexpr T unit_cast(U const& u) {
  return static_cast<T>(u);
}

#endif

namespace units{
/*
  template<>
  struct inverse<float>{
    using type = scalar_t;
  };

  template<>
  struct compound_unit<float, inverse<float>>{
    using type = scalar_t;
  };
*/

  template <typename U>
  constexpr float& unit_cast(U& u) noexcept {
    return static_cast<float&>(u);
  }

  template <typename U>
  constexpr float unit_cast(U const& u) noexcept{
    return static_cast<float>(u);
  }

  namespace detail{

    template<typename T>
    struct compound_impl<T, float>{
      using type = scalar_t;
    };

    template<typename T>
    struct compound_impl<float, T>{
      using type = scalar_t;
    };

    template<>
    struct inverse_impl<float>{
      using type = float;
    };

    template<>
    struct compound_impl<float, float>{
      using type = float;
    };

    template<>
    struct compound_impl<float, float, float>{
      using type = float;
    };

    template<>
    struct compound_impl<float, float, float, float>{
      using type = float;
    };

    template<>
    struct compound_impl<float, float, float, float, float>{
      using type = float;
    };
  }
}

