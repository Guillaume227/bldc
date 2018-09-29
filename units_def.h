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
using ohm_t     = float;
using microhenry_t = float;

using celsius_t = float;
using watt_t    = float;
using dutycycle_t = float;

using namespace units::frequency;
using namespace units::time;
using namespace units::dimensionless;
using namespace units::angle;
using namespace units::angular_velocity;
using rpm_t = revolutions_per_minute_t;
/*
namespace angular_velocity{
  UNIT_ADD(torque, electrical_rotations_per_minute, electrical_rotations_per_minute, erpm, units<<((2.0 * M_PI) / 60.0)> revolutions_per_minute>);
}

using erpm_t = electrical_rotations_per_minute;
*/
using erpm_t = rpm_t;//electrical_rotations_per_minute;

namespace units{
  UNIT_ADD(torque, kv, kv, kv, compound_unit<angular_velocity::revolutions_per_minute, inverse<voltage::volts>>);
  UNIT_ADD(angular_velocity, inv_sec_2, inv_sec_2, inv_s2, inverse<squared<seconds>>);
}

//using units::torque::kv_t;
using kv_t = rpm_t;

using units::angular_velocity::inv_sec_2_t;

//using namespace units::literals;
//using units::literals::operator""_degC;


using units::literals::operator""_us;
using units::literals::operator""_ms;
using units::literals::operator""_s;
using units::literals::operator""_Hz;
using units::literals::operator""_MHz;
using units::literals::operator""_rad;
using units::literals::operator""_deg;
using units::literals::operator""_rpm;
using units::literals::operator""_deg_per_s;
using units::literals::operator""_rad_per_s;
using units::literals::operator""_kv;
using units::literals::operator""_inv_s2;

constexpr celsius_t operator"" _degC(unsigned long long f){
  return static_cast<float>(f);
}

constexpr celsius_t operator"" _degC(long double f){
  return static_cast<float>(f);
}

constexpr ampere_t operator"" _A(long double f){
  return static_cast<float>(f);
}

constexpr volt_t operator"" _V(unsigned long long f){
  return static_cast<float>(f);
}

constexpr volt_t operator"" _V(long double f){
  return static_cast<float>(f);
}

constexpr watt_t operator"" _W(unsigned long long f){
  return static_cast<float>(f);
}

using weber_t = decltype(1_V * 1_s);

constexpr weber_t operator"" _Wb(unsigned long long f){
  return weber_t{static_cast<float>(f)};
}

using bemf_coupling_t = decltype(1_Wb * 1_rpm / 1_V); //units::compound_unit<voltage::volt, angle::radian>;

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

using dutycycle_t = float;
*/

using units::unit_cast;


//using namespace std::chrono_literals;
//auto u = 2s;
//auto t = 4.0_N;

constexpr radian_t PI_rad {units::constants::detail::PI_VAL};

#else
#include <math.h> // for M_PI
#include <type_traits>

#define volatil_ volatile

using ampere_t  = float;
using volt_t    = float;
using ohm_t     = float;
using microhenry_t = float;

using celsius_t = float;
using watt_t    = float;
using microsecond_t = float;
using millisecond_t = int;
using second_t  = float;
using minute_t  = float;
using hertz_t   = float;
using radian_t  = float;
using degree_t  = float;
using radians_per_second_t = float;
using degrees_per_second_t = float;
using revolutions_per_minute_t = float;
using inv_sec_2_t = float;

using weber_t = float;
using bemf_coupling_t = float;


constexpr revolutions_per_minute_t operator"" _rpm(unsigned long long f){
  return static_cast<float>(f);
}

constexpr radian_t operator"" _rad(long double f){
  return static_cast<float>(f);
}

constexpr degree_t operator"" _deg(unsigned long long f){
  return static_cast<float>(f);
}

constexpr radians_per_second_t operator"" _rad_per_s(long double f){
  return static_cast<float>(f);
}

constexpr degrees_per_second_t operator"" _deg_per_s(unsigned long long f){
  return static_cast<float>(f);
}

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

constexpr inv_sec_2_t operator"" _inv_s2(unsigned long long f){
  return static_cast<float>(f);
}

constexpr second_t operator"" _s(long double f){
  return static_cast<float>(f);
}

constexpr celsius_t operator"" _degC(long double f){
  return static_cast<float>(f);
}

constexpr celsius_t operator"" _degC(unsigned long long f){
  return static_cast<float>(f);
}

constexpr ampere_t operator"" _A(long double f){
  return static_cast<float>(f);
}

constexpr volt_t operator"" _V(long double f){
  return static_cast<float>(f);
}

constexpr volt_t operator"" _V(unsigned long long f){
  return static_cast<float>(f);
}

constexpr watt_t operator"" _W(unsigned long long f){
  return static_cast<float>(f);
}

constexpr weber_t operator"" _Wb(unsigned long long f){
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

constexpr radian_t PI_rad {M_PI};

#endif


template<typename T, typename = std::enable_if_t<!std::is_integral<T>::value>>
T fabsf(T t){
  return T{fabsf(static_cast<float>(t))};
}
template<typename T>
T fmodf(T t, float f){
  return T{fmodf(static_cast<float>(t), f)};
}

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

