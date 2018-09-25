
//#define USE_UNITS

#define UNIT_LIB_DISABLE_IOSTREAM
#define UNIT_LIB_DEFAULT_TYPE float
#include "units.h"

using units::traits::unit_t_traits;

using ampere_t  = float;
using volt_t    = float;
//using celsius_t = float;
using watt_t    = float;
using microsecond_t = float;
using millisecond_t = float;
using second_t  = float;
using minute_t  = float;
using hertz_t   = float;
using radian_t  = float;
using degree_t  = float;


using kv_t        = float;
using rpm_t       = float;
using dutycycle_t = float;
using scalar_t    = float;

//using namespace units::literals;
using units::literals::operator""_degC;

constexpr ampere_t operator"" _A(long double f){
  return static_cast<float>(f);
}

using namespace units::temperature;
/*
using namespace units::time;
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

float map(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//std::enable_if_t<std::is_same<S, typename units::compound_unit<U, units::inverse<T>>>::value, T>
template<typename T, typename U>
U map(T volatile const x, T volatile const in_min, T volatile const in_max, U volatile const out_min, U volatile const out_max){
  //return map(unit_cast<float>(x), unit_cast<float>(in_min), unit_cast<float>(in_max), unit_cast<float>(out_min), unit_cast<float>(out_max));
  return U{map(static_cast<float>(x), static_cast<float>(in_min), static_cast<float>(in_max), static_cast<float>(out_min), static_cast<float>(out_max))};
}

template<typename T>
bool truncate_number_abs(T & number, T max){
  return truncate_number_abs(static_cast<float&>(number), static_cast<float>(max));
}

bool truncate_number(float, float) {
  return false;
}

template<typename T>
bool truncate_number(T volatile const& number, T volatile const& max) {
  return truncate_number(static_cast<float>(number), static_cast<float>(max));
}

bool truncate_number_abs(float &number, float max) {

    if (number > max) {
        number = max;
        return true;
    } else if (number < -max) {
        number = -max;
        return true;
    }
    return false;
}
float f1 = 10.;
float f2 = 20.;
volatile float vf1 = 10.;
volatile float vf2 = 20.;
celsius_t d1 = 10_degC;
celsius_t d2 = 20_degC;
volatile celsius_t vd1 = 10_degC;
volatile celsius_t vd2 = 20_degC;

/*
volatile bool blaf = vf1 < f1;
volatile bool bla = vd1 < d1;
volatile bool bla2 = vd1 > d1;
volatile bool bla3 = vd1 <= d1;
volatile bool bla4 = vd1 >= d1;

volatile bool bla5 = d1 < vd1;
volatile bool bla6 = d1 > vd1;
volatile bool bla7 = d1 <= vd1;
volatile bool bla8 = d1 >= vd1;

volatile auto res1 = vd1 * 2.0;
volatile auto res2 = 2.0 * vd1;
volatile auto res3 = vd1 / 2.0;
volatile auto res4 = vd1 - d1;
volatile auto res5 = d1 - vd1;
volatile auto res6 = vd2 - vd1;
volatile auto res7 = d2  d1;
volatile auto res8 = vd1 + d1;
volatile auto res9 = d1 + vd1;
volatile auto res10 = vd2 + vd1;
volatile auto res11 = d2 + d1;
volatile auto res12 = vd1 += d1;
volatile auto res13 = vd1 -= d1;
volatile auto res14 = vd1 += vd2;
volatile auto res15 = vd1 -= vd2;
namespace {
volatile auto res12 = d1 += d1;
volatile auto res13 = d1 -= d1;
volatile auto res14 = d1 += vd2;
volatile auto res15 = d1 -= vd2;
}
*/

volatile auto did_truncfvfv = truncate_number(vf1, vf2);
volatile auto did_truncd  = truncate_number(d1, d2);
volatile auto did_truncvf  = truncate_number(vf1, f2);
volatile auto did_truncdv = truncate_number(vd1, vd2);
volatile auto did_truncdvn = truncate_number(vd1, d2);

volatile auto did_truncf = truncate_number_abs(f1, f2);
volatile auto did_truncfv = truncate_number_abs(f1, vf2);
//volatile auto did_truncvf = truncate_number_abs(vf1, f2);
volatile auto did_trunc = truncate_number_abs(d1, d2);


volatile auto temp_mapped = map(d2, d1, d2, d1, d2);
static_assert(std::is_same<volatile celsius_t, decltype(temp_mapped)>::value);

volatile auto temp_mapped_mix = map(d2, d1, d2, f1, f2);
volatile auto temp_mapped_mix_vn = map(vd2, d1, d2, f1, 0.0f);

static_assert(std::is_same<volatile float, decltype(temp_mapped_mix)>::value);
