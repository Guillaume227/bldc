/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#pragma once
#include "units_def.h"
//#include <type_traits>

namespace utils{

  void step_towards(float& value, float const goal, float const step);

  template<typename T>
  void step_towards(T& value, T const goal, T const step) {
    step_towards(static_cast<float&>(value), static_cast<float>(goal), static_cast<float>(step));
  }

  float calc_ratio(float low, float high, float val);
  void norm_angle(degree_t &angle);
  void norm_angle_rad(radian_t &angle);

  bool truncate_number(float &number, float min, float max);

  template<typename T>
  inline bool truncate_number(T& number, T const& va, T const& vb){
    return truncate_number(static_cast<float&>(number),
                           static_cast<float>(va),
                           static_cast<float>(vb));
  }

  bool truncate_number(int &number, int min, int max);
  bool truncate_number_abs(float& number, float max);
  template<typename T>
  inline bool truncate_number_abs(T& number, T max){
    return truncate_number_abs(static_cast<float&>(number), static_cast<float>(max));
  }

  float map(float x, float in_min, float in_max, float out_min, float out_max);

  template<typename T, typename U>
  inline U map(T const& x, T const& in_min, T const& in_max, U const& out_min, U const& out_max){
    return U{map(static_cast<float>(x), static_cast<float>(in_min), static_cast<float>(in_max), static_cast<float>(out_min), static_cast<float>(out_max))};
  }

  int map_int(int x, int in_min, int in_max, int out_min, int out_max);
  void deadband(float &value, float tres, float max);
  degree_t angle_difference(degree_t angle1, degree_t angle2);
  radian_t angle_difference_rad(radian_t angle1, radian_t angle2);
  radian_t avg_angles_rad_fast(radian_t const*angles, float const*weights, int angles_num);
  float middle_of_3(float a, float b, float c);
  int middle_of_3_int(int a, int b, int c);
  float fast_inv_sqrt(float x);
  radian_t fast_atan2(float y, float x);
  template<typename T>
  radian_t fast_atan2(T y, T x){
    return fast_atan2(static_cast<float>(y), static_cast<float>(x));
  }
  bool saturate_vector_2d(float &x, float &y, float max);

  template<typename T>
  inline bool saturate_vector_2d(T &x, T &y, T max){
    return saturate_vector_2d(static_cast<float&>(x), static_cast<float&>(y), static_cast<float>(max));
  }

  void fast_sincos(radian_t angle, float *sin, float *cos);
  void fast_sincos_better(radian_t angle, float *sin, float *cos);

  float min_abs(float va, float vb);
  template<typename T>
  inline T min_abs(T va, T vb){
    return T{min_abs(static_cast<float>(va), static_cast<float>(vb))};
  }

  float max_abs(float va, float vb);
  template<typename T>
  inline T max_abs(T va, T vb){
    return T{max_abs(static_cast<float>(va), static_cast<float>(vb))};
  }

  void byte_to_binary(int x, char *b);
  float throttle_curve(float val, float curve_acc, float curve_brake, int mode);

  /**
   * A system locking function with a counter. For every lock, a corresponding unlock must
   * exist to unlock the system. That means, if lock is called five times, unlock has to
   * be called five times as well.
   * Note that chSysLock and chSysLockFromIsr are the same for this port.
   */
  struct sys_lock_scope_t{
    sys_lock_scope_t();
    ~sys_lock_scope_t();
  };

  // Return the sign of the argument. -1 if negative, 1 if zero or positive.
  template<typename T>
  inline constexpr int SIGN(T x){ return x < T{0} ? -1 : 1; }

  // Squared
  template<typename T>
  inline constexpr auto SQ(T x){ return x * x; }
}

// Return the age of a timestamp in seconds
#define UTILS_AGE_S(x)		((float)chVTTimeElapsedSinceX(x) / (float)CH_CFG_ST_FREQUENCY)

// nan and infinity check for floats
#define UTILS_IS_INF(x)		((x) == (1.0 / 0.0) || (x) == (-1.0 / 0.0))
#define UTILS_IS_NAN(x)		((x) != (x))

inline void UTILS_NAN_ZERO(float& x){
  if(UTILS_IS_NAN(x))
    x = 0.0;
}

template<typename T>
inline void UTILS_NAN_ZERO(T& x){
  UTILS_NAN_ZERO(static_cast<float&>(x));
}

template<typename T>
inline void UTILS_NAN_ZERO(T volatile& x){
  UTILS_NAN_ZERO(static_cast<float&>(const_cast<T&>(x)));
}

/**
 * A simple low pass filter.
 *
 * @param value
 * The filtered value.
 *
 * @param sample
 * Next sample.
 *
 * @param filter_constant
 * Filter constant. Range 0.0 to 1.0, where 1.0 gives the unfiltered value.
 */
#define UTILS_LP_FAST(value, sample, filter_constant)	(value -= (filter_constant) * (value - (sample)))

// Some constants
#define ONE_BY_SQRT3			(0.57735026919)
#define TWO_BY_SQRT3			(2.0f * 0.57735026919)
#define SQRT3_BY_2				(0.86602540378)
