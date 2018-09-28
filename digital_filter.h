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

#include <stdint.h>

namespace filter {
  // Functions
  void fft(int dir, int m, float *real, float *imag);
  void dft(int dir, int len, float *real, float *imag);
  void fftshift(float *data, int len);
  void hamming(float *data, int len);
  void zeroPad(float *data, float *result, int dataLen, int resultLen);

  void create_fir_lowpass(float *filter_vector, float f_break, int bits, bool use_hamming);
  float _run_fir_iteration(float const* vector, float const* filter, int bits, uint32_t offset);

  void _add_sample(float *buffer, float sample, int bits, uint32_t &offset);

  template<typename T, uint32_t BITS>
  class FIRFilter{
    static constexpr uint32_t FIR_TAPS_BITS = BITS;
    static constexpr uint32_t FIR_LEN = (1 << BITS);

    float m_fir_coeffs[FIR_LEN];
    float m_fir_samples[FIR_LEN];
    uint32_t m_fir_index = 0;

  public:
    FIRFilter(float f_break, bool use_hamming){
      create_fir_lowpass(m_fir_coeffs, f_break, FIR_TAPS_BITS, use_hamming);
    }

    inline void add_sample(T sample) volatile {
      _add_sample((float*)m_fir_samples, static_cast<float>(sample), FIR_TAPS_BITS, (uint32_t&)m_fir_index);
    }

    inline T run_fir_iteration() const volatile {
      return T{_run_fir_iteration((float*)m_fir_samples, (float*)m_fir_coeffs, FIR_TAPS_BITS, m_fir_index)};
    }
  };

}
