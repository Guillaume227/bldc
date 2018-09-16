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
  void create_fir_lowpass(float *filter_vector, float f_break, int bits, int use_hamming);
  float run_fir_iteration(float *vector, float *filter, int bits, uint32_t offset);
  void add_sample(float *buffer, float sample, int bits, uint32_t &offset);
}
