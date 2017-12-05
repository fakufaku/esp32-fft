/*

  ESP32 FFT
  =========

  This provides a vanilla radix-2 FFT implementation and a test example.

  Author
  ------

  This code was written by [Robin Scheibler](http://www.robinscheibler.org) during rainy days in October 2017.

  License
  -------

  Copyright (c) 2017 Robin Scheibler

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

*/
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "fft.h"

#define TWO_PI 6.28318530

fft_config_t *fft_init(int size, fft_type_t type, fft_direction_t direction, float *input, float *output)
{
  /*
   * Prepare an FFT of correct size and types.
   *
   * If no input or output buffers are provided, they will be allocated.
   */
  int k,m;

  fft_config_t *config = (fft_config_t *)malloc(sizeof(fft_config_t));

  // Check if the size is a power of two
  if ((size & (size-1)) != 0)  // tests if size is a power of two
    return NULL;

  // start configuration
  config->flags = 0;
  config->type = type;
  config->direction = direction;
  config->size = size;

  // Allocate and precompute twiddle factors
  config->twiddle_factors = (float *)malloc(2 * config->size * sizeof(float));

  float two_pi_by_n = TWO_PI / config->size;

  for (k = 0, m = 0 ; k < config->size ; k++, m+=2)
  {
    config->twiddle_factors[m] = cosf(two_pi_by_n * k);    // real
    config->twiddle_factors[m+1] = sinf(two_pi_by_n * k);  // imag
  }

  // Allocate input buffer
  if (input != NULL)
    config->input = input;
  else 
  {
    if (config->type == FFT_REAL)
      config->input = (float *)malloc(config->size * sizeof(float));
    else if (config->type == FFT_COMPLEX)
      config->input = (float *)malloc(2 * config->size * sizeof(float));

    config->flags |= FFT_OWN_INPUT_MEM;
  }

  if (config->input == NULL)
    return NULL;

  // Allocate output buffer
  if (output != NULL)
    config->output = output;
  else
  {
    if (config->type == FFT_REAL)
      config->output = (float *)malloc(config->size * sizeof(float));
    else if (config->type == FFT_COMPLEX)
      config->output = (float *)malloc(2 * config->size * sizeof(float));

    config->flags |= FFT_OWN_OUTPUT_MEM;
  }

  if (config->output == NULL)
    return NULL;

  return config;
}

void fft_destroy(fft_config_t *config)
{
  if (config->flags & FFT_OWN_INPUT_MEM)
    free(config->input);

  if (config->flags & FFT_OWN_OUTPUT_MEM)
    free(config->output);

  free(config->twiddle_factors);
  free(config);
}

void fft_execute(fft_config_t *config)
{
  if (config->type == FFT_REAL && config->direction == FFT_FORWARD)
    rfft(config->input, config->output, config->twiddle_factors, config->size);
  else if (config->type == FFT_REAL && config->direction == FFT_BACKWARD)
    irfft(config->input, config->output, config->twiddle_factors, config->size);
  else if (config->type == FFT_COMPLEX && config->direction == FFT_FORWARD)
    fft(config->input, config->output, config->twiddle_factors, config->size);
  else if (config->type == FFT_COMPLEX && config->direction == FFT_BACKWARD)
    ifft(config->input, config->output, config->twiddle_factors, config->size);
}

void fft(float *input, float *output, float *twiddle_factors, int n)
{
  /*
   * Forward fast Fourier transform
   * DIT, radix-2, out-of-place implementation
   *
   * Parameters
   * ----------
   *  input (float *)
   *    The input array containing the complex samples with
   *    real/imaginary parts interleaved [Re(x0), Im(x0), ..., Re(x_n-1), Im(x_n-1)]
   *  output (float *)
   *    The output array containing the complex samples with
   *    real/imaginary parts interleaved [Re(x0), Im(x0), ..., Re(x_n-1), Im(x_n-1)]
   *  n (int)
   *    The FFT size, should be a power of 2
   */

  fft_primitive(input, output, n, 2, twiddle_factors, 2);
}

void ifft(float *input, float *output, float *twiddle_factors, int n)
{
  /*
   * Inverse fast Fourier transform
   * DIT, radix-2, out-of-place implementation
   *
   * Parameters
   * ----------
   *  input (float *)
   *    The input array containing the complex samples with
   *    real/imaginary parts interleaved [Re(x0), Im(x0), ..., Re(x_n-1), Im(x_n-1)]
   *  output (float *)
   *    The output array containing the complex samples with
   *    real/imaginary parts interleaved [Re(x0), Im(x0), ..., Re(x_n-1), Im(x_n-1)]
   *  n (int)
   *    The FFT size, should be a power of 2
   */
  ifft_primitive(input, output, n, 2, twiddle_factors, 2);
}

void rfft(float *x, float *y, float *twiddle_factors, int n)
{

  // This code uses the two-for-the-price-of-one strategy
  fft_primitive(x, y, n / 2, 2, twiddle_factors, 4);

  // Now apply post processing to recover positive
  // frequencies of the real FFT
  float t = y[0];
  y[0] = t + y[1];  // DC coefficient
  y[1] = t - y[1];  // Center coefficient

  // Apply post processing to quarter element
  // this boils down to taking complex conjugate
  y[n/2+1] = -y[n/2+1];

  // Now process all the other frequencies
  int k;
  for (k = 2 ; k < n / 2 ; k += 2)
  {
    float xer, xei, xor, xoi, c, s, tr, ti;

    c = twiddle_factors[k];
    s = twiddle_factors[k+1];
    
    // even half coefficient
    xer = 0.5 * (y[k] + y[n-k]);
    xei = 0.5 * (y[k+1] - y[n-k+1]);

    // odd half coefficient
    xor = 0.5 * (y[k+1] + y[n-k+1]);
    xoi = - 0.5 * (y[k] - y[n-k]);

    tr =  c * xor + s * xoi;
    ti = -s * xor + c * xoi;

    y[k]   = xer + tr;
    y[k+1] = xei + ti;

    y[n-k]   =   xer - tr;
    y[n-k+1] = -(xei - ti);
  }
}

void irfft(float *x, float *y, float *twiddle_factors, int n)
{
  /*
   * Destroys content of input vector
   */
  int k;

  // Here we need to apply a pre-processing first
  float t = x[0];
  x[0] = 0.5 * (t + x[1]);
  x[1] = 0.5 * (t - x[1]);

  x[n/2+1] = -x[n/2+1];

  for (k = 2 ; k < n / 2 ; k += 2)
  {
    float xer, xei, xor, xoi, c, s, tr, ti;

    c = twiddle_factors[k];
    s = twiddle_factors[k+1];

    xer = 0.5 * (x[k] + x[n-k]);
    tr  = 0.5 * (x[k] - x[n-k]);

    xei = 0.5 * (x[k+1] - x[n-k+1]);
    ti  = 0.5 * (x[k+1] + x[n-k+1]);

    xor = c * tr - s * ti;
    xoi = s * tr + c * ti;

    x[k]   = xer - xoi;
    x[k+1] = xor + xei;

    x[n-k]   = xer + xoi;
    x[n-k+1] = xor - xei;
  }

  ifft_primitive(x, y, n / 2, 2, twiddle_factors, 4);
}

void fft_primitive(float *x, float *y, int n, int stride, float *twiddle_factors, int tw_stride)
{
  /*
   * This code will compute the FFT of the input vector x
   *
   * The input data is assumed to be real/imag interleaved
   *
   * The size n should be a power of two
   *
   * y is an output buffer of size 2n to accomodate for complex numbers
   *
   * Forward fast Fourier transform
   * DIT, radix-2, out-of-place implementation
   *
   * For a complex FFT, call first stage as:
   * fft(x, y, n, 2, 2);
   *
   * Parameters
   * ----------
   *  x (float *)
   *    The input array containing the complex samples with
   *    real/imaginary parts interleaved [Re(x0), Im(x0), ..., Re(x_n-1), Im(x_n-1)]
   *  y (float *)
   *    The output array containing the complex samples with
   *    real/imaginary parts interleaved [Re(x0), Im(x0), ..., Re(x_n-1), Im(x_n-1)]
   *  n (int)
   *    The FFT size, should be a power of 2
   *  stride (int)
   *    The number of elements to skip between two successive samples
   *  tw_stride (int)
   *    The number of elements to skip between two successive twiddle factors
   */
  int k;

  // End condition, stop at n=2 to avoid one trivial recursion
  if (n == 2)
  {
    y[0] = x[0] + x[stride];
    y[1] = x[1] + x[stride + 1];
    y[2] = x[0] - x[stride];
    y[3] = x[1] - x[stride + 1];
    return;
  }

  // Recursion -- Decimation In Time algorithm
  fft_primitive(x, y, n / 2, 2 * stride, twiddle_factors, 2 * tw_stride);             // even half
  fft_primitive(x + stride, y+n, n / 2, 2 * stride, twiddle_factors, 2 * tw_stride);  // odd half

  // Stitch back together
  for (k = 0 ; k < n / 2 ; k++)
  {
    float x1r, x1i, x2r, x2i, c, s;
    c = twiddle_factors[k * tw_stride];
    s = twiddle_factors[k * tw_stride + 1];

    x1r = y[2 * k];
    x1i = y[2 * k + 1];
    x2r =  c * y[n + 2 * k] + s * y[n + 2 * k + 1];
    x2i = -s * y[n + 2 * k] + c * y[n + 2 * k + 1];

    y[2 * k] = x1r + x2r;
    y[2 * k + 1] = x1i + x2i;

    y[n + 2 * k] = x1r - x2r;
    y[n + 2 * k + 1] = x1i - x2i;
  }

}

void ifft_primitive(float *input, float *output, int n, int stride, float *twiddle_factors, int tw_stride)
{

  fft_primitive(input, output, n, stride, twiddle_factors, tw_stride);

  int ks;

  int ns = n * stride;

  // reverse all coefficients from 1 to n / 2 - 1
  for (ks = stride ; ks < ns / 2 ; ks += stride)
  {
    float tr, ti;
    tr = output[ks];
    ti = output[ks+1];

    output[ks] = output[ns-ks];
    output[ks+1] = output[ns-ks+1];

    output[ns-ks] = tr;
    output[ns-ks+1] = ti;
  }

  // Apply normalization
  float norm = 1. / n;
  for (ks = 0 ; ks < ns ; ks += stride)
  {
    output[ks]   *= norm;
    output[ks+1] *= norm;
  }

}

