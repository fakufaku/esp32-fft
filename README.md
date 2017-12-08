ESP32 FFT
=========

This provides a vanilla radix-2 FFT out-of-place implementation and a test example.

Author
------

This code was written by [Robin Scheibler](http://www.robinscheibler.org) during rainy days in October 2017.

Example
-------

    #include "fft.h"

    ...

    // Create the FFT config structure
    fft_config_t *real_fft_plan = fft_init(NFFT, FFT_REAL, FFT_FORWARD, NULL, NULL);

    // Fill array with some data
    for (k = 0 ; k < fft_analysis->size ; k++)
      real_fft_plan->input[k] = (float)k;

    // Execute transformation
    fft_execute(real_fft_plan);

    // Now do something with the output
    printf("DC component : %f\n", real_fft_plan->output[0]);  // DC is at [0]
    for (k = 1 ; k < real_fft_plan->size / 2 ; k++)
      printf("%d-th freq : %f+j%f\n", k, real_fft_plan->output[2*k], real_fft_plan->output[2*k+1]);
    printf("Middle component : %f\n", real_fft_plan->output[1]);  // N/2 is real and stored at [1]

    // Don't forget to clean up at the end to free all the memory that was allocated
    fft_destroy(real_fft_plan)


Documentation
-------------

1. Create the FFT configuration by running 
  `fft_init`.
        
        fft_config_t *fft_init(int size, fft_type_t type, fft_direction_t direction, float *input, float *output)

        Parameters
        ----------
        size : int
            The FFT size (should be a power of two), if not, returns NULL.
        type : fft_type_t
            The type of FFT, FFT_REAL or FFT_COMPLEX
        direction : fft_direction_t
            The direction, FFT_FORWARD or FFT_BACKWARD (inverse transformation)
        input : float *
            A pointer to a buffer of the correct size. If NULL, a buffer is allocated dynamically
        output : float *
            A pointer to a buffer of the correct size. If NULL, a buffer is allocated dynamically.

        Returns
        -------
        A pointer to an `fft_config_t` structure that holds pointers to the buffers and
        all the necessary configuration options.

2. Fill data in the `input` buffer

3. Call `fft_execute` to run the FFT

4. Use the transformed data located in the `output` buffer

5. Possibly free up memory by calling `fft_destroy` on the configuration structure

### Note about Inverse Real FFT

When doing an inverse real FFT, the data in the input buffer is destroyed.

### Buffer sizes and data organization

* For `FFT_REAL` FFTs (forward as well as backward) of size `NFFT`, the buffer is of size `NFFT`.
  Then, the input data is organized as

        Input  : [ x[0], x[1], x[2], ..., x[NFFT-1] ]
        Output : [ X[0], X[NFFT/2], Re(X[1]), Im(X[1]), ..., Re(X[NFFT/2-1]), Im(X[NFFT/2-1]) ]

* For `FFT_COMPLEX` of size `NFFT`, the buffer is of size `2 * NFFT` as both real and imaginary parts should be saved.

        Input  : [ Re(x[0]), Im(x[0]), ..., Re(x[NFFT-1]), Im(x[NFFT-1]) ]
        Output : [ Re(X[0]), Im(X[0]), ..., Re(X[NFFT-1]), Im(X[NFFT-1]) ]

License
-------

This software is released under the MIT license.

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
