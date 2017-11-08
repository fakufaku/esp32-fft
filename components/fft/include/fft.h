#ifndef __FFT_H__
#define __FFT_H__

typedef enum
{
  FFT_REAL,
  FFT_COMPLEX
} fft_type_t;

typedef enum
{
  FFT_FORWARD,
  FFT_BACKWARD
} fft_direction_t;

#define FFT_OWN_INPUT_MEM 1
#define FFT_OWN_OUTPUT_MEM 2

typedef struct
{
  int size;  // FFT size
  float *input;  // pointer to input buffer
  float *output; // pointer to output buffer
  float *twiddle_factors;  // pointer to buffer holding twiddle factors
  fft_type_t type;   // real or complex
  fft_direction_t direction; // forward or backward
  unsigned int flags; // FFT flags
} fft_config_t;

fft_config_t *fft_init(int size, fft_type_t type, fft_direction_t direction, float *input, float *output);
void fft_destroy(fft_config_t *config);
void fft_execute(fft_config_t *config);
void fft(float *input, float *output, float *twiddle_factors, int n);
void ifft(float *input, float *output, float *twiddle_factors, int n);
void rfft(float *x, float *y, float *twiddle_factors, int n);
void irfft(float *x, float *y, float *twiddle_factors, int n);
void fft_primitive(float *x, float *y, int n, int stride, float *twiddle_factors, int tw_stride);
void ifft_primitive(float *input, float *output, int n, int stride, float *twiddle_factors, int tw_stride);

#endif // __FFT_H__
