#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "msp432_boostxl_init.h"
#include "msp432_arm_dsp.h"
#include <math.h>
#include <stdio.h>

#define BUFLEN BUFLEN_64
#define BUFLEN_SZ 64
#define N 64

typedef struct {
  float32_t real;
  float32_t imag;
} COMPLEX;

COMPLEX samples[N];
COMPLEX twiddle[N];

#define TESTFREQ 2500.0
#define SAMPLING_FREQ 8000.0

void inittwiddle() {
    int i;
    for(i=0 ; i<N ; i++) {
        twiddle[i].real =  cos(2*M_PI*i/N);
        twiddle[i].imag = -sin(2*M_PI*i/N);
    }
}

void initsamples() {
    int i;
    for(i=0 ; i<N ; i++) {
        samples[i].real = 0.01 * cosf(2*M_PI*TESTFREQ*i/SAMPLING_FREQ);
        samples[i].imag = 0.0;
    }
}

void dft() {
  COMPLEX result[N];
  int k,n;

  for (k=0 ; k<N ; k++)  {

    result[k].real = 0.0;
    result[k].imag = 0.0;

    for (n=0 ; n<N ; n++) {
         result[k].real += samples[n].real*twiddle[(n*k)%N].real
                         - samples[n].imag*twiddle[(n*k)%N].imag;
         result[k].imag += samples[n].imag*twiddle[(n*k)%N].real
                         + samples[n].real*twiddle[(n*k)%N].imag;
    }
  }

  for (k=0 ; k<N ; k++) {
    samples[k] = result[k];
  }
}

void processBuffer(uint16_t x[BUFLEN_SZ], uint16_t y[BUFLEN_SZ]) {
    int i;
    for (i=0; i<N; i++) {
        y[i] = f32_to_dac14(samples[i].real);
    }
}

void perfCheck(uint16_t x[BUFLEN_SZ], uint16_t y[BUFLEN_SZ]) {
    int i;
    for (i=0; i<N; i++) {
        samples[i].real = adc14_to_f32(x[i]);
    }
    dft();
    for (i=0; i<N; i++) {
        y[i] = f32_to_dac14(samples[i].real);
    }
}

int main() {
  int n;

  inittwiddle();

  msp432_boostxl_init_dma (FS_8000_HZ,
                           BOOSTXL_J1_2_IN,
                           BUFLEN_64,
                           processBuffer);

  int c = measurePerfBuffer(perfCheck);
  printf("Cycles %d\n", c);

  initsamples();
  dft();

  msp432_boostxl_run();

}
