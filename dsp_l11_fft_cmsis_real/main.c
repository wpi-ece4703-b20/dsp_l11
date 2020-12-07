#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "msp432_boostxl_init.h"
#include "msp432_arm_dsp.h"
#include <math.h>
#include <stdio.h>

#include "arm_const_structs.h" // data structure for FFT

#define BUFLEN BUFLEN_64
#define BUFLEN_SZ 64
#define N 64

float32_t insamples[N];
float32_t outsamples[N];

arm_rfft_fast_instance_f32 armFFT;

#define TESTFREQ 2500.0
#define SAMPLING_FREQ 8000.0

void initsamples() {
    int i;
    for(i=0 ; i<N ; i++)
        insamples[i] = 0.01 * cosf(2*M_PI*TESTFREQ*i/SAMPLING_FREQ);
}

void initfft() {
    arm_rfft_fast_init_f32(&armFFT, N);
}

void processBuffer(uint16_t x[BUFLEN_SZ], uint16_t y[BUFLEN_SZ]) {
    int i;
    for (i=0; i<N; i++)
        y[i] = f32_to_dac14(outsamples[i]);
}

void perfCheck(uint16_t x[BUFLEN_SZ], uint16_t y[BUFLEN_SZ]) {
    int i;

    for (i=0; i<N; i++)
        insamples[i] = adc14_to_f32(x[i]);

    arm_rfft_fast_f32(&armFFT, insamples, outsamples, 0);

    for (i=0; i<N; i = i + 2)
        y[i] = y[i+1] = f32_to_dac14(0.1*(outsamples[i]*outsamples[i] +
                                 outsamples[i+1]*outsamples[i+1]));

}

int main() {

  initfft();

  msp432_boostxl_init_dma (FS_8000_HZ,
                           BOOSTXL_J1_2_IN,
                           BUFLEN_64,
                           perfCheck);

  int c = measurePerfBuffer(perfCheck);
  printf("Cycles %d\n", c);

  initsamples();
  arm_rfft_fast_f32(&armFFT, insamples, outsamples, 0);

  msp432_boostxl_run();

}
