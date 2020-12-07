#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "msp432_boostxl_init.h"
#include "msp432_arm_dsp.h"
#include <math.h>
#include <stdio.h>

#include "arm_const_structs.h" // data structure for FFT

#define BUFLEN BUFLEN_64
#define BUFLEN_SZ 64
#define N 64

float32_t samples[2*N];

#define TESTFREQ 2500.0
#define SAMPLING_FREQ 8000.0

void initsamples() {
    int i;
    for(i=0 ; i<N ; i++) {
        samples[2*i]   = 0.01 * cosf(2*M_PI*TESTFREQ*i/SAMPLING_FREQ);
        samples[2*i+1] = 0.0;
    }
}

void processBuffer(uint16_t x[BUFLEN_SZ], uint16_t y[BUFLEN_SZ]) {
    int i;
    for (i=0; i<N; i++) {
        y[i] = f32_to_dac14(samples[2*i]);
    }
}

void perfCheck(uint16_t x[BUFLEN_SZ], uint16_t y[BUFLEN_SZ]) {
    int i;
    for (i=0; i<N; i=i+1) {
        samples[2*i]     = adc14_to_f32(x[i]);
        samples[2*i + 1] = 0.0;
    }
    arm_cfft_f32(&arm_cfft_sR_f32_len64, samples, 0, 1);
    for (i=0; i<N; i=i+1) {
        y[i] = f32_to_dac14(0.1*(samples[2*i]*samples[2*i] +
                                 samples[2*i+1]*samples[2*i+1]));
    }
}

int main() {

  msp432_boostxl_init_dma (FS_8000_HZ,
                           BOOSTXL_J1_2_IN,
                           BUFLEN_64,
                           perfCheck);

  int c = measurePerfBuffer(perfCheck);
  printf("Cycles %d\n", c);

  initsamples();
  arm_cfft_f32(&arm_cfft_sR_f32_len64, samples, 0, 1);

  msp432_boostxl_run();

}
