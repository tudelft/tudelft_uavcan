#ifndef ADCS_H
#define ADCS_H

#include <stdint.h>

#define MAX_AV_NB_SAMPLE 0x20

struct adc_buf {
  uint32_t sum;                      /* Sum of samples in buffer (avg = sum / av_nb_sample) */
  uint16_t values[MAX_AV_NB_SAMPLE]; /* Buffer for sample values from ADC                   */
  uint8_t  head;                     /* Position index of write head in buffer              */
  uint8_t  av_nb_sample;             /* Number of samples to use in buffer (used for avg)   */
};

extern struct adc_buf adc1_buffers[];

void adcs_init(void);

#endif /* ADCS_H */