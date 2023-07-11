#include "adcs.h"

#include "config.h"

#include <ch.h>
#include <hal.h>


#define ADC_BUF_DEPTH (MAX_AV_NB_SAMPLE/2)


static ADCConversionGroup adc1_group;
static adcsample_t adc_samples[ADC_NUM_CHANNELS * MAX_AV_NB_SAMPLE];
struct adc_buf adc1_buffers[ADC_NUM_CHANNELS];
static uint32_t adc1_sum_tmp[ADC_NUM_CHANNELS];
static uint8_t adc1_samples_tmp[ADC_NUM_CHANNELS];

/**
 * Adc1 callback
 *
 * Callback, fired after half of the buffer is filled (i.e. half of the samples
 * is collected). Since we are assuming continuous ADC conversion, the ADC state is
 * never equal to ADC_COMPLETE.
 *
 * @note    Averaging is done when the modules ask for ADC values
 * @param[in] adcp pointer to a @p ADCDriver object
 * @param[in] buffer pointer to a @p buffer with samples
 * @param[in] n number of samples
 */
void adc1callback(ADCDriver *adcp)
{
  if (adcp->state != ADC_STOP) {
    const size_t n = ADC_BUF_DEPTH / 2U;
    // depending on half buffer that has just been filled
    // if adcIsBufferComplete return true, the last filled
    // half buffer start in the middle of buffer, else, is start at
    // beginiing of buffer
    const adcsample_t *buffer = adc_samples; //+ (adcIsBufferComplete(adcp) ? n *ADC_NUM_CHANNELS : 0U);

    for (int channel = 0; channel < ADC_NUM_CHANNELS; channel++) {
      //if (adc1_buffers[channel] != NULL) {
        adc1_sum_tmp[channel] = 0;
        if (n > 0) {
          adc1_samples_tmp[channel] = n;
        } else {
          adc1_samples_tmp[channel] = 1;
        }
        for (unsigned int sample = 0; sample < n; sample++) {
          adc1_sum_tmp[channel] += buffer[channel + sample * ADC_NUM_CHANNELS];
        }
      //}
    }
    chSysLockFromISR();
    for (int channel = 0; channel < ADC_NUM_CHANNELS; channel++) {
      //if (adc1_buffers[channel] != NULL) {
        adc1_buffers[channel].sum = adc1_sum_tmp[channel];
        adc1_buffers[channel].av_nb_sample = adc1_samples_tmp[channel];
      //}
    }

    chSysUnlockFromISR();
  }
}

static void adc1errorcallback(ADCDriver *adcp, adcerror_t err) {

  (void)adcp;
  (void)err;
}

/**
 * @brief Configure the ADC conversion group depending on the architecture
 *
 * @param cfg The configuration to be set
 * @param num_channels The number of channels in the ADC
 * @param channels The channel mapping to real channels
 * @param sample_rate The sample rate for all channels
 * @param end_cb The callback function at the end of conversion
 * @param error_cb The callback function whenever an error occurs
 */
static void adc_configure(ADCConversionGroup *cfg, uint8_t num_channels, const uint8_t channels[], uint32_t sample_rate,
                          adccallback_t end_cb, adcerrorcallback_t error_cb)
{
  // Set the general configuration
  cfg->circular = true;
  cfg->num_channels = num_channels;
  cfg->end_cb = end_cb;
  cfg->error_cb = error_cb;

  // Set to 16bits by default else try 12bit
#if defined(ADC_CFGR_RES_16BITS)
  cfg->cfgr = ADC_CFGR_CONT | ADC_CFGR_RES_16BITS;
#elif defined(ADC_CFGR_RES_12BITS)
  cfg->cfgr = ADC_CFGR_CONT | ADC_CFGR_RES_12BITS;
#else
  cfg->sqr1 = ADC_SQR1_NUM_CH(num_channels);
  //cfg->cr2 = ADC_CR2_SWSTART;

#if defined(ADC_CR2_TSVREFE)
  //cfg->cr2 |= ADC_CR2_TSVREFE;
#endif
#endif

  // Go through all the channels
  for (uint8_t i = 0; i < num_channels; i++) {
    uint8_t chan = channels[i];

#if defined(STM32H7XX) || defined(STM32F3XX) || defined(STM32G4XX) || defined(STM32L4XX)
    cfg->pcsel |= (1 << chan);
    cfg->smpr[chan / 10] |= sample_rate << (3 << (chan % 10));

    if (i < 4) {
      cfg->sqr[0] |= chan << (6 * (i + 1));
    } else if (i < 9) {
      cfg->sqr[1] |= chan << (6 * (i - 4));
    } else {
      cfg->sqr[2] |= chan << (6 * (i - 9));
    }
#else
    if (chan < 10) {
      cfg->smpr2 |= sample_rate << (3 * chan);
    } else {
      cfg->smpr1 |= sample_rate << (3 * (chan - 10));
    }

    if (i < 6) {
      cfg->sqr3 |= chan << (5 * i);
    } else if (i < 12) {
      cfg->sqr2 |= chan << (5 * (i - 6));
    } else {
      cfg->sqr3 |= chan << (5 * (i - 12));
    }
#endif
  }
}

void adcs_init(void) {
  static const uint8_t adc_channel_map[ADC_NUM_CHANNELS] = {
    ADC_CHANNEL_IN8,
    ADC_CHANNEL_IN9
  };

  palSetLineMode(ADC1_LINE, PAL_MODE_INPUT_ANALOG);
  palSetLineMode(ADC2_LINE, PAL_MODE_INPUT_ANALOG);

  // Configure the ADC structure
  adc_configure(&adc1_group, ADC_NUM_CHANNELS, adc_channel_map, ADC_SAMPLE_41P5, adc1callback, adc1errorcallback);

  // Start ADC in continious conversion mode
  adcStart(&ADCD1, NULL);
  adcStartConversion(&ADCD1, &adc1_group, adc_samples, ADC_BUF_DEPTH);

  adc1_buffers[0].sum = 99;
}