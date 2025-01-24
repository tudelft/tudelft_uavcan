
#include <ch.h>
#include <hal.h>
#include <math.h>

#include "adcs.h"
#include "config.h"

static ADCConversionGroup adc1_group = {0};
static adcsample_t adc_samples[ADC_MAX_CHANNELS * MAX_AV_NB_SAMPLE] = {0};
struct adc_buf adc1_buffers[ADC_MAX_CHANNELS] = {0};
static uint32_t adc1_sum_tmp[ADC_MAX_CHANNELS] = {0};
static uint8_t adc1_samples_tmp[ADC_MAX_CHANNELS] = {0};
static uint8_t adc1_channel_map[ADC_MAX_CHANNELS] = {0};
static uint8_t adc1_num_channels = 0;

struct adc_chan_config_t {
  bool available;
  ioline_t line;
  uint8_t channel;
};
static struct adc_chan_config_t adc1_chan_config[ADC_MAX_CHANNELS] = {0};

struct adc_ntc_t {
  uint8_t channel;
  uint8_t channel_idx;
  uint8_t device_id;
  float frequency;

  float pullup_r;
  float she_a;
  float she_b;
  float she_c;
};

static struct adc_ntc_t ntc1;
static THD_WORKING_AREA(ntc1_wa, 512);
static struct adc_ntc_t ntc2;
static THD_WORKING_AREA(ntc2_wa, 512);

struct adc_power_t {
  uint8_t device_id;
  float frequency;

  // ADC values
  int16_t current_channel;
  uint8_t current_channel_idx;
  int16_t power_channel;
  uint8_t power_channel_idx;

  // Calibration values
  float power_mult;
  float power_offset;
  float current_mult;
  float current_offset;
};

static struct adc_power_t power1;
static THD_WORKING_AREA(power1_wa, 512);
static struct adc_power_t power2;
static THD_WORKING_AREA(power2_wa, 512);

struct potmeter_t {
  uint8_t channel;
  uint8_t channel_idx;
  uint8_t device_id;
  float frequency;
  uint8_t type;         ///< 0: Actuator angle

  float cal_a;          ///< (x - a) * b
  float cal_b;          ///< (x - a) * b
};

static struct potmeter_t potmeter1;
static THD_WORKING_AREA(potmeter1_wa, 512);
static struct potmeter_t potmeter2;
static THD_WORKING_AREA(potmeter2_wa, 512);


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
    const size_t n = MAX_AV_NB_SAMPLE / 2U;
    // depending on half buffer that has just been filled
    // if adcIsBufferComplete return true, the last filled
    // half buffer start in the middle of buffer, else, is start at
    // beginiing of buffer
    const adcsample_t *buffer = adc_samples;
    if (adcIsBufferComplete(adcp)) {
      buffer = &adc_samples[n * adc1_num_channels];
    }

    for (uint8_t c = 0; c < adc1_num_channels; c++) {
      adc1_sum_tmp[c] = 0;
      if (n > 0) {
        adc1_samples_tmp[c] = n;
      } else {
        adc1_samples_tmp[c] = 1;
      }
      for (unsigned int sample = 0; sample < n; sample++) {
        adc1_sum_tmp[c] += buffer[c + sample * adc1_num_channels];
      }
    }
    chSysLockFromISR();
    for (uint8_t channel = 0; channel < adc1_num_channels; channel++) {
      adc1_buffers[channel].sum = adc1_sum_tmp[channel];
      adc1_buffers[channel].av_nb_sample = adc1_samples_tmp[channel];
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

/*
 * NTC thread.
 */
static THD_FUNCTION(ntc_thread, p) {
  struct adc_ntc_t *ntc = (struct adc_ntc_t *)p;

  uint64_t vt_delay = 1000.f / ntc->frequency;
  while(true) {
    // Calculate the temperature
    float raw_adc = (adc1_buffers[ntc->channel_idx].sum / adc1_buffers[ntc->channel_idx].av_nb_sample);
    float log_r = log((ntc->pullup_r * raw_adc) / (4095 - raw_adc));
    float temp = 1 / (ntc->she_a + (ntc->she_b * log_r) + (ntc->she_c * log_r * log_r * log_r));

    // Set the values
    struct uavcan_equipment_device_Temperature deviceTemperature;
    deviceTemperature.device_id = ntc->device_id;
    deviceTemperature.temperature = temp;

    uint8_t buffer[UAVCAN_EQUIPMENT_DEVICE_TEMPERATURE_MAX_SIZE];
    uint16_t total_size = uavcan_equipment_device_Temperature_encode(&deviceTemperature, buffer);

    static uint8_t transfer_id;
    uavcanBroadcastAll(
        UAVCAN_EQUIPMENT_DEVICE_TEMPERATURE_SIGNATURE,
        UAVCAN_EQUIPMENT_DEVICE_TEMPERATURE_ID, &transfer_id,
        CANARD_TRANSFER_PRIORITY_LOW, buffer, total_size);
 
    chThdSleepMilliseconds(vt_delay);
  }
}

/*
 * POWER thread.
 */
static THD_FUNCTION(power_thread, p) {
  struct adc_power_t *power = (struct adc_power_t *)p;

  uint64_t vt_delay = 1000.f / power->frequency;
  while(true) {
    // Calculate the current
    float raw_adc, current, voltage = 0;
    if(power->current_channel >= 0 && adc1_buffers[power->current_channel_idx].av_nb_sample > 0) {
      raw_adc = (adc1_buffers[power->current_channel_idx].sum / adc1_buffers[power->current_channel_idx].av_nb_sample);
      current = (raw_adc / 4095) * 3.3 * power->current_mult + power->current_offset;
    }

    // Calculate the voltage
    if(power->power_channel >= 0 && adc1_buffers[power->power_channel_idx].av_nb_sample > 0) {
      raw_adc = (adc1_buffers[power->power_channel_idx].sum / adc1_buffers[power->power_channel_idx].av_nb_sample);
      voltage = (raw_adc / 4095) * 3.3 * power->power_mult + power->power_offset;
    }

    // Set the values
    struct uavcan_equipment_power_CircuitStatus circuitStatus;
    circuitStatus.circuit_id = power->device_id;
    circuitStatus.current = current;
    circuitStatus.voltage = voltage;

    uint8_t buffer[UAVCAN_EQUIPMENT_POWER_CIRCUITSTATUS_MAX_SIZE];
    uint16_t total_size = uavcan_equipment_power_CircuitStatus_encode(&circuitStatus, buffer);

    static uint8_t transfer_id;
    uavcanBroadcastAll(
        UAVCAN_EQUIPMENT_POWER_CIRCUITSTATUS_SIGNATURE,
        UAVCAN_EQUIPMENT_POWER_CIRCUITSTATUS_ID, &transfer_id,
        CANARD_TRANSFER_PRIORITY_LOW, buffer, total_size);

    // struct uavcan_equipment_power_BatteryInfo batteryInfo;
    // batteryInfo.battery_id = power->device_id;
    // batteryInfo.current = current;
    // batteryInfo.voltage = voltage;

    // uint8_t buffer[UAVCAN_EQUIPMENT_POWER_BATTERYINFO_MAX_SIZE];
    // uint16_t total_size = uavcan_equipment_power_BatteryInfo_encode(&batteryInfo, buffer);

    // static uint8_t transfer_id;
    // uavcanBroadcastAll(
    //     UAVCAN_EQUIPMENT_POWER_BATTERYINFO_SIGNATURE,
    //     UAVCAN_EQUIPMENT_POWER_BATTERYINFO_ID, &transfer_id,
    //     CANARD_TRANSFER_PRIORITY_LOW, buffer, total_size);
 
    chThdSleepMilliseconds(vt_delay);
  }
}

/*
 * POTMETER thread.
 */
static THD_FUNCTION(potmeter_thread, p) {
  struct potmeter_t *potmeter = (struct potmeter_t *)p;

  uint64_t vt_delay = 1000.f / potmeter->frequency;
  while(true) {
    // Calculate the temperature
    float raw_adc = 0;
    if(adc1_buffers[potmeter->channel_idx].av_nb_sample > 0)
      raw_adc = (adc1_buffers[potmeter->channel_idx].sum / adc1_buffers[potmeter->channel_idx].av_nb_sample);
    float scaled_value = (raw_adc - potmeter->cal_a) * potmeter->cal_b;

    // Set the values
    struct uavcan_equipment_actuator_Status actuatorStatus;
    actuatorStatus.actuator_id = potmeter->device_id;
    actuatorStatus.position = scaled_value;
    actuatorStatus.force = raw_adc; // For debugging

    uint8_t buffer[UAVCAN_EQUIPMENT_ACTUATOR_STATUS_MAX_SIZE];
    uint16_t total_size = uavcan_equipment_actuator_Status_encode(&actuatorStatus, buffer);

    static uint8_t transfer_id;
    uavcanBroadcastAll(
        UAVCAN_EQUIPMENT_ACTUATOR_STATUS_SIGNATURE,
        UAVCAN_EQUIPMENT_ACTUATOR_STATUS_ID, &transfer_id,
        CANARD_TRANSFER_PRIORITY_LOW, buffer, total_size);
 
    chThdSleepMilliseconds(vt_delay);
  }
}

void adcs_init(void) {
  adc1_num_channels = 0;

  /* Define all available channels */
#ifdef ADC_CHAN0_LINE
  adc1_chan_config[0].channel = ADC_CHANNEL_IN0;
  adc1_chan_config[0].line = ADC_CHAN0_LINE;
  adc1_chan_config[0].available = true;
#endif
#ifdef ADC_CHAN1_LINE
  adc1_chan_config[1].channel = ADC_CHANNEL_IN1;
  adc1_chan_config[1].line = ADC_CHAN1_LINE;
  adc1_chan_config[1].available = true;
#endif
#ifdef ADC_CHAN2_LINE
  adc1_chan_config[2].channel = ADC_CHANNEL_IN2;
  adc1_chan_config[2].line = ADC_CHAN2_LINE;
  adc1_chan_config[2].available = true;
#endif
#ifdef ADC_CHAN3_LINE
  adc1_chan_config[3].channel = ADC_CHANNEL_IN3;
  adc1_chan_config[3].line = ADC_CHAN3_LINE;
  adc1_chan_config[3].available = true;
#endif
#ifdef ADC_CHAN4_LINE
  adc1_chan_config[4].channel = ADC_CHANNEL_IN4;
  adc1_chan_config[4].line = ADC_CHAN4_LINE;
  adc1_chan_config[4].available = true;
#endif
#ifdef ADC_CHAN5_LINE
  adc1_chan_config[5].channel = ADC_CHANNEL_IN5;
  adc1_chan_config[5].line = ADC_CHAN5_LINE;
  adc1_chan_config[5].available = true;
#endif
#ifdef ADC_CHAN6_LINE
  adc1_chan_config[6].channel = ADC_CHANNEL_IN6;
  adc1_chan_config[6].line = ADC_CHAN6_LINE;
  adc1_chan_config[6].available = true;
#endif
#ifdef ADC_CHAN7_LINE
  adc1_chan_config[7].channel = ADC_CHANNEL_IN7;
  adc1_chan_config[7].line = ADC_CHAN7_LINE;
  adc1_chan_config[7].available = true;
#endif
#ifdef ADC_CHAN8_LINE
  adc1_chan_config[8].channel = ADC_CHANNEL_IN8;
  adc1_chan_config[8].line = ADC_CHAN8_LINE;
  adc1_chan_config[8].available = true;
#endif
#ifdef ADC_CHAN9_LINE
  adc1_chan_config[9].channel = ADC_CHANNEL_IN9;
  adc1_chan_config[9].line = ADC_CHAN9_LINE;
  adc1_chan_config[9].available = true;
#endif

  /* Define the power inputs */
  power1.device_id = config_get_by_name("POWER1 device id", 0)->val.i;
  power1.frequency = config_get_by_name("POWER1 frequency", 0)->val.f;
  power1.power_channel = config_get_by_name("POWER1 volt chan", 0)->val.i;
  power1.current_channel = config_get_by_name("POWER1 current chan", 0)->val.i;
  power1.power_mult = config_get_by_name("POWER1 volt mult", 0)->val.f;
  power1.power_offset = config_get_by_name("POWER1 volt offset", 0)->val.f;
  power1.current_mult = config_get_by_name("POWER1 current mult", 0)->val.f;
  power1.current_offset = config_get_by_name("POWER1 current offset", 0)->val.f;

  if(power1.frequency > 0 && adc1_chan_config[power1.power_channel].available) {
    power1.power_channel_idx = adc1_num_channels++;
    adc1_channel_map[power1.power_channel_idx] = adc1_chan_config[power1.power_channel].channel;
    palSetLineMode(adc1_chan_config[power1.power_channel].line, PAL_MODE_INPUT_ANALOG);
  }
  if(power1.frequency > 0 && adc1_chan_config[power1.current_channel].available) {
    power1.current_channel_idx = adc1_num_channels++;
    adc1_channel_map[power1.current_channel_idx] = adc1_chan_config[power1.current_channel].channel;
    palSetLineMode(adc1_chan_config[power1.current_channel].line, PAL_MODE_INPUT_ANALOG);
  }

  power2.device_id = config_get_by_name("POWER2 device id", 0)->val.i;
  power2.frequency = config_get_by_name("POWER2 frequency", 0)->val.f;
  power2.power_channel = config_get_by_name("POWER2 volt chan", 0)->val.i;
  power2.current_channel = config_get_by_name("POWER2 current chan", 0)->val.i;
  power2.power_mult = config_get_by_name("POWER2 volt mult", 0)->val.f;
  power2.power_offset = config_get_by_name("POWER2 volt offset", 0)->val.f;
  power2.current_mult = config_get_by_name("POWER2 current mult", 0)->val.f;
  power2.current_offset = config_get_by_name("POWER2 current offset", 0)->val.f;

  if(power2.frequency > 0 && adc1_chan_config[power2.power_channel].available) {
    power2.power_channel_idx = adc1_num_channels++;
    adc1_channel_map[power2.power_channel_idx] = adc1_chan_config[power2.power_channel].channel;
    palSetLineMode(adc1_chan_config[power2.power_channel].line, PAL_MODE_INPUT_ANALOG);
  }
  if(power2.frequency > 0 && adc1_chan_config[power2.current_channel].available) {
    power2.current_channel_idx = adc1_num_channels++;
    adc1_channel_map[power2.current_channel_idx] = adc1_chan_config[power2.current_channel].channel;
    palSetLineMode(adc1_chan_config[power2.current_channel].line, PAL_MODE_INPUT_ANALOG);
  }

  /* Possible NTC inputs */
  ntc1.device_id = config_get_by_name("NTC1 device id", 0)->val.i;
  ntc1.channel = config_get_by_name("NTC1 channel", 0)->val.i;
  ntc1.frequency = config_get_by_name("NTC1 frequency", 0)->val.f;
  ntc1.pullup_r = config_get_by_name("NTC1 pull up R", 0)->val.f;
  ntc1.she_a = config_get_by_name("NTC1 SH eq a", 0)->val.f;
  ntc1.she_b = config_get_by_name("NTC1 SH eq b", 0)->val.f;
  ntc1.she_c = config_get_by_name("NTC1 SH eq c", 0)->val.f;
  if(ntc1.frequency > 0 && adc1_chan_config[ntc1.channel].available) {
    ntc1.channel_idx = adc1_num_channels++;
    adc1_channel_map[ntc1.channel_idx] = adc1_chan_config[ntc1.channel].channel;
    palSetLineMode(adc1_chan_config[ntc1.channel].line, PAL_MODE_INPUT_ANALOG);
  }

  ntc2.device_id = config_get_by_name("NTC2 device id", 0)->val.i;
  ntc2.channel = config_get_by_name("NTC2 channel", 0)->val.i;
  ntc2.frequency = config_get_by_name("NTC2 frequency", 0)->val.f;
  ntc2.pullup_r = config_get_by_name("NTC2 pull up R", 0)->val.f;
  ntc2.she_a = config_get_by_name("NTC2 SH eq a", 0)->val.f;
  ntc2.she_b = config_get_by_name("NTC2 SH eq b", 0)->val.f;
  ntc2.she_c = config_get_by_name("NTC2 SH eq c", 0)->val.f;
  if(ntc2.frequency > 0 && adc1_chan_config[ntc2.channel].available) {
    ntc2.channel_idx = adc1_num_channels++;
    adc1_channel_map[ntc2.channel_idx] = adc1_chan_config[ntc2.channel].channel;
    palSetLineMode(adc1_chan_config[ntc2.channel].line, PAL_MODE_INPUT_ANALOG);
  }

  /* Possible POTMETER inputs */
  potmeter1.device_id = config_get_by_name("POTMETER1 device id", 0)->val.i;
  potmeter1.channel = config_get_by_name("POTMETER1 channel", 0)->val.i;
  potmeter1.frequency = config_get_by_name("POTMETER1 frequency", 0)->val.f;
  potmeter1.type = config_get_by_name("POTMETER1 type", 0)->val.i;
  potmeter1.cal_a = config_get_by_name("POTMETER1 cal_a", 0)->val.f;
  potmeter1.cal_b = config_get_by_name("POTMETER1 cal_b", 0)->val.f;
  if(potmeter1.frequency > 0 && adc1_chan_config[potmeter1.channel].available) {
    potmeter1.channel_idx = adc1_num_channels++;
    adc1_channel_map[potmeter1.channel_idx] = adc1_chan_config[potmeter1.channel].channel;
    palSetLineMode(adc1_chan_config[potmeter1.channel].line, PAL_MODE_INPUT_ANALOG);
  }

  potmeter2.device_id = config_get_by_name("POTMETER2 device id", 0)->val.i;
  potmeter2.channel = config_get_by_name("POTMETER2 channel", 0)->val.i;
  potmeter2.frequency = config_get_by_name("POTMETER2 frequency", 0)->val.f;
  potmeter2.type = config_get_by_name("POTMETER2 type", 0)->val.i;
  potmeter2.cal_a = config_get_by_name("POTMETER2 cal_a", 0)->val.f;
  potmeter2.cal_b = config_get_by_name("POTMETER2 cal_b", 0)->val.f;
  if(potmeter2.frequency > 0 && adc1_chan_config[potmeter2.channel].available) {
    potmeter2.channel_idx = adc1_num_channels++;
    adc1_channel_map[potmeter2.channel_idx] = adc1_chan_config[potmeter2.channel].channel;
    palSetLineMode(adc1_chan_config[potmeter2.channel].line, PAL_MODE_INPUT_ANALOG);
  }

  // Configure the ADC structure
  adc_configure(&adc1_group, adc1_num_channels, adc1_channel_map, ADC_SAMPLE_41P5, adc1callback, adc1errorcallback);

  // Start ADC in continious conversion mode
  adcStart(&ADCD1, NULL);
  adcStartConversion(&ADCD1, &adc1_group, adc_samples, MAX_AV_NB_SAMPLE);

  // Start POWER transmitting threads
  if(power1.frequency > 0) {
    chThdCreateStatic(power1_wa, sizeof(power1_wa), NORMALPRIO-22, power_thread, (void*)&power1);
  }
  if(power2.frequency > 0) {
    chThdCreateStatic(power2_wa, sizeof(power2_wa), NORMALPRIO-22, power_thread, (void*)&power2);
  }

  // Start NTC transmitting threads
  if(ntc1.frequency > 0) {
    chThdCreateStatic(ntc1_wa, sizeof(ntc1_wa), NORMALPRIO-21, ntc_thread, (void*)&ntc1);
  }
  if(ntc2.frequency > 0) {
    chThdCreateStatic(ntc2_wa, sizeof(ntc2_wa), NORMALPRIO-21, ntc_thread, (void*)&ntc2);
  }

  // Start POTMETER transmitting threads
  if(potmeter1.frequency > 0) {
    chThdCreateStatic(potmeter1_wa, sizeof(potmeter1_wa), NORMALPRIO-20, potmeter_thread, (void*)&potmeter1);
  }
  if(potmeter2.frequency > 0) {
    chThdCreateStatic(potmeter2_wa, sizeof(potmeter2_wa), NORMALPRIO-20, potmeter_thread, (void*)&potmeter2);
  }
}

float adcs_get_value(uint8_t channel) {
  if(channel >= adc1_num_channels) {
    return 0;
  }

  return (adc1_buffers[channel].sum / adc1_buffers[channel].av_nb_sample);
}