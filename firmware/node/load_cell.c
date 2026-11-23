/*
 * Load cell (strain gauge bridge) driver using the TI ADS1220 24-bit ADC
 *
 * Up to two full bridges are read differentially: bridge 1 on AIN0-AIN1 and
 * bridge 2 on AIN2-AIN3. The ADC runs in single-shot mode at a high data rate
 * and switches the input multiplexer between the bridges every conversion.
 *
 * Every sample goes through a median-of-3 filter (rejects single spikes) and
 * a 2nd order low-pass filter (two cascaded 1st order stages). The filtered
 * values of all bridges are broadcast together in one
 * pprz.equipment.loadcell.Measurement at a lower rate. The reference is AVDD,
 * so the measurement is ratiometric to the bridge excitation.
 *
 * The SPI peripheral is driven directly in polled mode, so it does not claim
 * DMA channels (SPI1 shares them with USART3 on the STM32F1). Data ready is
 * read from the DOUT/DRDY pin (DRDYM=1), so only SCK, MISO, MOSI and CS are
 * needed.
 */
#include "load_cell.h"
#include "config.h"
#include <math.h>
#include <string.h>
#include <pprz.equipment.loadcell.Measurement.h>

#ifdef LOADCELL_SPI

/* ADS1220 commands */
#define ADS1220_CMD_RESET         0x06
#define ADS1220_CMD_START         0x08
#define ADS1220_CMD_RDATA         0x10
#define ADS1220_CMD_RREG(r, n)    (0x20 | ((r) << 2) | ((n) - 1))
#define ADS1220_CMD_WREG(r, n)    (0x40 | ((r) << 2) | ((n) - 1))

/* ADS1220 register fields */
#define ADS1220_MUX_AIN0_AIN1     0x0
#define ADS1220_MUX_AIN2_AIN3     0x5
#define ADS1220_MODE_TURBO        0x2
#define ADS1220_VREF_AVDD         0x3
#define ADS1220_CONF3_DRDYM       0x02

#define LOAD_CELL_MAX             2
#define LOAD_CELL_MAX_ERRORS      10

struct load_cell_ch_t {
  uint8_t reg0;               ///< Register 0 (input mux and gain) for this bridge
  int32_t med_buf[3];         ///< Median filter samples
  uint8_t med_cnt;            ///< Number of samples in the median filter
  uint8_t med_idx;            ///< Next index in the median filter
  float lp1;                  ///< First low-pass stage
  float lp2;                  ///< Second low-pass stage (output)
  systime_t last_sample;      ///< Time of the last sample
  bool valid;                 ///< Filter is initialized
};

struct load_cell_t {
  float frequency;            ///< Broadcast frequency (Hz)
  uint8_t id;                 ///< Sensor id of the first bridge
  float tau;                  ///< Time constant of one low-pass stage (s), 0 is unfiltered
  uint8_t conf[4];            ///< ADS1220 configuration registers
  sysinterval_t timeout;      ///< Conversion timeout
  uint8_t nb_ch;              ///< Number of enabled bridges
  struct load_cell_ch_t ch[LOAD_CELL_MAX];
  uint32_t errors;            ///< Total conversion errors
};

static struct load_cell_t load_cell;

/* Data rates in normal mode, doubled in turbo mode */
static const uint16_t ads1220_data_rates[] = {20, 45, 90, 175, 330, 600, 1000};
#define ADS1220_NB_RATES (sizeof(ads1220_data_rates) / sizeof(ads1220_data_rates[0]))

/*
 * Polled SPI
 */
static void spi_select(void) {
  palClearLine(LOADCELL_CS_LINE);
}

static void spi_unselect(void) {
  while (LOADCELL_SPI->SR & SPI_SR_BSY);
  palSetLine(LOADCELL_CS_LINE);
}

static uint8_t spi_xfer(uint8_t byte) {
  while (!(LOADCELL_SPI->SR & SPI_SR_TXE));
  LOADCELL_SPI->DR = byte;
  while (!(LOADCELL_SPI->SR & SPI_SR_RXNE));
  return LOADCELL_SPI->DR;
}

static void spi_init(void) {
  palSetLine(LOADCELL_CS_LINE);
  palSetLineMode(LOADCELL_CS_LINE, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLineMode(LOADCELL_SCK_LINE, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
  palSetLineMode(LOADCELL_MOSI_LINE, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
  palSetLineMode(LOADCELL_MISO_LINE, PAL_MODE_INPUT_PULLUP);

  LOADCELL_SPI_RCC_ENABLE();

  // Master, SPI mode 1 (CPOL=0, CPHA=1), 8 bit MSB first, software CS
  LOADCELL_SPI->CR1 = 0;
  LOADCELL_SPI->CR2 = 0;
  LOADCELL_SPI->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_CPHA | LOADCELL_SPI_BR;
  LOADCELL_SPI->CR1 |= SPI_CR1_SPE;
}

/*
 * ADS1220
 */
static bool ads1220_configure(void) {
  // Reset the device
  spi_select();
  spi_xfer(ADS1220_CMD_RESET);
  spi_unselect();
  chThdSleepMilliseconds(1);

  // Write all 4 configuration registers
  spi_select();
  spi_xfer(ADS1220_CMD_WREG(0, 4));
  for (uint8_t i = 0; i < 4; i++) {
    spi_xfer(load_cell.conf[i]);
  }
  spi_unselect();

  // Read back to verify the device is there
  uint8_t readback[4];
  spi_select();
  spi_xfer(ADS1220_CMD_RREG(0, 4));
  for (uint8_t i = 0; i < 4; i++) {
    readback[i] = spi_xfer(0x00);
  }
  spi_unselect();

  return memcmp(readback, load_cell.conf, 4) == 0;
}

static bool ads1220_convert(uint8_t reg0, int32_t *value) {
  // Select the input and start a single-shot conversion
  spi_select();
  spi_xfer(ADS1220_CMD_WREG(0, 1));
  spi_xfer(reg0);
  spi_xfer(ADS1220_CMD_START);
  while (LOADCELL_SPI->SR & SPI_SR_BSY);

  // Keep CS low, DOUT/DRDY goes low when the conversion is done
  systime_t start = chVTGetSystemTimeX();
  while (palReadLine(LOADCELL_MISO_LINE) == PAL_HIGH) {
    if (chVTTimeElapsedSinceX(start) > load_cell.timeout) {
      spi_unselect();
      return false;
    }
    chThdSleep(1);
  }

  // Read the 24 bit result
  spi_xfer(ADS1220_CMD_RDATA);
  uint32_t raw = (uint32_t)spi_xfer(0x00) << 16;
  raw |= (uint32_t)spi_xfer(0x00) << 8;
  raw |= spi_xfer(0x00);
  spi_unselect();

  // Sign extend
  if (raw & 0x800000) {
    raw |= 0xFF000000;
  }
  *value = (int32_t)raw;
  return true;
}

/*
 * Filtering
 */
static int32_t median3(int32_t a, int32_t b, int32_t c) {
  if ((a <= b && b <= c) || (c <= b && b <= a)) {
    return b;
  } else if ((b <= a && a <= c) || (c <= a && a <= b)) {
    return a;
  }
  return c;
}

static void load_cell_filter(struct load_cell_ch_t *ch, int32_t sample, systime_t now) {
  // Median of the last 3 samples against single sample spikes
  ch->med_buf[ch->med_idx] = sample;
  ch->med_idx = (ch->med_idx + 1) % 3;
  if (ch->med_cnt < 3) {
    ch->med_cnt++;
  }
  float x = (ch->med_cnt < 3) ? (float)sample : (float)median3(ch->med_buf[0], ch->med_buf[1], ch->med_buf[2]);

  // 2nd order low-pass using the measured sample time
  if (!ch->valid || load_cell.tau <= 0) {
    ch->lp1 = x;
    ch->lp2 = x;
    ch->valid = true;
  } else {
    float dt = TIME_I2US(chTimeDiffX(ch->last_sample, now)) * 1e-6f;
    float alpha = dt / (load_cell.tau + dt);
    ch->lp1 += alpha * (x - ch->lp1);
    ch->lp2 += alpha * (ch->lp1 - ch->lp2);
  }
  ch->last_sample = now;
}

static void load_cell_reset_filters(void) {
  for (uint8_t i = 0; i < load_cell.nb_ch; i++) {
    load_cell.ch[i].med_cnt = 0;
    load_cell.ch[i].med_idx = 0;
    load_cell.ch[i].valid = false;
  }
}

static void load_cell_broadcast(void) {
  struct pprz_equipment_loadcell_Measurement msg;
  msg.sensor_id = load_cell.id;
  msg.raw.len = load_cell.nb_ch;
  for (uint8_t i = 0; i < load_cell.nb_ch; i++) {
    msg.raw.data[i] = load_cell.ch[i].valid ? load_cell.ch[i].lp2 : NAN;
  }

  uint8_t buffer[PPRZ_EQUIPMENT_LOADCELL_MEASUREMENT_MAX_SIZE];
  uint16_t total_size = pprz_equipment_loadcell_Measurement_encode(&msg, buffer);
  static uint8_t transfer_id;
  uavcanBroadcastAll(
      PPRZ_EQUIPMENT_LOADCELL_MEASUREMENT_SIGNATURE,
      PPRZ_EQUIPMENT_LOADCELL_MEASUREMENT_ID, &transfer_id,
      CANARD_TRANSFER_PRIORITY_LOW, buffer, total_size);
}

static THD_FUNCTION(load_cell_thd, arg) {
  (void)arg;
  chRegSetThreadName("load_cell");

  const sysinterval_t broadcast_interval = TIME_US2I((uint32_t)(1e6f / load_cell.frequency));
  systime_t last_broadcast = chVTGetSystemTimeX();
  uint8_t consecutive_errors = LOAD_CELL_MAX_ERRORS;
  uint8_t ch_idx = 0;

  while (true) {
    // (Re)configure the ADC on startup and after repeated errors
    if (consecutive_errors >= LOAD_CELL_MAX_ERRORS) {
      while (!ads1220_configure()) {
        chThdSleepMilliseconds(100);
      }
      load_cell_reset_filters();
      consecutive_errors = 0;
    }

    // Convert the next bridge
    struct load_cell_ch_t *ch = &load_cell.ch[ch_idx];
    int32_t sample;
    if (ads1220_convert(ch->reg0, &sample)) {
      load_cell_filter(ch, sample, chVTGetSystemTimeX());
      consecutive_errors = 0;
    } else {
      load_cell.errors++;
      consecutive_errors++;
    }
    ch_idx = (ch_idx + 1) % load_cell.nb_ch;

    // Broadcast the filtered values
    sysinterval_t since_broadcast = chTimeDiffX(last_broadcast, chVTGetSystemTimeX());
    if (since_broadcast < broadcast_interval) {
      continue;
    }
    if (since_broadcast >= 2 * broadcast_interval) {
      last_broadcast = chVTGetSystemTimeX(); // Fell behind (e.g. reconfiguring), don't burst
    } else {
      last_broadcast = chTimeAddX(last_broadcast, broadcast_interval);
    }
    load_cell_broadcast();
  }
}

void load_cell_init(void) {
  load_cell.frequency = config_get_f32("LOADCELL frequency", 0.0f);
  load_cell.id = config_get_u8("LOADCELL id", 0);
  load_cell.nb_ch = config_get_u8("LOADCELL bridges", 1);
  if (load_cell.frequency <= 0 || load_cell.nb_ch == 0) {
    return;
  }
  if (load_cell.nb_ch > LOAD_CELL_MAX) {
    load_cell.nb_ch = LOAD_CELL_MAX;
  }

  // Gain must be a power of 2 up to 128
  uint8_t gain = config_get_u8("LOADCELL gain", 128);
  uint8_t gain_code = 0;
  while (gain_code < 7 && (1U << (gain_code + 1)) <= gain) {
    gain_code++;
  }

  // Bridge 1 on AIN0-AIN1, bridge 2 on AIN2-AIN3
  static const uint8_t muxes[LOAD_CELL_MAX] = {ADS1220_MUX_AIN0_AIN1, ADS1220_MUX_AIN2_AIN3};
  for (uint8_t i = 0; i < load_cell.nb_ch; i++) {
    load_cell.ch[i].reg0 = (muxes[i] << 4) | (gain_code << 1);
  }

  // Lowest data rate that is at least the requested rate (turbo mode doubles the normal rates)
  uint16_t req_rate = config_get_u16("LOADCELL data rate", 1000);
  uint8_t dr = ADS1220_NB_RATES - 1;
  bool turbo = true;
  for (uint8_t i = 0; i < 2 * ADS1220_NB_RATES; i++) {
    bool t = (i >= ADS1220_NB_RATES);
    uint8_t d = i % ADS1220_NB_RATES;
    uint32_t r = ads1220_data_rates[d] * (t ? 2 : 1);
    if (r >= req_rate) {
      dr = d;
      turbo = t;
      break;
    }
  }
  load_cell.conf[0] = load_cell.ch[0].reg0;
  load_cell.conf[1] = (dr << 5) | ((turbo ? ADS1220_MODE_TURBO : 0) << 3); // single-shot
  load_cell.conf[2] = (ADS1220_VREF_AVDD << 6);  // no 50/60Hz filter, no low-side switch, no IDACs
  load_cell.conf[3] = ADS1220_CONF3_DRDYM;

  // Conversion timeout of 3 conversion periods plus margin
  uint32_t rate = ads1220_data_rates[dr] * (turbo ? 2 : 1);
  load_cell.timeout = TIME_US2I(3000000UL / rate + 2000);

  // Low-pass filter: two equal 1st order stages with a combined -3dB point at the cutoff
  float cutoff = config_get_f32("LOADCELL cutoff (Hz)", 5.0f);
  if (cutoff > 0) {
    float stage_cutoff = cutoff / sqrtf(sqrtf(2.0f) - 1.0f);
    load_cell.tau = 1.0f / (2.0f * M_PI * stage_cutoff);
  } else {
    load_cell.tau = 0;
  }

  spi_init();
  chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(1024), "load_cell", NORMALPRIO - 10, load_cell_thd, NULL);
}

#else /* LOADCELL_SPI */

void load_cell_init(void) {}

#endif /* LOADCELL_SPI */
