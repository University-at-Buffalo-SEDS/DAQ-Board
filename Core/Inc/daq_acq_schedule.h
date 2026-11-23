#ifndef DAQ_ACQ_SCHEDULE_H
#define DAQ_ACQ_SCHEDULE_H
/* MCP3564R acquisition frame schedule. Pure: no HAL, no RTOS, so the host
 * tests run the exact tables and arithmetic the firmware uses.
 *
 * One frame every DAQ_ACQ_FRAME_US (1 ms), started by a TIM2 compare:
 *
 *   t = 0            scan A: KG1000 (CH0) + chamber PT (CH6), one-shot SCAN
 *   t = slot offset  scan B: one slow item: KG50 in most frames; in FILL the
 *                    die temperature in frame 8 of every 10; one auxiliary
 *                    channel (rotating) in frame 9 of every 10
 *
 * Scan A never changes during a profile, so both fast channels are converted
 * at exactly the same instants in every frame (1 kHz, no gaps). Scan B uses
 * whatever gain/OSR its item needs and ends before the next frame. All SPI
 * configuration happens between scans while the ADC is in standby, so a
 * register write never restarts a running conversion.
 *
 * Two profiles trade ADC time between the fast pair and the slow item:
 *   FILL   (default): fast pair OSR 128, slow item OSR 256 (KG50 is the
 *          channel that matters while filling), KG50 in 8 of 10 frames;
 *   LAUNCH (launch window and its 0.5 s pre-roll): fast pair OSR 256,
 *          slow item OSR 64 (thrust and chamber pressure matter), KG50 in 9
 *          of 10 frames, no temperature conversion.
 * Both profiles sample the fast pair at the same 1 kHz instants. */
#include <stdint.h>
#include "daq_chamber_pressure.h"

#define DAQ_ACQ_PROFILE_FILL 0U
#define DAQ_ACQ_PROFILE_LAUNCH 1U

/* Fast-channel sample period. */
#ifndef DAQ_ACQ_FRAME_US
#define DAQ_ACQ_FRAME_US 1000U
#endif
/* The slot pattern repeats every DAQ_ACQ_AUX_EVERY frames (10 ms), which
 * divides the 20 ms fast and 50 ms KG50 report bins: every report bin holds
 * whole, identical patterns. The temperature conversion (the only scan on a
 * different ADC clock) sits at a fixed position in every 10 ms pattern, so
 * any settling it causes repeats at 100 Hz and averages out of every bin,
 * instead of beating with the reports as the former 100 ms passes did. */
#define DAQ_ACQ_AUX_EVERY 10U
#define DAQ_ACQ_TEMPERATURE_FRAME 8U /* FILL only */
#define DAQ_ACQ_ITEM_TEMPERATURE 12U /* MCP3564R TEMP channel ID */
#define DAQ_ACQ_ITEM_P6 9U           /* CH2-CH3 differential channel ID */
#define DAQ_ACQ_ITEM_NONE 0xFFU

#define DAQ_ADC_MCLK_HZ 16000000U
/* CONFIG0: internal 2.4 V reference, external MCLK, standby. START (fast
 * command) sets conversion mode; each configuration burst writes CONFIG0
 * first, so it also stops any conversion before changing anything. */
#define DAQ_ACQ_CONFIG0_STANDBY 0x82U
/* CONFIG2: BOOST x2, gain x1 (0xCF) or x16 (0xEF), AZ_MUX on. */
#define DAQ_ACQ_CONFIG2_GAIN1 0xCFU
#define DAQ_ACQ_CONFIG2_GAIN16 0xEFU
#define DAQ_ACQ_P6_GAIN 16.0f
/* CONFIG3: one-shot SCAN cycle (returns to standby), 32-bit data with
 * channel ID, no digital calibration. */
#define DAQ_ACQ_CONFIG3 0xB0U
/* IRQ: push-pull inactive-high IRQ pin, fast commands on. */
#define DAQ_ACQ_IRQ 0x06U
#define DAQ_ACQ_MUX 0x08U

#define DAQ_ACQ_OSR_BITS(osr) ((osr) == 32U ? 0U : (osr) == 64U ? 1U : (osr) == 128U ? 2U : \
    (osr) == 256U ? 3U : (osr) == 512U ? 4U : (osr) == 1024U ? 5U : (osr) == 2048U ? 6U : 7U)
/* CONFIG1 = PRE[7:6] | OSR[5:2]. PRE 0 is MCLK/1, 2 is MCLK/4. */
#define DAQ_ACQ_CONFIG1(osr, pre_bits) ((uint8_t)(((pre_bits) << 6U) | (DAQ_ACQ_OSR_BITS(osr) << 2U)))

#define DAQ_ACQ_FILL_FAST_OSR 128U
#define DAQ_ACQ_FILL_SLOW_OSR 256U
#define DAQ_ACQ_LAUNCH_FAST_OSR 256U
#define DAQ_ACQ_LAUNCH_SLOW_OSR 64U
/* The die sensor is measured at MCLK/4 (PRE=2): at 16 MHz it reads about
 * -33 C. OSR 64 keeps that conversion inside the FILL slot. */
#define DAQ_ACQ_TEMPERATURE_OSR 64U
#define DAQ_ACQ_TEMPERATURE_PRE_BITS 2U

/* Settled SCAN conversion (datasheet table 5-15) in DMCLK = MCLK/PRE/4
 * periods, doubled by AZ_MUX (both input polarities). Rounded up to us. */
#define DAQ_ACQ_SETTLE_CLOCKS(osr) ((osr) <= 512U ? 3U * (osr) : (osr) + 1024U)
#define DAQ_ACQ_CONV_US(osr, pre_div, channels) \
    (((channels) * 2U * DAQ_ACQ_SETTLE_CLOCKS(osr) * 4U * (pre_div) * 1000000ULL + DAQ_ADC_MCLK_HZ - 1U) / DAQ_ADC_MCLK_HZ)

/* Timing budget: modelled conversion + 3 % + 10 us margin, one 5-byte read
 * (4 us at 10 MHz SPI plus interrupt latency), one 10-byte configuration
 * burst, and a minimum analog settling gap after each reconfiguration. */
#define DAQ_ACQ_MARGIN_US(conv) ((conv) + ((conv) * 3U + 99U) / 100U + 10U)
#define DAQ_ACQ_READ_US 8U
#define DAQ_ACQ_CONFIG_US 12U
#define DAQ_ACQ_SETTLE_US 10U
#define DAQ_ACQ_FAST_CHANNELS (DAQ_CHAMBER_SCAN_MASK != 0U ? 2U : 1U)
#define DAQ_ACQ_FAST_SCAN (0x000001U | DAQ_CHAMBER_SCAN_MASK)

#define DAQ_ACQ_FILL_FAST_US DAQ_ACQ_CONV_US(DAQ_ACQ_FILL_FAST_OSR, 1U, DAQ_ACQ_FAST_CHANNELS)
#define DAQ_ACQ_LAUNCH_FAST_US DAQ_ACQ_CONV_US(DAQ_ACQ_LAUNCH_FAST_OSR, 1U, DAQ_ACQ_FAST_CHANNELS)
#define DAQ_ACQ_FILL_SLOW_US DAQ_ACQ_CONV_US(DAQ_ACQ_FILL_SLOW_OSR, 1U, 1U)
#define DAQ_ACQ_LAUNCH_SLOW_US DAQ_ACQ_CONV_US(DAQ_ACQ_LAUNCH_SLOW_OSR, 1U, 1U)
#define DAQ_ACQ_TEMPERATURE_US DAQ_ACQ_CONV_US(DAQ_ACQ_TEMPERATURE_OSR, 4U, 1U)

/* Slot start, from the frame start: after scan A's margin, its last read,
 * the slot configuration and the settling gap. */
#define DAQ_ACQ_SLOT_OFFSET_US(fast_us) \
    (DAQ_ACQ_MARGIN_US(fast_us) + DAQ_ACQ_READ_US + DAQ_ACQ_CONFIG_US + DAQ_ACQ_SETTLE_US)
#define DAQ_ACQ_FILL_SLOT_US DAQ_ACQ_SLOT_OFFSET_US(DAQ_ACQ_FILL_FAST_US)
#define DAQ_ACQ_LAUNCH_SLOT_US DAQ_ACQ_SLOT_OFFSET_US(DAQ_ACQ_LAUNCH_FAST_US)
/* Latest end of a slot item, including its read, scan A's configuration
 * and the settling gap before the next frame. */
#define DAQ_ACQ_SLOT_END_US(offset, item_us) \
    ((offset) + DAQ_ACQ_MARGIN_US(item_us) + DAQ_ACQ_READ_US + DAQ_ACQ_CONFIG_US + DAQ_ACQ_SETTLE_US)

#define DAQ_ACQ_FILL_SLOW_MAX_US \
    (DAQ_ACQ_FILL_SLOW_US > DAQ_ACQ_TEMPERATURE_US ? DAQ_ACQ_FILL_SLOW_US : DAQ_ACQ_TEMPERATURE_US)
_Static_assert(DAQ_ACQ_SLOT_END_US(DAQ_ACQ_FILL_SLOT_US, DAQ_ACQ_FILL_SLOW_MAX_US) <= DAQ_ACQ_FRAME_US,
               "FILL frame does not fit: fast pair plus slow item exceed the frame period");
_Static_assert(DAQ_ACQ_SLOT_END_US(DAQ_ACQ_LAUNCH_SLOT_US, DAQ_ACQ_LAUNCH_SLOW_US) <= DAQ_ACQ_FRAME_US,
               "LAUNCH frame does not fit: fast pair plus slow item exceed the frame period");
_Static_assert(1000000U % DAQ_ACQ_FRAME_US == 0U, "frame period must divide one second");

typedef struct {
  uint8_t config1, config2;
  uint8_t channels;   /* conversions in this scan */
  uint8_t item;       /* channel ID of a slot item, DAQ_ACQ_ITEM_NONE for scan A */
  uint16_t conv_us;   /* modelled duration of the whole scan */
  uint16_t offset_us; /* START time from the frame start */
  uint32_t scan;      /* SCAN register value */
} daq_acq_scan_t;

static inline daq_acq_scan_t daq_acq_fast_scan(uint8_t profile)
{
  const uint8_t launch = profile == DAQ_ACQ_PROFILE_LAUNCH;
  return (daq_acq_scan_t){
    .config1 = launch ? DAQ_ACQ_CONFIG1(DAQ_ACQ_LAUNCH_FAST_OSR, 0U) : DAQ_ACQ_CONFIG1(DAQ_ACQ_FILL_FAST_OSR, 0U),
    .config2 = DAQ_ACQ_CONFIG2_GAIN1, .channels = DAQ_ACQ_FAST_CHANNELS,
    .item = DAQ_ACQ_ITEM_NONE,
    .conv_us = (uint16_t)(launch ? DAQ_ACQ_LAUNCH_FAST_US : DAQ_ACQ_FILL_FAST_US),
    .offset_us = 0U, .scan = DAQ_ACQ_FAST_SCAN};
}

/* Auxiliary rotation: DAQ_ACQ_AUX_ROTATION slots, one every 10 ms, so the
 * whole rotation repeats every 100 ms. It holds every MCP3564R channel that is
 * not a fast channel and not the selected KG50 input. P6's legs CH2 and CH3
 * open each half of the rotation, back to back, so the leg-balance check gets
 * a CH2/CH3 pair 10 ms apart every 50 ms; the other channels fill the
 * remaining slots in turn (each at least every 100 ms). */
#define DAQ_ACQ_AUX_ROTATION 10U
static inline unsigned daq_acq_aux_items(uint8_t profile, uint8_t kg50_input, uint8_t items[DAQ_ACQ_AUX_ROTATION])
{
  (void)profile;
  uint8_t others[8];
  unsigned count = 0U;
  for (unsigned channel = 1U; channel <= 7U; ++channel) {
    if (daq_chamber_is_channel(channel) || channel == 2U || channel == 3U) continue;
    if (channel == 1U && kg50_input == 0U) continue;
    others[count++] = (uint8_t)channel;
  }
  unsigned next = 0U;
  for (unsigned i = 0U; i < DAQ_ACQ_AUX_ROTATION; ++i) {
    const unsigned half = i % (DAQ_ACQ_AUX_ROTATION / 2U);
    items[i] = half == 0U ? 2U : half == 1U ? 3U : others[next++ % count];
  }
  return DAQ_ACQ_AUX_ROTATION;
}

/* The slot item of frame `frame`. A pure function of the frame number, so the
 * rotation is deterministic and every item recurs at a fixed period. */
static inline uint8_t daq_acq_slot_item(uint8_t profile, uint8_t kg50_input, uint32_t frame)
{
  const uint32_t position = frame % DAQ_ACQ_AUX_EVERY;
  /* The die temperature: its MCLK/4 conversion does not fit the LAUNCH slot. */
  if (position == DAQ_ACQ_TEMPERATURE_FRAME && profile != DAQ_ACQ_PROFILE_LAUNCH)
    return DAQ_ACQ_ITEM_TEMPERATURE;
  if (position != DAQ_ACQ_AUX_EVERY - 1U)
    return kg50_input == 1U ? DAQ_ACQ_ITEM_P6 : 1U;
  uint8_t items[DAQ_ACQ_AUX_ROTATION];
  const unsigned count = daq_acq_aux_items(profile, kg50_input, items);
  return items[(frame / DAQ_ACQ_AUX_EVERY) % count];
}

static inline daq_acq_scan_t daq_acq_slot_scan(uint8_t profile, uint8_t item)
{
  const uint8_t launch = profile == DAQ_ACQ_PROFILE_LAUNCH;
  const unsigned osr = launch ? DAQ_ACQ_LAUNCH_SLOW_OSR : DAQ_ACQ_FILL_SLOW_OSR;
  daq_acq_scan_t scan = {
    .config1 = DAQ_ACQ_CONFIG1(osr, 0U), .config2 = DAQ_ACQ_CONFIG2_GAIN1, .channels = 1U,
    .item = item, .conv_us = (uint16_t)(launch ? DAQ_ACQ_LAUNCH_SLOW_US : DAQ_ACQ_FILL_SLOW_US),
    .offset_us = (uint16_t)(launch ? DAQ_ACQ_LAUNCH_SLOT_US : DAQ_ACQ_FILL_SLOT_US),
    .scan = item < 8U ? 1UL << item : 0U};
  if (item == DAQ_ACQ_ITEM_P6) {
    scan.config2 = DAQ_ACQ_CONFIG2_GAIN16;
    scan.scan = 1UL << 9U;
  } else if (item == DAQ_ACQ_ITEM_TEMPERATURE) {
    scan.config1 = DAQ_ACQ_CONFIG1(DAQ_ACQ_TEMPERATURE_OSR, DAQ_ACQ_TEMPERATURE_PRE_BITS);
    scan.conv_us = (uint16_t)DAQ_ACQ_TEMPERATURE_US;
    scan.scan = 1UL << 12U;
  }
  return scan;
}

/* Midpoint of the conversion that ended (DRDY) at `end_us`: the instant the
 * sample represents. Each channel of a scan takes conv_us / channels. */
static inline uint32_t daq_acq_sample_time_us(const daq_acq_scan_t *scan, uint32_t end_us)
{
  return end_us - (uint32_t)(scan->conv_us / scan->channels) / 2U;
}

/* Whether a slot item may still start at `now` (us since frame start) and
 * end before the next frame. */
static inline int daq_acq_slot_fits(const daq_acq_scan_t *slot, uint32_t start_us)
{
  return DAQ_ACQ_SLOT_END_US(start_us, (uint32_t)slot->conv_us) <= DAQ_ACQ_FRAME_US;
}

/* 10-byte incremental write from CONFIG0 (address 1) through SCAN (address 7):
 * standby first, then the scan's configuration. */
#define DAQ_ACQ_CONFIG_BURST_BYTES 10U
static inline void daq_acq_config_burst(const daq_acq_scan_t *scan, uint8_t out[DAQ_ACQ_CONFIG_BURST_BYTES])
{
  out[0] = 0x42U | (1U << 2U); /* device address 01, incremental write at CONFIG0 */
  out[1] = DAQ_ACQ_CONFIG0_STANDBY;
  out[2] = scan->config1;
  out[3] = scan->config2;
  out[4] = DAQ_ACQ_CONFIG3;
  out[5] = DAQ_ACQ_IRQ;
  out[6] = DAQ_ACQ_MUX;
  out[7] = (uint8_t)(scan->scan >> 16U);
  out[8] = (uint8_t)(scan->scan >> 8U);
  out[9] = (uint8_t)scan->scan;
}
#endif
