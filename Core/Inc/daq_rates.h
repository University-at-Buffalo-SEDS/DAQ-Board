#ifndef DAQ_RATES_H
#define DAQ_RATES_H
#include <stdint.h>
#include "daq_chamber_pressure.h"
#include "daq_acq_schedule.h"

/* Board-local settings. No network variables; rebuild/reflash to change.
 * The MCP3564R frame schedule (conversion instants, OSR, gains) lives in
 * daq_acq_schedule.h; this file sizes the software around it. docs/
 * data-pipeline.md has the whole pipeline with diagrams. */

/* Conversions per second reaching the acquisition thread: the fast pair and
 * one slot item per frame, plus the launch-window supply pair (two ADC1
 * records every DAQ_SUPPLY_LAUNCH_PERIOD_MS). Sizes batches and SD reserves;
 * it programs nothing. */
#define DAQ_ACQ_FRAME_HZ (1000000U / DAQ_ACQ_FRAME_US)
#define DAQ_ADC_READ_RATE_HZ (DAQ_ACQ_FRAME_HZ * (DAQ_ACQ_FAST_CHANNELS + 1U) + 2U * 1000U / 4U)

/* Acquisition thread period: drains the conversion queue, builds SD records,
 * runs the report filters. It does not time any conversion. */
#ifndef DAQ_ACQUISITION_PERIOD_MS
#define DAQ_ACQUISITION_PERIOD_MS 2U
#endif

/* Published (network/display) streams. SD keeps every raw conversion; these
 * filters only shape what is published. Each report is the mean of one bin
 * of median-filtered samples, binned on the microsecond sample clock.
 *
 * Fast pair (KG1000, chamber PT): 3-sample median (rejects any single-sample
 * spike, 1 sample = 1 ms delay, a step passes intact), 20 ms bins -> 50 Hz.
 * Selected KG50: 5-sample median (~2.2 ms delay at ~900 Hz), 50 ms bins ->
 * 20 Hz; 50 ms is three 60 Hz periods, so mains pickup averages out.
 * Auxiliary MCP3564R channels: 3-sample median of their ~14 Hz slots,
 * reported with the 10 Hz analog packet. */
#define DAQ_FAST_MEDIAN 3U
#define DAQ_FAST_REPORT_US 20000U
#define DAQ_KG50_MEDIAN 5U
#define DAQ_KG50_REPORT_US 50000U
#define DAQ_AUX_MEDIAN 3U
_Static_assert(DAQ_FAST_REPORT_US % (DAQ_ACQ_FRAME_US * DAQ_ACQ_AUX_EVERY) == 0U &&
               DAQ_KG50_REPORT_US % (DAQ_ACQ_FRAME_US * DAQ_ACQ_AUX_EVERY) == 0U,
               "report bins must hold whole auxiliary rotations (no beat with the schedule)");

/* Board supply (VMAIN/IMON on ADC1) logging period inside the launch window;
 * outside it the supply follows DAQ_ANALOG_REPORT_PERIOD_MS. Two polled
 * conversions (~60 us) per period; 4 ms resolves sub-100 ms supply dips. */
#ifndef DAQ_SUPPLY_LAUNCH_PERIOD_MS
#define DAQ_SUPPLY_LAUNCH_PERIOD_MS (DAQ_ACQUISITION_PERIOD_MS > 4U ? DAQ_ACQUISITION_PERIOD_MS : 4U)
#endif
#if DAQ_SUPPLY_LAUNCH_PERIOD_MS < DAQ_ACQUISITION_PERIOD_MS || DAQ_SUPPLY_LAUNCH_PERIOD_MS > 10U
#error "Launch supply period must be from one acquisition period to 10 ms (100 Hz)"
#endif
/* The MCP3564R LAUNCH profile follows sd_card_launch_window_active(): from
 * the same 0.5 s supply pre-roll before the launch window until it closes. */
/* Temperature must refresh independently of acquisition-loop overruns. */
#define DAQ_TEMPERATURE_REPORT_PERIOD_MS 100U
/* Auxiliary channels and internal ADCs report at 10 Hz. */
#ifndef DAQ_ANALOG_REPORT_PERIOD_MS
#define DAQ_ANALOG_REPORT_PERIOD_MS 100U
#endif
#if DAQ_ANALOG_REPORT_PERIOD_MS < 100U || DAQ_ANALOG_REPORT_PERIOD_MS > 1000U
#error "Auxiliary analog period must be 100..1000 ms"
#endif
/* P6 leg pairing window: CH2 and CH3 are converted 10 ms apart. */
#define DAQ_P6_LEG_PAIR_MS 20U
/* An auxiliary value older than this is reported as NaN (stale). */
#define DAQ_AUX_STALE_MS 300U
/* Acquisition outranks the I/O workers so reporting or SD backlogs cannot
 * consume its service budget. Live I/O workers share a bounded slice; storage
 * runs below them and sleeps for DMA. ADC interrupts remain independent. */
#define DAQ_IO_THREAD_PRIORITY 6U
/* Storage never runs at acquisition/report/network priority. */
#define DAQ_SD_WRITER_THREAD_PRIORITY 7U
#define DAQ_ACQUISITION_THREAD_PRIORITY 5U
#define DAQ_IO_THREAD_SLICE_MS 1U
/* Records per acquisition cycle plus bounded scheduling slack. */
#define DAQ_RAW_BATCH_CAPACITY (((DAQ_ADC_READ_RATE_HZ * DAQ_ACQUISITION_PERIOD_MS + 999U) / 1000U) + 4U)
/* Reserve for intermittent card/filesystem stalls, independent of network
 * cadence. The former 200 ms reserve overflowed during sustained bench runs. */
#ifndef DAQ_SD_RAW_BUFFER_MS
#define DAQ_SD_RAW_BUFFER_MS 800U
#endif
/* Full-batch equivalents used to budget the compact byte pool. The actual
 * pointer queue can hold more, smaller batches within that same RAM budget. */
#define DAQ_SD_RAW_QUEUE_DEPTH ((DAQ_SD_RAW_BUFFER_MS + DAQ_ACQUISITION_PERIOD_MS - 1U) / DAQ_ACQUISITION_PERIOD_MS + 1U)
#if DAQ_SD_RAW_QUEUE_DEPTH < 2 || DAQ_SD_RAW_QUEUE_DEPTH > 512
#error "SD raw reserve must fit between 2 and 512 batches"
#endif
#if DAQ_ACQUISITION_PERIOD_MS < 1 || DAQ_ACQUISITION_PERIOD_MS > 20
#error "DAQ_ACQUISITION_PERIOD_MS must be from 1 to 20 (the conversion queue holds 42 ms)"
#endif
#endif
