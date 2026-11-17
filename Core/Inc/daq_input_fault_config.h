#ifndef DAQ_INPUT_FAULT_CONFIG_H
#define DAQ_INPUT_FAULT_CONFIG_H
/* Live input and sensor fault thresholds (daq_input_faults.h). Compile-time
 * only: rebuild to change. Every limit is in volts at the point named, and
 * every fault is a warning for the operator; none changes acquisition.
 *
 * Each fault has a set level and a clear level (level hysteresis), a set
 * time the input must stay past the set level before the fault is active,
 * and a clear time it must stay back inside the clear level before the fault
 * clears (time hysteresis). Every entry past the set level counts as one
 * episode, even when it is too short to activate the fault, so a dip between
 * two reports is still counted and latched.
 *
 * Basis: the 2026-11-16 test-fire recording (GS recording
 * 2026-11-17_00-59-46_065), unless stated otherwise. */

/* --- DAQ supply (VMAIN, VMON divider R57 100k / R69 10k, 10 Hz) --------- */
/* Healthy VMAIN read 14.0-14.5 V (4327 samples) with the fill-box battery at
 * 14.1 V. In the fault VMAIN dropped to 11.0-11.2 V (164 samples, 53 dips
 * below 12.5 V) and every load-cell and chamber reading shifted. 12.5 V sits
 * 1.5 V below the healthy band and 1.3 V above the fault level. */
#ifndef DAQ_FAULT_SUPPLY_LOW_V
#define DAQ_FAULT_SUPPLY_LOW_V 12.5f
#endif
/* A dip ends once VMAIN is back above 13.0 V (0.5 V hysteresis, about 50x
 * the 10 mV sample noise), so noise at the threshold cannot count twice. */
#ifndef DAQ_FAULT_SUPPLY_RECOVERED_V
#define DAQ_FAULT_SUPPLY_RECOVERED_V 13.0f
#endif
/* Two consecutive 100 ms samples activate the fault; a single low sample is
 * still counted as a dip. */
#define DAQ_FAULT_SUPPLY_SET_MS 100U
#define DAQ_FAULT_SUPPLY_CLEAR_MS 2000U

/* --- Load-cell rails (ADC-pin volts; mirror GS loadcell_rail_limits) ----- */
/* MCP3564R full scale at unity gain is +/-2^23 codes = +/-2.4 V (internal
 * reference). KG1000 clipped at code 0xB40000 (3.375 V nominal, beyond it). */
#define DAQ_FAULT_ADC_FULL_SCALE_V 2.4f
/* Clear 0.1 V inside full scale: KG1000 zero is 0.209 V and the largest
 * thrust spike was about 1.03 V, far below either level. */
#define DAQ_FAULT_ADC_FULL_SCALE_CLEAR_V 2.3f
/* KG1000 amplifier floor: legacy raw 0.0025 * 16000/921 = 0.0434 V (GS
 * KG1000_LOW_RAIL_V). The stuck amplifier read 0.021-0.036 V (legacy raw
 * 0.0012-0.0021), zero load reads 0.209 V. */
#define DAQ_FAULT_KG1000_LOW_V ((float)(0.0025 * (16000.0 / 921.0)))
/* Clear above 0.06 V: 0.017 V of hysteresis, still -95 kg below zero load. */
#define DAQ_FAULT_KG1000_LOW_CLEAR_V 0.06f
/* P6 differential at PGA x16: full scale 2.4 V / 16 = 0.15 V (GS
 * DAQ_P6_FULL_SCALE_V). The healthy KG50 range is about 0.0037-0.0043 V. */
#define DAQ_FAULT_P6_FULL_SCALE_V (DAQ_FAULT_ADC_FULL_SCALE_V / 16.0f)
#define DAQ_FAULT_P6_FULL_SCALE_CLEAR_V 0.14f
/* Each 2 ms acquisition window is one observation. 20 ms of clipping
 * activates the fault; the KG1000 rail runs in the test fire lasted
 * 0.1-9 s. Clear after 500 ms back in range. */
#define DAQ_FAULT_RAIL_SET_MS 20U
#define DAQ_FAULT_RAIL_CLEAR_MS 500U

/* --- P6 differential KG50 leg balance (CH2 = P6/3 S+, CH3 = S-) --------- */
/* Single-ended leg volts from the 10 Hz auxiliary scan. A bridge's two sense
 * legs differ only by its output: |CH2 - CH3| was 4.3 mV median and 4.8 mV
 * p99. When the S+ lead dropped out, CH2 fell 1.139 -> 0.91 V while CH3 held
 * 1.135 V (|CH2 - CH3| = 0.23 V) and KG50 spiked to -383 kg. 50 mV is 10x the
 * normal difference and 1/4.6 of the dropout. Checked only while P6 is the
 * selected KG50 input. */
#define DAQ_FAULT_P6_IMBALANCE_V 0.05f
#define DAQ_FAULT_P6_IMBALANCE_CLEAR_V 0.02f
#define DAQ_FAULT_P6_IMBALANCE_SET_MS 100U
#define DAQ_FAULT_P6_IMBALANCE_CLEAR_MS 1000U
/* P6 bridge excitation. No excitation rail is measured on this board: +12VA
 * has only an enable readback, and the bridges run from the ADC's 2.4 V
 * reference. The P6 legs' mean, about half the bridge excitation, is the
 * only proxy: 1.137 V in the healthy state, 1.163 V during the supply dips.
 * Outside 0.6-1.8 V both legs have moved together, so the excitation or both
 * leads are lost. */
#define DAQ_FAULT_P6_COMMON_LOW_V 0.6f
#define DAQ_FAULT_P6_COMMON_HIGH_V 1.8f
#define DAQ_FAULT_P6_COMMON_HYSTERESIS_V 0.1f
#define DAQ_FAULT_P6_COMMON_SET_MS 200U
#define DAQ_FAULT_P6_COMMON_CLEAR_MS 1000U

/* --- Chamber pressure transducer (connector volts at P4, after the 11:1
 * divider is undone) ---------------------------------------------------- */
/* Live zero of the installed transducer output: 0 V for a 0-5 V output
 * (OMEGA PX309-1KG5V, installed), 1.0 V for a 1-5 V output, 0.5 V for a
 * 0.5-4.5 V output. */
#ifndef DAQ_CHAMBER_LIVE_ZERO_V
#define DAQ_CHAMBER_LIVE_ZERO_V 0.0f
#endif
/* No signal: below half the live zero, and never below 0.05 V. A dead input
 * read 0.006-0.017 V all session, burn included (CH6, P4/3), and the
 * unconnected CH7 read 0.013 V; 0.05 V is 3x that. */
#define DAQ_FAULT_CHAMBER_FLOOR_V 0.05f
#define DAQ_FAULT_CHAMBER_NO_SIGNAL_V \
  (DAQ_CHAMBER_LIVE_ZERO_V * 0.5f > DAQ_FAULT_CHAMBER_FLOOR_V ? DAQ_CHAMBER_LIVE_ZERO_V * 0.5f : DAQ_FAULT_CHAMBER_FLOOR_V)
#define DAQ_FAULT_CHAMBER_CLEAR_V (DAQ_FAULT_CHAMBER_NO_SIGNAL_V * 2.0f)
/* A steady condition: 2 s below the level activates it, so a valve transient
 * cannot. Clear after 1 s with a signal. */
#define DAQ_FAULT_CHAMBER_SET_MS 2000U
#define DAQ_FAULT_CHAMBER_CLEAR_MS 1000U
/* When the check runs. A transducer with a live zero above the floor (1-5 V,
 * 0.5-4.5 V) reads it whenever powered, so a missing signal is a fault at any
 * time and the check always runs. A 0-5 V transducer (the installed PX309,
 * same type as the oxidizer tank PT) reads about 0 V at 0 psig, within its
 * +/-2% FS (+/-0.1 V) zero offset: vented, healthy and dead look alike. Its
 * check therefore runs only while the chamber must be pressurized, in the
 * launch burn window T0 + BURN_START to T0 + BURN_END, with T0 from
 * DAQ_LOG_CLOCK (deadline - 20 s, sd_card_launch_since_t0_ms). Outside it
 * (at rest, during fill, with no launch scheduled, or without the SD launch
 * clock) the chamber fault is inactive; its reading is still reported. */
#define DAQ_FAULT_CHAMBER_ALWAYS_ON (DAQ_CHAMBER_LIVE_ZERO_V * 0.5f > DAQ_FAULT_CHAMBER_FLOOR_V)
/* Start: past ignition delay and chamber fill. In the 2026-11-16 fire T0 was
 * 9.3 s and thrust began at 10.3 s (T0 + 1.0 s); 1.5 s leaves 0.5 s margin,
 * so a healthy transducer has a signal before the check starts and the rise
 * is never counted as an episode. */
#ifndef DAQ_FAULT_CHAMBER_BURN_START_MS
#define DAQ_FAULT_CHAMBER_BURN_START_MS 1500U
#endif
/* End: a bound on the burn. The 2026-11-16 burn ended at T0 + 5.4 s; 5 s stays
 * inside it, so tail-off (a real return to 0 psig) is never checked. With the
 * 2 s set time a dead input activates by T0 + 3.5 s. */
#ifndef DAQ_FAULT_CHAMBER_BURN_END_MS
#define DAQ_FAULT_CHAMBER_BURN_END_MS 5000U
#endif
_Static_assert(DAQ_FAULT_CHAMBER_BURN_END_MS - DAQ_FAULT_CHAMBER_BURN_START_MS > DAQ_FAULT_CHAMBER_SET_MS &&
               DAQ_FAULT_CHAMBER_BURN_END_MS > DAQ_FAULT_CHAMBER_BURN_START_MS,
               "the chamber burn window must be longer than the chamber set time");

/* --- Reporting (DAQ_INPUT_HEALTH) ---------------------------------------- */
/* Periodic report; a new fault episode is reported early, no sooner than the
 * minimum spacing after the previous report. */
#define DAQ_INPUT_HEALTH_PERIOD_MS 1000U
#define DAQ_INPUT_HEALTH_MIN_SPACING_MS 200U

#endif
