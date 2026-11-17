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
 * unconnected CH7 read 0.013 V; 0.05 V is 3x that. Caveat: a 0-5 V PX309 has
 * a zero offset of +/-2% FS (+/-0.1 V), so a vented, healthy one may also read
 * near 0 V. The operator text says so; a 1-5 V or 0.5-4.5 V transducer is
 * unambiguous. */
#define DAQ_FAULT_CHAMBER_FLOOR_V 0.05f
#define DAQ_FAULT_CHAMBER_NO_SIGNAL_V \
  (DAQ_CHAMBER_LIVE_ZERO_V * 0.5f > DAQ_FAULT_CHAMBER_FLOOR_V ? DAQ_CHAMBER_LIVE_ZERO_V * 0.5f : DAQ_FAULT_CHAMBER_FLOOR_V)
#define DAQ_FAULT_CHAMBER_CLEAR_V (DAQ_FAULT_CHAMBER_NO_SIGNAL_V * 2.0f)
/* A steady condition: 2 s below the level activates it, so a valve transient
 * cannot. Clear after 1 s with a signal. */
#define DAQ_FAULT_CHAMBER_SET_MS 2000U
#define DAQ_FAULT_CHAMBER_CLEAR_MS 1000U

/* --- Reporting (DAQ_INPUT_HEALTH) ---------------------------------------- */
/* Periodic report; a new fault episode is reported early, no sooner than the
 * minimum spacing after the previous report. */
#define DAQ_INPUT_HEALTH_PERIOD_MS 1000U
#define DAQ_INPUT_HEALTH_MIN_SPACING_MS 200U

#endif
