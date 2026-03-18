#pragma once
#include "tx_api.h"

/* ------ Telemetry Thread ------ */
extern TX_THREAD telemetry_thread;
extern TX_THREAD daq_thread;
extern TX_THREAD sd_writer_thread;

void telemetry_thread_entry(ULONG initial_input);
UINT create_telemetry_thread(TX_BYTE_POOL *byte_pool);
void daq_thread_entry(ULONG initial_input);
UINT create_daq_thread(void);
void sd_writer_thread_entry(ULONG initial_input);
UINT create_sd_writer_thread(void);
/* ------ Telemetry Thread ------ */
