#pragma once
#include "sedsnet_config.h"
SedsResult daq_log_clock_init(SedsRouter *router);
SedsResult daq_log_clock_poll(SedsRouter *router);
/* Last accepted receiver revision, visible only after storage considered it.
 * Compare with the actual storage session/deadline; receipt alone is not apply. */
uint64_t daq_log_clock_revision(void);
