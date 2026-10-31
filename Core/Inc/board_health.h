#ifndef BOARD_HEALTH_H
#define BOARD_HEALTH_H
#include "sedsnet_config.h"
/* Low-priority, unreliable 96-byte report every five seconds; DAQ adds a
 * separate32-byte SD status report so existing v1 schemas remain compatible. No polling API
 * or heap walk; bounded refusal accounting, no retry storm. */
void board_health_poll(SedsRouter *router);
#endif
