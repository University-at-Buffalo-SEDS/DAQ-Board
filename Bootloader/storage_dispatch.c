#include "launchcore/storage.h"
static const launchcore_storage_driver_t *active_driver;
void launchcore_storage_set_driver(const launchcore_storage_driver_t *driver) { active_driver = driver; }
const launchcore_storage_driver_t *launchcore_storage_driver(void) { return active_driver; }
launchcore_storage_status_t launchcore_storage_init(void) { return active_driver ? active_driver->init() : LAUNCHCORE_STORAGE_ERR_INIT; }
uint32_t launchcore_slot_base(launchcore_slot_id_t slot) { const launchcore_storage_layout_t *l = active_driver->layout(); return slot == LAUNCHCORE_SLOT_A ? l->slot_a_base : l->slot_b_base; }
uint32_t launchcore_slot_size(launchcore_slot_id_t slot) { const launchcore_storage_layout_t *l = active_driver->layout(); return slot == LAUNCHCORE_SLOT_A ? l->slot_a_size : l->slot_b_size; }
bool launchcore_storage_range_preserves_persistent(uint32_t addr, uint32_t len) { const launchcore_storage_layout_t *l = active_driver->layout(); if (!l->persistent_data_size || !len) return true; uint64_t end=(uint64_t)addr+len, pe=(uint64_t)l->persistent_data_base+l->persistent_data_size; return end <= l->persistent_data_base || pe <= addr; }
