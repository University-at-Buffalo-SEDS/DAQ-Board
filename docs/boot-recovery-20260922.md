# ST-Link boot recovery, 2026-09-22

The attached DAQ was stuck in `NMI_Handler`. `FLASH_ECCR=0x802fa000`
reported an uncorrectable flash ECC error in bank 2 at `0x081fa000`, the
second LaunchCore metadata page. The interrupted read was in the bootloader's
`memcpy`/storage reader. Metadata page 0 at `0x081f8000` passed its CRC;
page 1 did not. The installed application matched the pre-repair release binary.

Both metadata pages and the persistent settings region were backed up under
`build/boot-recovery-20260922/` before recovery. Erasing only the damaged
redundant page allowed the application to run again.

Live debugging then exposed a repeatable flash-driver defect: programming
succeeded, but CPU readback verification returned `LAUNCHCORE_STORAGE_ERR_VERIFY`.
Consequently boot confirmation retried and continuously rewrote metadata.
Invalidating ICACHE immediately before the verification changed the metadata
commit result from storage error to success on the physical board.

`Bootloader/storage_internal_flash.c` now invalidates ICACHE after erases and
programming, including partial failures, before verification or subsequent
reads. The bootloader links the ICACHE HAL as well as the application. This
matches ST's description of cache invalidation after main-memory modification:
[STM32U5 ICACHE training](https://www.st.com/content/ccc/resource/training/technical/product_training/group1/9c/5e/73/79/df/f0/49/65/STM32U5-System-Instruction-cache_ICACHE/files/STM32U5-System-Instruction-cache_ICACHE.pdf/_jcr_content/translations/en.STM32U5-System-Instruction-cache_ICACHE.pdf).

The repaired bootloader, application image, and matching metadata were flashed
and verified through ST-Link. The persistent-settings pages were excluded from
the flash/erase operations. The physical board subsequently had two valid
metadata records with stable sequences 1 and 2, advancing load-cell publish
counters, and zero ECC flags. The release build, 95 Python tests, and 3
GoogleTest cases passed. The new flash-cache regression covers stale reads,
partial flash failures, and cache-maintenance failure.

A second software-reset boot advanced the metadata sequence exactly once to
3 and then remained stable over three observations. Both channel publish
counters continued advancing. All previously written persistent-setting bytes
were preserved; the running application appended a calibration record to unused
journal space. The post-reset observations are saved in
`build/boot-recovery-20260922/live-check.jsonl`.

This recovery does not establish why the original metadata page acquired an
ECC error, and it does not implement recovery from arbitrary future torn flash
writes. Separately, the live SD logger remained at initialization stage 3 with
`g_sd_ready=0` and zero written records; SD recording is not qualified by this
boot repair.

Follow-up: the SD worker was provisioning the new/empty 64 GB card. Once that
completed, a separate scheduling starvation problem was found and corrected.
See [SD recording sessions](sd-log-sessions.md) for the measured write rates and
validation limits after that fix.
