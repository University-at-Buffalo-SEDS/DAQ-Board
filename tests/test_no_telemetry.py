"""SDK-free optional-feature contract; full embedded smoke is explicitly opt-in."""
import dataclasses
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]


def cached_source(name, env):
    explicit = os.environ.get(env)
    if explicit and (Path(explicit) / "CMakeLists.txt").is_file():
        return Path(explicit)
    return next((p for p in ROOT.glob(f"build/*/_deps/{name}-src")
                 if (p / "CMakeLists.txt").is_file()), None)


def overlay_source(destination):
    for entry in ROOT.iterdir():
        if entry.name.startswith('.') or entry.name in {'build', 'target', 'dist', 'cmake'}:
            continue
        (destination / entry.name).symlink_to(entry, target_is_directory=entry.is_dir())
    # Real directories keep ../../Core references inside the selected source.
    cmake = destination / 'cmake'
    cmake.mkdir()
    for entry in (ROOT / 'cmake').iterdir():
        if entry.name == 'stm32cubemx':
            child = cmake / entry.name
            child.mkdir()
            for item in entry.iterdir():
                (child / item.name).symlink_to(item, target_is_directory=item.is_dir())
        elif entry.name in {'sedsnet_fetch.cmake', 'sedsnet_source.cmake'}:
            (cmake / entry.name).write_text('message(FATAL_ERROR "SDK_FETCH_MUST_NOT_RUN")\n')
        else:
            (cmake / entry.name).symlink_to(entry, target_is_directory=entry.is_dir())


class NoTelemetryTests(unittest.TestCase):
    def test_cli_keeps_no_telemetry_in_cmake(self):
        spec = importlib.util.spec_from_file_location('sdk_free_build_cli', ROOT / 'build.py')
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        args = module.make_parser().parse_args(['build', '--release', '--no-telemetry'])
        cfg = module.build_cfg_from_args(mock.Mock(), args)
        self.assertFalse(cfg.telemetry)
        with tempfile.TemporaryDirectory() as directory:
            cfg = dataclasses.replace(cfg, repo_root=Path(directory), project_name='TestBoard')
            commands = []
            with mock.patch.object(module, 'run', side_effect=lambda ui, cmd, **kw: commands.append(cmd)), \
                 mock.patch.object(module, 'pick_elf', return_value=cfg.build_dir/'TestBoard.elf'), \
                 mock.patch.object(module, 'assert_embedded_cache', create=True):
                module.configure_and_build(mock.Mock(), cfg, 'factory-image')
            self.assertIn('-DENABLE_TELEMETRY=OFF', commands[0])
            self.assertIn('factory-image', commands[1])

    def test_local_headers_and_clock_do_not_include_sdk(self):
        compiler = shutil.which('cc')
        if not compiler:
            self.skipTest('Native C compiler unavailable')
        with tempfile.TemporaryDirectory() as directory:
            work = Path(directory)
            (work / 'sedsnet.h').write_text('#error SDK_HEADER_MUST_NOT_BE_INCLUDED\n')
            headers = ['sedsnet_config.h', 'telemetry.h', 'flight_state_cache.h']
            for optional in ['board_commands.h', 'status_report_retry.h', 'daq_calibration.h', 'daq_log_time.h']:
                if (ROOT / 'Core/Inc' / optional).exists():
                    headers.append(optional)
            body = '#include <assert.h>\n#include <stdint.h>\n'
            body += ''.join(f'#include "{header}"\n' for header in headers)
            body += '\nuint64_t board_uptime_ticks(void) { return 1234U; }\nint main(void) {\n'
            body += '_Static_assert(sizeof(SEDS_DT_KG1000) == sizeof(uint32_t), "wire IDs are scalar");\n'
            if 'status_report_retry.h' in headers:
                body += 'publish_umbilical_status(0U, 0U); retry_pending_status_reports();\n'
            if 'daq_log_time.h' in headers:
                body += 'assert(daq_log_monotonic_ms()==1234U); assert(daq_log_unix_ms()==0U); assert(daq_log_unix_ms_cached()==0U);\n'
            body += 'return 0; }\n'
            (work / 'local.c').write_text(body)
            built = subprocess.run([compiler, '-std=c11', '-Wall', '-Werror', '-I', str(work),
                                    '-I', str(ROOT / 'Core/Inc'), str(work / 'local.c'),
                                    '-o', str(work / 'local')], capture_output=True, text=True)
            self.assertEqual(built.returncode, 0, built.stderr)
            subprocess.run([str(work / 'local')], check=True)

    def test_local_flash_restore_does_not_need_sdk_or_write_network_state(self):
        if not shutil.which('cc'):
            self.skipTest('Native C compiler unavailable')
        with tempfile.TemporaryDirectory() as directory:
            work = Path(directory)
            (work/'launchcore').mkdir()
            (work/'sedsnet.h').write_text('#error SDK_HEADER_MUST_NOT_BE_INCLUDED\n')
            (work/'launchcore/persist.h').write_text('typedef enum {LAUNCHCORE_PERSIST_OK=0, LAUNCHCORE_PERSIST_NOT_FOUND=1, LAUNCHCORE_PERSIST_STORAGE_ERROR=2} launchcore_persist_status_t;\n')
            (work/'main.h').write_text('#include <stdint.h>\n')
            (work/'restore.c').write_text(r'''#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include "flight_state_cache.h"
#include "persistent_store.h"
static unsigned inits, reads;
launchcore_persist_status_t persistent_store_init(void) { ++inits; return LAUNCHCORE_PERSIST_OK; }
launchcore_persist_status_t persistent_store_get(uint32_t key, void *value, size_t *size) { assert(key==0x46535445u && *size==1); ++reads; *(uint8_t*)value=3; return LAUNCHCORE_PERSIST_OK; }
launchcore_persist_status_t persistent_store_set(uint32_t key,const void *value,size_t size) { (void)key;(void)value;(void)size;assert(0);return LAUNCHCORE_PERSIST_STORAGE_ERROR; }
int main(void) { flight_state_cache_restore();flight_state_cache_restore();assert(inits==1&&reads==1);assert(flight_state_cache_has_value());assert(flight_state_cache_value()==3);return 0; }
''')
            command = ['cc','-std=c11','-Wall','-Wextra','-Werror','-Wno-unknown-attributes',
                       '-I'+str(work),'-I'+str(ROOT/'Core/Inc'),str(ROOT/'Core/Src/flight_state_cache.c'),
                       str(work/'restore.c'),'-o',str(work/'restore')]
            built = subprocess.run(command,capture_output=True,text=True)
            self.assertEqual(built.returncode,0,built.stderr)
            subprocess.run([str(work/'restore')],check=True)

    def test_watchdog_starts_and_requires_local_progress_without_telemetry(self):
        compiler = shutil.which('cc')
        if not compiler:
            self.skipTest('Native C compiler unavailable')
        source = (ROOT/'Core/Src/board_watchdog.c').read_text()
        source = '\n'.join(line for line in source.splitlines() if not line.startswith('#include'))
        stub = r'''#include <stdint.h>
#include <assert.h>
#include "board_watchdog.h"
#include "board_uptime.h"
typedef uint32_t ULONG;
#define BOARD_WATCHDOG_ENABLE 1
#define IWDG_SR_PVU 1U
#define IWDG_SR_RVU 2U
#define IWDG_SR_WVU 4U
#define __HAL_RCC_CLEAR_RESET_FLAGS() ((void)0)
#define TX_TIMER_TICKS_PER_SECOND 1000U
static struct {uint32_t CSR,RSR;} rcc;
#define RCC (&rcc)
static struct {uint32_t KR,PR,RLR,SR;} iwdg;
#define IWDG (&iwdg)
static uint32_t tick,mask;
static ULONG tx_time_get(void){return tick;}
static uint32_t HAL_GetTick(void){return tick;}
static uint32_t __get_PRIMASK(void){return mask;}
static void __disable_irq(void){mask=1;}
static void __set_PRIMASK(uint32_t saved){mask=saved;}
static void __NOP(void){}
'''
        main = r'''int main(void){
const uint32_t required=BOARD_WATCHDOG_REQUIRED_MASK & ~BOARD_WATCHDOG_NETWORK;
assert(required!=0U);board_watchdog_start();assert(g_watchdog_started==1U);
assert(iwdg.PR==6U&&iwdg.RLR==2047U&&g_watchdog_missing_mask==required);
tick=250;board_watchdog_progress(BOARD_WATCHDOG_NETWORK);assert(g_watchdog_feed_count==0U);
for(uint32_t bit=1;bit;bit<<=1)if(required&bit)board_watchdog_progress(bit);
assert(g_watchdog_feed_count==1U&&!mask);
tick=500;board_watchdog_progress(BOARD_WATCHDOG_NETWORK);assert(g_watchdog_feed_count==1U);
const uint32_t first=required & (0U-required);board_watchdog_progress(first);
if(required!=first)assert(g_watchdog_feed_count==1U);
return 0;}
'''
        with tempfile.TemporaryDirectory() as directory:
            exe=Path(directory)/'watchdog'
            built=subprocess.run([compiler,'-std=c11','-Wall','-Wextra','-Werror','-I'+str(ROOT/'Core/Inc'),'-x','c','-','-o',str(exe)],input=stub+source+main,text=True,capture_output=True)
            self.assertEqual(built.returncode,0,built.stderr)
            subprocess.run([str(exe)],check=True)

    def test_fresh_configuration_skips_sdk_fetch_and_cached_headers(self):
        cmake = shutil.which('cmake')
        arm = shutil.which('arm-none-eabi-gcc') or next(Path('/opt/ST').glob('*/GNU-tools-for-STM32/bin/arm-none-eabi-gcc'), None)
        launchcore = cached_source('sedslaunchcore', 'SDK_FREE_LAUNCHCORE_SOURCE')
        if not cmake or not arm or not launchcore:
            self.skipTest('Requires CMake, ARM compiler and cached LaunchCore (or SDK_FREE_LAUNCHCORE_SOURCE)')
        with tempfile.TemporaryDirectory() as directory:
            work = Path(directory)
            source = work / 'source'
            source.mkdir()
            overlay_source(source)
            poison = work / 'poison'
            poison.mkdir()
            (poison / 'sedsnet.h').write_text('#error SDK_HEADER_MUST_NOT_BE_INCLUDED\n')
            command = [cmake, '-S', str(source), '-B', str(work / 'off'), '-G', 'Ninja',
                       '-DCMAKE_BUILD_TYPE=Release', '-DENABLE_TELEMETRY=OFF',
                       '-DTELEMETRY_USE_TLSF=ON', '-DFETCHCONTENT_FULLY_DISCONNECTED=ON',
                       '-DCMAKE_TOOLCHAIN_FILE=' + str(source / 'cmake/gcc-arm-none-eabi.cmake'),
                       '-DFETCHCONTENT_SOURCE_DIR_SEDSLAUNCHCORE=' + str(launchcore),
                       '-DFETCHCONTENT_SOURCE_DIR_SEDSNET=' + str(work / 'no-sdk'),
                       '-DCMAKE_C_FLAGS=-I' + str(poison)]
            if 'FetchContent_MakeAvailable(filex)' in (ROOT / 'CMakeLists.txt').read_text():
                filex = cached_source('filex', 'SDK_FREE_FILEX_SOURCE')
                if not filex:
                    self.skipTest('Requires cached FileX or SDK_FREE_FILEX_SOURCE for local SD')
                command += ['-DFETCHCONTENT_SOURCE_DIR_FILEX=' + str(filex)]
            result = subprocess.run(command, capture_output=True, text=True, timeout=60)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            commands = json.loads((work / 'off/compile_commands.json').read_text())
            for entry in commands:
                self.assertNotIn('C-Headers', entry['command'])
                self.assertNotIn('no-sdk', entry['command'])
                self.assertFalse(entry['file'].endswith('/telemetry.c'), entry['file'])
                self.assertFalse(entry['file'].endswith('/telemetry_thread.c'), entry['file'])
            self.assertFalse((work / 'off/_deps/sedsnet-src').exists())
            # Positive control: our trap really intercepts an SDK-enabled build.
            on = [arg.replace(str(work / 'off'), str(work / 'on')).replace(
                  '-DENABLE_TELEMETRY=OFF', '-DENABLE_TELEMETRY=ON') for arg in command]
            enabled = subprocess.run(on, capture_output=True, text=True, timeout=60)
            self.assertNotEqual(enabled.returncode, 0)
            self.assertIn('SDK_FETCH_MUST_NOT_RUN', enabled.stdout + enabled.stderr)
            if os.environ.get('SDK_FREE_FULL_BUILD') == '1':
                result = subprocess.run([cmake, '--build', str(work / 'off'), '--parallel', '4'],
                                        capture_output=True, text=True, timeout=300)
                self.assertEqual(result.returncode, 0, result.stdout[-6000:] + result.stderr[-6000:])
                self.assertTrue(list((work / 'off').glob('*.factory.bin')))


if __name__ == '__main__':
    unittest.main()
