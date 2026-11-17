from pathlib import Path
import subprocess
import tempfile
import unittest

from checked_router_fixture import checked_hooks
from router_lifecycle_fixture import function as actual_function
ROOT = Path(__file__).resolve().parents[1]


def function(source, name):
    start = source.index(name)
    brace = source.index("{", start)
    depth, end = 1, brace + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def run_c(code):
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory)
        (path / "test.c").write_text(code)
        subprocess.run(["cc", "-DTELEMETRY_ENABLED=1", "-std=c11", "-Wall", "-Wextra", "-Werror",
                        "-Wno-unused-variable", "-I", str(ROOT / "Core/Inc"),
                        str(path / "test.c"), "-o", str(path / "test")], check=True)
        subprocess.run([str(path / "test")], check=True)


class ReportTests(unittest.TestCase):
    def test_live_values_have_no_sd_report_queue(self):
        # The former report queue/thread existed only to write SD copies of the
        # live KG1000/KG50 values; raw records are now the single SD copy.
        source = (ROOT / "Core/Src/daq_thread.c").read_text()
        for gone in ("g_report_queue", "g_report_thread", "daq_enqueue_report(",
                     "daq_report_thread_entry", "sd_card_enqueue_csv_row", "g_daq_report_drop_count"):
            self.assertNotIn(gone, source)
        entry = function(source, "void daq_thread_entry(")
        self.assertIn("daq_offer_live_report(&report);", entry)

    def test_packet_time_subtracts_queue_age_and_handles_no_sync(self):
        source = (ROOT / "Core/Src/telemetry.c").read_text()
        run_c(r'''
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#define TELEMETRY_ENABLED 1
#define SEDS_OK 0
#define SEDS_BAD_ARG 1
#define SEDS_ERR 2
typedef int SedsResult;typedef unsigned SedsDataType;
static struct {void*r;uint64_t start_time;} g_router={(void*)1,100};
static unsigned locked;
void telemetry_lock(void){assert(!locked);++locked;}
void telemetry_unlock(void){assert(locked==1);--locked;}
static int synced=1;static uint64_t emitted;
static uint64_t telemetry_now_ms(void){return 1000;}
static SedsResult init_telemetry_router(void){return SEDS_OK;}
static int guess_kind_from_elem_size(size_t n){return (int)n;}
static int seds_router_get_network_time_ms(void*r,uint64_t*t){(void)r;*t=1000000;return synced?SEDS_OK:SEDS_ERR;}
static int seds_router_log_typed_ex(void*r,unsigned ty,const void*d,size_t n,size_t w,int k,const uint64_t*t,int queued){(void)r;(void)ty;(void)d;(void)n;(void)w;(void)k;assert(queued);emitted=*t;return SEDS_OK;}
''' +checked_hooks(ROOT)+'\n'+'\n'.join(actual_function(source,n)for n in ('log_telemetry_captured_locked','log_telemetry_captured'))+ r'''
int main(void){float value=1;
 assert(log_telemetry_captured(118,&value,1,4,950)==SEDS_OK && emitted==999950);
 synced=0;assert(log_telemetry_captured(118,&value,1,4,950)==SEDS_OK && emitted==850);
 assert(log_telemetry_captured(118,&value,1,4,0)==SEDS_OK && emitted==0);
 assert(log_telemetry_captured(118,0,1,4,950)==SEDS_BAD_ARG);
}
''')


if __name__ == "__main__":
    unittest.main()
