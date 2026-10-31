"""Actual read-only storage receipt, exact wide words, and bounded reporter schema."""
from pathlib import Path
import json,re,subprocess,tempfile,unittest
from test_sd_raw_queue import function
ROOT=Path(__file__).resolve().parents[1]

class LaunchLogStatusTests(unittest.TestCase):
 def test_actual_snapshot_preserves_pair_flags_and_does_not_advance_deadline(self):
  source=(ROOT/'Core/Src/sd_card.c').read_text()
  body=function(source,'sd_card_launch_log_snapshot')
  self.assertNotIn('sd_launch_finished(',body)
  self.assertNotIn('sd_route_session_locked(',body)
  code=r'''
#include <stdint.h>
#include <assert.h>
#include <stddef.h>
static unsigned mask;
static uint32_t __get_PRIMASK(void){return mask;}
static void __disable_irq(void){mask=1;}
static void __set_PRIMASK(uint32_t m){mask=m;}
static uint64_t revision, g_sd_session,g_sd_close_unix_ms,g_sd_raw_session,g_sd_telemetry_session;
static uint8_t g_sd_services_initialized,g_sd_ready,g_sd_deadline_anchored,g_sd_launch_closed,g_file_open,g_telemetry_file_open,g_sd_launch_flush_pending;
#ifdef TELEMETRY_ENABLED
static uint64_t daq_log_clock_revision(void){assert(!mask);return revision;}
#endif
''' +body+r'''
int main(void){
 uint32_t words[7];sd_card_launch_log_snapshot(NULL);sd_card_launch_log_snapshot(words);
 for(unsigned i=0;i<7;i++)assert(!words[i]);
 g_sd_session=UINT64_C(0x12345678abcdef01);g_sd_close_unix_ms=UINT64_C(0x1111222233334444);
 revision=UINT64_C(0xfeedcafe12345678);
 g_sd_services_initialized=g_sd_ready=g_sd_deadline_anchored=1;
 g_sd_raw_session=g_sd_telemetry_session=g_sd_session;g_file_open=g_telemetry_file_open=1;
 sd_card_launch_log_snapshot(words);
 assert(words[0]==0xabcdef01U&&words[1]==0x12345678U&&words[2]==0x33334444U&&words[3]==0x11112222U&&words[6]==55U);
#ifdef TELEMETRY_ENABLED
 assert(words[4]==0x12345678U&&words[5]==0xfeedcafeU);
#else
 assert(!words[4]&&!words[5]);
#endif
 g_sd_launch_closed=1;g_sd_launch_flush_pending=1;sd_card_launch_log_snapshot(words);
 assert(words[6]==127U); /* closed but buffered launch handles may still drain */
 assert(g_sd_session==UINT64_C(0x12345678abcdef01)&&g_sd_close_unix_ms==UINT64_C(0x1111222233334444));
 ++g_sd_raw_session;g_telemetry_file_open=0;sd_card_launch_log_snapshot(words);assert(words[6]==79U);
 g_sd_session=0;g_sd_close_unix_ms=0;sd_card_launch_log_snapshot(words);
 assert(!words[0]&&!words[1]&&!words[2]&&!words[3]&&!(words[6]&48U));
 assert(!mask);
}
'''
  with tempfile.TemporaryDirectory() as d:
   for flags in [[],['-DTELEMETRY_ENABLED']]:
    exe=Path(d)/'status'
    p=subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror',*flags,'-x','c','-','-o',str(exe)],input=code,text=True,capture_output=True)
    self.assertEqual(p.returncode,0,p.stderr);subprocess.run([str(exe)],check=True)
 def test_schema_header_and_reporter_are_exact_low_priority_companion(self):
  d=json.loads((ROOT/'config/sedsnet.json').read_text());entry=next(t for t in d['types'] if t['name']=='DAQ_LAUNCH_LOG_STATUS')
  self.assertEqual(entry['element'],dict(kind='Static',data_type='UInt32',count=7))
  self.assertEqual((entry['reliable'],entry['priority'],entry['endpoints']),('Off',0,['ActuatorGroundStation']))
  h=0x811c9dc5^0x5459
  for byte in entry['name'].encode():h=((h^byte)*0x01000193)&0xffffffff
  expected=0x10000|(h&0x0fffffff)
  header=(ROOT/'Core/Inc/sedsnet_config.h').read_text()
  self.assertIn(f'#define SEDS_DT_DAQ_LAUNCH_LOG_STATUS ((uint32_t){expected}U)',header)
  producer=(ROOT/'Core/Src/board_health.c').read_text()
  self.assertEqual(producer.count('sd_card_launch_log_snapshot('),1)
  self.assertIn('5000U',producer)
  self.assertLess(producer.index('SEDS_DT_BOARD_HEALTH'),producer.index('SEDS_DT_DAQ_LAUNCH_LOG_STATUS'))
