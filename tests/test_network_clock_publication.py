"""A wrongly anchored network clock is restarted from the router owner.

Bench evidence (Actuator Board): the SEDSnet clock read about the board's own
uptime while the station's clock was UTC. A relayed consumer never completes a
verified time exchange, so only a timesync restart lets the next announce
anchor it again. The actual telemetry_update_network_health() runs with
modeled SDK calls.
"""
from pathlib import Path
import re,subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
def function(text,signature):
 i=text.index(signature);j=text.index('{',i);depth=1;k=j+1
 while depth:depth+=(text[k]=='{')-(text[k]=='}');k+=1
 return text[i:k]
def compile_and_run(test,code):
 with tempfile.TemporaryDirectory() as d:
  exe=Path(d)/'probe';out=subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-x','c','-','-o',str(exe)],input=code,text=True,capture_output=True)
  test.assertEqual(out.returncode,0,out.stderr);subprocess.run([str(exe)],check=True,timeout=10)

HOLDOFF=re.compile(r'#define NETWORK_CLOCK_RESYNC_HOLDOFF_TICKS.*?static ULONG g_network_clock_resync_tick;',re.S)
def pacing(source,ulong='unsigned long'):
 """ThreadX tick stubs plus the production restart pacing, when present."""
 found=HOLDOFF.search(source)
 return ('typedef '+ulong+' ULONG;\n#define TX_TIMER_TICKS_PER_SECOND 1000U\n'
         'static ULONG ticks=7;__attribute__((unused)) static ULONG tx_time_get(void){return ticks;}\n'
         +(found.group() if found else '')+'\n')

class NetworkClockResync(unittest.TestCase):
 def test_suspect_clock_restarts_timesync_and_is_not_observed(self):
  source=(ROOT/'Core/Src/telemetry.c').read_text()
  body=re.search(r'static void telemetry_update_network_health\(SedsRouter \*router\)\s*\{.*?\n\}',source,re.S).group()
  code=r'''#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
typedef int SedsResult;typedef int SedsRouter;
#define SEDS_OK 0
'''+pacing(source)+r'''static SedsRouter router;static uint64_t net;static bool valid,suspect;static unsigned configures,observed;static uint64_t last_observed=1;
static uint32_t g_telemetry_timesync_valid,g_telemetry_discovery_seen=1,g_telemetry_network_ready,g_network_clock_resyncs;
static SedsResult seds_router_get_network_time_ms(SedsRouter*r,uint64_t*out){assert(r==&router);if(!valid)return -1;*out=net;return SEDS_OK;}
static bool board_recovery_network_clock_resync_needed(uint64_t n){assert(n==net);return suspect;}
static SedsResult telemetry_configure_timesync_locked(SedsRouter*r){assert(r==&router);configures++;return SEDS_OK;}
/* DAQ publishes the observed sample to its acquisition clock cache. */
typedef struct{uint64_t utc;uint32_t tick;}daq_clock_cache_t;static daq_clock_cache_t g_acquisition_clock;
static uint32_t HAL_GetTick(void){return 7;}
static uint32_t __get_PRIMASK(void){return 0;}static void __disable_irq(void){}static void __set_PRIMASK(uint32_t m){(void)m;}
static daq_clock_cache_t daq_clock_cache_make(uint64_t utc,uint32_t tick){observed++;last_observed=utc;return (daq_clock_cache_t){utc,tick};}
'''+body+r'''
int main(void){
 (void)g_acquisition_clock;(void)HAL_GetTick;(void)__get_PRIMASK;(void)__disable_irq;(void)__set_PRIMASK;(void)daq_clock_cache_make;(void)observed;(void)last_observed;
 valid=false;telemetry_update_network_health(&router);assert(!configures&&!g_telemetry_timesync_valid&&!g_telemetry_network_ready);
 valid=true;net=1794183600000ULL;telemetry_update_network_health(&router);assert(!configures&&g_telemetry_timesync_valid&&g_telemetry_network_ready);
 OBSERVED_SYNCED
 net=5006972U;suspect=true;telemetry_update_network_health(&router);assert(configures==1&&g_network_clock_resyncs==1);
 OBSERVED_RESYNC
 suspect=false;telemetry_update_network_health(&router);assert(configures==1&&g_network_clock_resyncs==1);
 return 0;}
'''
  daq='daq_clock_cache_make(' in body
  code=code.replace('OBSERVED_SYNCED','assert(last_observed==net);' if daq else '')
  code=code.replace('OBSERVED_RESYNC','assert(last_observed==0);' if daq else '')
  compile_and_run(self,code)
 def test_gs_heartbeat_advances_station_clock(self):
  source=(ROOT/'Core/Src/telemetry.c').read_text()
  body=function(source,'static SedsResult telemetry_heartbeat_handler(')
  code=r'''#include <assert.h>
#include <stddef.h>
#include <stdint.h>
typedef int SedsResult;typedef int SedsRouter;typedef struct{uint64_t timestamp;}SedsPacketView;
#define SEDS_OK 0
static SedsRouter router;static struct{SedsRouter*r;}g_router={&router};static unsigned probes,observed;static uint64_t seen;
static SedsResult sim_probe_heartbeat_handler(const SedsPacketView*p,void*u){(void)p;(void)u;probes++;return SEDS_OK;}
static void board_recovery_observe_station_time(SedsRouter*r,const SedsPacketView*p){assert(r==&router&&p);observed++;seen=p->timestamp;}
'''+body+r'''
int main(void){SedsPacketView p={.timestamp=1794183600000ULL};
 assert(telemetry_heartbeat_handler(&p,NULL)==SEDS_OK&&observed==1&&seen==p.timestamp&&probes==1);
 assert(telemetry_heartbeat_handler(NULL,NULL)==SEDS_OK&&observed==1);return 0;}
'''
  compile_and_run(self,code)
 def test_restarts_are_paced_when_the_source_disagrees_with_the_gs(self):
  """Bench: a source ~607 s from the GS clock restarted timesync on every GS
  heartbeat, keeping the network clock invalid. A GS heartbeat arrives each
  second and the source re-anchors the clock every 2 s; the restart is
  needed each time. At most one restart per 30 s of ticks, across a wrap."""
  source=(ROOT/'Core/Src/telemetry.c').read_text()
  body=re.search(r'static void telemetry_update_network_health\(SedsRouter \*router\)\s*\{.*?\n\}',source,re.S).group()
  code=r"""#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
typedef int SedsResult;typedef int SedsRouter;
#define SEDS_OK 0
#define SEDS_FIRMWARE_SIM_TEST 0
"""+pacing(source,'uint32_t')+r"""
static SedsRouter router;static bool valid,station;static unsigned configures;
__attribute__((unused)) static uint32_t g_telemetry_timesync_valid,g_telemetry_discovery_seen=1,g_radio_link_seen=1,g_telemetry_network_ready,g_network_clock_resyncs;
static SedsResult seds_router_get_network_time_ms(SedsRouter*r,uint64_t*out){assert(r==&router);if(!valid)return -1;*out=1794183600000ULL-607000U;return SEDS_OK;}
/* The station clock restarts from the next GS heartbeat after each restart. */
static bool board_recovery_network_clock_resync_needed(uint64_t n){(void)n;if(!station)return false;station=false;return true;}
static SedsResult telemetry_configure_timesync_locked(SedsRouter*r){assert(r==&router);valid=false;configures++;return SEDS_OK;}
typedef struct{uint64_t utc;uint32_t tick;}daq_clock_cache_t;__attribute__((unused)) static daq_clock_cache_t g_acquisition_clock;
__attribute__((unused)) static uint32_t HAL_GetTick(void){return (uint32_t)ticks;}
__attribute__((unused)) static uint32_t __get_PRIMASK(void){return 0;}__attribute__((unused)) static void __disable_irq(void){}__attribute__((unused)) static void __set_PRIMASK(uint32_t m){(void)m;}
__attribute__((unused)) static daq_clock_cache_t daq_clock_cache_make(uint64_t utc,uint32_t tick){return (daq_clock_cache_t){utc,tick};}
"""+body+r"""
int main(void){
 const ULONG start=UINT32_MAX-60000U;
 for(ULONG t=0;t<120000U;t++){
  ticks=start+t;
  if(t%1000U==500U)station=true;
  if(t%2000U==37U)valid=true;
  telemetry_update_network_health(&router);
 }
 assert(configures==g_network_clock_resyncs);
 assert(configures>=4U&&configures<=5U);
 return 0;}
"""
  compile_and_run(self,code)
if __name__=='__main__':unittest.main()
