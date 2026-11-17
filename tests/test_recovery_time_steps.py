"""Recovery and command freshness across network time steps (SEDSnet 6).

Bench (2026-11-16): the GS clock stepped about 3 h 49 min back. Station
requests are now judged by seds_router_network_stamp_fresh() and ordered by
(time_epoch, timestamp): a step starts a newer epoch, so a request after a
step in either direction is later than the persisted recovery floor, a copy
stamped before the step is never fresh again, and a replay below the floor
is refused. No network time refuses everything but Abort.

The actual recovery transport, commissioning service, strict journal, boot
identity, floor and cache owner run over the recovery_wire fixture. The SDK
window is modeled as SEDSnet documents it: stamps from older epochs are
translated across the steps this board remembers, a stamp from a newer epoch
is fresh while the board has known of that epoch for at most the window.
"""
from pathlib import Path
import os,subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
def launchcore_source():
 try:
  from native_sdk import configured_dependency_source
 except ImportError:
  return Path(os.environ['SDK_FREE_LAUNCHCORE_SOURCE'])
 return configured_dependency_source(ROOT,'sedslaunchcore','SDK_FREE_LAUNCHCORE_SOURCE')
WINDOW_OLD_START='/* The SDK window over the fixture clock:'
WINDOW_OLD_END='if(out)*out=clock_epoch;return SEDS_OK;}'
WINDOW_NEW=r'''/* SDK step memory model: the board clock is in epoch clock_epoch and reads
 * clock_ms; step_delta[e] is the change of the reading at the step into e. */
static int net_mode; /* 0 network time, 1 none */
static uint16_t clock_epoch;static int64_t step_delta[16];static uint64_t heard_newer_at;static bool heard_newer;
SedsResult seds_router_network_stamp_fresh(SedsRouter*r,uint16_t e,uint64_t ts,uint64_t age,uint64_t ahead,bool*out){
 sdk_unlocked();assert(r&&out&&age==15000U&&ahead==1000U);
 if(net_mode==1){*out=false;return SEDS_OK;}
 if(e>clock_epoch){if(!heard_newer){heard_newer=true;heard_newer_at=clock_ms;}*out=clock_ms-heard_newer_at<=age;return SEDS_OK;}
 int64_t t=(int64_t)ts;for(unsigned k=(unsigned)e+1U;k<=clock_epoch;k++)t+=step_delta[k];
 const int64_t a=(int64_t)clock_ms-t;*out=a>=0?(uint64_t)a<=age:(uint64_t)(-a)<=ahead;return SEDS_OK;}
SedsResult seds_router_raise_time_epoch(SedsRouter*r,uint16_t e,uint16_t*out){sdk_unlocked();assert(r);if(e>clock_epoch)clock_epoch=e;if(out)*out=clock_epoch;return SEDS_OK;}
/* The GS stepped and this board followed: a new epoch, the reading moved. */
static void step(int64_t delta){assert(clock_epoch<15U);clock_epoch++;step_delta[clock_epoch]=delta;clock_ms=(uint64_t)((int64_t)clock_ms+delta);heard_newer=false;}'''
TICK_OLD='ULONG tx_time_get(void){return 1;}'
TICK_NEW='static ULONG ticks=1;ULONG tx_time_get(void){return ticks;}'
MAIN=r'''
#define T0 UINT64_C(1794183589988)
#define H4 INT64_C(14400000)
static uint8_t body[32];static uint32_t endpoint=SEDS_EP_BOARD_RECOVERY;
static SedsPacketView packet={.ty=200,.sender="GS",.sender_len=2,.endpoints=&endpoint,.num_endpoints=1,.payload=body,.payload_len=32};
static void set_req(uint8_t op,uint64_t nonce,uint64_t gen,uint32_t epoch,uint16_t time_epoch,uint64_t time){
 memset(body,0,sizeof body);body[0]=1;body[1]=op;put64(body+4,nonce);put64(body+12,gen);
 for(unsigned i=0;i<4;i++){body[20+i]=(uint8_t)(epoch>>(8*i));}
 put64(body+24,77);packet.timestamp=time;packet.time_epoch=time_epoch;
}
static uint8_t query(SedsRouter*r,uint64_t nonce,uint16_t e,uint64_t time){set_req(0,nonce,0,0,e,time);request(r,&packet,false);return response[1];}
static uint8_t recover(SedsRouter*r,uint64_t nonce,uint64_t gen,uint32_t epoch,uint16_t e,uint64_t time,bool service){set_req(1,nonce,gen,epoch,e,time);request(r,&packet,service);return response[1];}
/* A station FLIGHT_STATE/command as the board admits it. */
static bool admitted(uint16_t e,uint64_t time){SedsPacketView p={.ty=SEDS_DT_FLIGHT_STATE,.sender="GS",.sender_len=2,.timestamp=time,.time_epoch=e};return !board_recovery_command_is_stale(&p);}
static uint32_t live_epoch(void){BoardRecoverySnapshot s={0};assert(board_recovery_snapshot(&s)==0);return s.abort_epoch;}
static bool latched(void){BoardRecoverySnapshot s={0};assert(board_recovery_snapshot(&s)==0);return (s.flags&BOARD_RECOVERY_ABORT_LATCHED)!=0;}
static uint64_t boot(uint64_t floor_stamp){
 memset(disk,255,sizeof disk);launchcore_storage_set_driver(&launchcore_board_storage_driver);uint64_t prior=41;uint8_t original=1;uint64_t floor[2]={77,floor_stamp};
 assert(launchcore_persist_init()==0);assert(launchcore_persist_set(0x42524745U,&prior,8)==0);assert(launchcore_persist_set(0x46535445U,&original,1)==0);assert(launchcore_persist_set(0x4252464cU,floor,16)==0);
 fixture=false;(void)persistent_store_boot_begin();uint64_t generation=board_recovery_boot_generation();(void)board_recovery_floor_prime_boot();board_commissioning_probe_boot();flight_state_cache_restore();
 assert(persistent_store_runtime_start()==0);runtime=true;assert(persistent_store_controls_ready()&&generation&&generation!=UINT64_MAX);return generation;
}
static uint64_t floor_stamp(void){uint64_t f=0;assert(board_recovery_floor_read_cutoff(&f));return f;}
/* A first Reset at T0 in epoch 0: the floor is (0, T0+1). */
static void first_reset(SedsRouter*r,uint64_t gen){
 clock_ms=T0;assert(query(r,3,0,T0)==0);assert(recover(r,4,gen,1,0,T0+1,true)==BOARD_RECOVERY_RESULT_OK);
 assert(live_epoch()==2&&floor_stamp()==board_stamp_key(0,T0+1)&&board_recovery_floor_active());
}
int main(int argc,char**argv){
 assert(argc==2);const char*scenario=argv[1];(void)send;(void)response_epoch;
 const bool persisted=!strcmp(scenario,"persisted_epoch");
 uint64_t gen=boot(persisted?board_stamp_key(3,T0):100U);int cookie=0;SedsRouter*r=(SedsRouter*)&cookie;assert(flight_state_cache_init(r)==0);board_recovery_transport_set_router(r);
 assert(board_recovery_restore_time_epoch(r)==SEDS_OK);
 if(persisted){
  /* The floor was persisted in epoch 3: the board's epoch is raised to it at
   * boot, so a whole-stack restart (back at epoch 0) cannot put every new
   * station stamp below the floor. */
  assert(clock_epoch==3&&floor_stamp()==board_stamp_key(3,T0));
  clock_ms=T0+5000;assert(admitted(3,T0+5000));assert(!admitted(3,T0-1));
  assert(!admitted(2,T0+5000)); /* an older epoch is below the floor */
  puts("persisted floor epoch raises the router epoch at boot PASS");return 0;
 }
 /* A record written before epochs existed reads as epoch 0. */
 assert(clock_epoch==0&&floor_stamp()==board_stamp_key(0,100));
 if(!strcmp(scenario,"no_time")){
  net_mode=1;unsigned writes=media_writes;clock_ms=T0;
  assert(recover(r,2,gen,1,0,T0,false)==BOARD_RECOVERY_RESULT_STALE&&media_writes==writes&&!persistent_store_commission_pending());
  assert(!admitted(0,T0));
  net_mode=0;assert(query(r,3,0,T0+1)==0&&recover(r,4,gen,1,0,T0+2,true)==BOARD_RECOVERY_RESULT_OK);
  assert(persistent_store_controls_ready()&&!persistent_store_commission_pending()&&!latched()&&live_epoch()==2);
  puts("no network time refuses recovery and commands until time returns PASS");return 0;
 }
 if(!strcmp(scenario,"step_back")){
  first_reset(r,gen);
  assert(admitted(0,T0+2));
  step(-H4);
  /* A copy sent just before the step is as old as it really is (the step
   * is translated): fresh just after it, refused once it is late. */
  assert(admitted(0,T0+2));
  clock_ms+=16000;const uint64_t now=clock_ms;
  assert(!admitted(0,T0+2)&&!admitted(0,T0+500));
  /* A fresh request on the new clock is admitted at once, though its
   * timestamp is 4 h below the floor's. */
  assert(admitted(1,now));assert(!admitted(1,now-16000));
  /* A replay of a request stamped before the Reset stays refused. */
  unsigned writes=media_writes;
  assert(recover(r,5,gen,2,0,T0,false)==BOARD_RECOVERY_RESULT_STALE&&media_writes==writes);
  /* Reset after the step: accepted, and the floor moves to the new epoch. */
  assert(query(r,6,1,now)==0);assert(recover(r,7,gen,2,1,now+1,true)==BOARD_RECOVERY_RESULT_OK);
  assert(live_epoch()==3&&floor_stamp()==board_stamp_key(1,now+1));
  assert(admitted(1,now+2)&&!admitted(1,now)&&!admitted(0,T0+2));
  /* The pre-step Reset request replayed now is still refused. */
  writes=media_writes;assert(recover(r,8,gen,3,0,T0+1,false)==BOARD_RECOVERY_RESULT_STALE&&media_writes==writes);
  puts("4 h step back: fresh requests and Reset admitted, older copies refused PASS");return 0;
 }
 if(!strcmp(scenario,"step_forward")){
  first_reset(r,gen);
  step(INT64_C(13680000));
  assert(admitted(1,clock_ms)&&admitted(0,T0+2));
  /* A late copy stamped before the step is refused; its timestamp alone
   * (3.8 h behind the new clock) is never compared with the clock. */
  clock_ms+=16000;const uint64_t now=clock_ms;
  assert(admitted(1,now)&&!admitted(0,T0+2));
  assert(query(r,6,1,now)==0);assert(recover(r,7,gen,2,1,now+1,true)==BOARD_RECOVERY_RESULT_OK&&live_epoch()==3);
  assert(floor_stamp()==board_stamp_key(1,now+1));
  puts("3.8 h step forward: fresh requests admitted, older copies refused PASS");return 0;
 }
 if(!strcmp(scenario,"newer_epoch")){
  first_reset(r,gen);
  /* The GS stepped 4 h back; this board has not heard its announce yet. A
   * request in the newer epoch is fresh while the board re-anchors. */
  const uint64_t gs=T0-(uint64_t)H4+3000U;
  assert(admitted(1,gs));
  assert(query(r,6,1,gs)==0);assert(recover(r,7,gen,2,1,gs+1,true)==BOARD_RECOVERY_RESULT_OK&&live_epoch()==3);
  /* Not for longer than the window. */
  clock_ms+=16000;assert(!admitted(1,gs+16000));
  puts("newer-epoch request admitted while the board re-anchors PASS");return 0;
 }
 if(!strcmp(scenario,"replay_floor")){
  first_reset(r,gen);
  assert(!admitted(0,T0+1)&&!admitted(0,T0)&&admitted(0,T0+2));
  unsigned writes=media_writes;
  assert(recover(r,5,gen,2,0,T0,false)==BOARD_RECOVERY_RESULT_STALE&&media_writes==writes);
  puts("replay at or below the floor refused PASS");return 0;
 }
 if(!strcmp(scenario,"reset_stale")||!strcmp(scenario,"reset_held")){
  /* A running board told to reset; the request went stale before the
   * commissioning service reached it. */
  bool held=!strcmp(scenario,"reset_held");clock_ms=T0;
  if(held)persistent_store_hold_commissioning(); /* a hold that predates the job */
  set_req(1,41,gen,1,0,T0);unsigned replies=app_replies,writes=media_writes;
  assert(board_recovery_handle(r,&packet,endpoint,123)==SEDS_OK&&app_replies==replies);
  assert(persistent_store_commission_pending()&&!persistent_store_controls_ready());
  clock_ms+=16000;ticks+=16000;board_commissioning_service();
  assert(app_replies==replies+1&&response[1]==BOARD_RECOVERY_RESULT_STALE&&media_writes==writes);
  if(held){
   assert(persistent_store_commission_pending()&&latched()&&live_epoch()==2);
   puts("stale reset keeps a hold that predates it PASS");return 0;
  }
  assert(!persistent_store_commission_pending()&&persistent_store_controls_ready()&&!latched()&&live_epoch()==1);
  assert(query(r,43,0,T0+16001)==0&&recover(r,44,gen,1,0,T0+16002,true)==BOARD_RECOVERY_RESULT_OK&&live_epoch()==2);
  puts("stale reset of a running board latches nothing and releases its hold PASS");return 0;
 }
 return 2;
}
'''
class RecoveryTimeSteps(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.tmp=tempfile.TemporaryDirectory(prefix='recovery-time-steps-');p=Path(cls.tmp.name)
  sdk=Path(os.environ['SEDSNET_TEST_SOURCE']);lc=launchcore_source()
  fixture=ROOT/'tests/fixtures/recovery_wire'
  harness=(fixture/'harness.c').read_text().split('int main(int argc,char**argv){',1)[0]
  assert WINDOW_OLD_START in harness and TICK_OLD in harness
  a=harness.index(WINDOW_OLD_START);b=harness.index(WINDOW_OLD_END,a)+len(WINDOW_OLD_END)
  harness=harness[:a]+WINDOW_NEW+harness[b:]
  (p/'steps.c').write_text(harness.replace(TICK_OLD,TICK_NEW)+MAIN)
  names=['board_recovery_wire.c','board_recovery_boot.c','board_recovery_transport.c','flight_state_cache.c','persistent_store.c','board_recovery_query.c','board_recovery_floor.c','board_commissioning.c','ota_stream.c']
  cls.exe=p/'steps'
  subprocess.run(['cc','-DEXPECTED_CAP=32','-DTELEMETRY_ENABLED=1','-std=c11','-Wall','-Wextra','-Werror','-Wno-unused-function','-fsanitize=address,undefined','-I'+str(fixture),'-I'+str(ROOT/'Core/Inc'),'-I'+str(ROOT/'Bootloader'),'-I'+str(sdk/'C-Headers'),'-I'+str(lc/'bootloader/include'),'-I'+str(lc/'update_lib/include'),*[str(ROOT/'Core/Src'/n)for n in names],str(ROOT/'Bootloader/storage_dispatch.c'),str(lc/'bootloader/src/persist.c'),str(lc/'bootloader/src/crc32.c'),str(p/'steps.c'),'-o',str(cls.exe)],check=True)
 @classmethod
 def tearDownClass(cls):cls.tmp.cleanup()
 def run_case(self,name):
  out=subprocess.run([str(self.exe),name],capture_output=True,text=True,timeout=60)
  self.assertEqual(out.returncode,0,out.stdout+out.stderr)
 def test_no_network_time_refuses_until_time_returns(self):self.run_case('no_time')
 def test_backward_4_h_step_admits_fresh_requests_and_reset(self):self.run_case('step_back')
 def test_forward_3_8_h_step(self):self.run_case('step_forward')
 def test_newer_epoch_request_while_reanchoring(self):self.run_case('newer_epoch')
 def test_replay_below_floor_refused(self):self.run_case('replay_floor')
 def test_persisted_floor_epoch_raises_router_epoch(self):self.run_case('persisted_epoch')
 def test_stale_reset_of_running_board_latches_nothing(self):self.run_case('reset_stale')
 def test_stale_reset_keeps_prior_hold(self):self.run_case('reset_held')
if __name__=='__main__':unittest.main()
