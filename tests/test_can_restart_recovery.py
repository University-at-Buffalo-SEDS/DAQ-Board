"""Fault injection against the actual foreground recovery function, not a copied implementation."""
from pathlib import Path
import subprocess, tempfile, unittest
ROOT=Path(__file__).resolve().parents[1]
class CanRestartRecovery(unittest.TestCase):
 def test_fault_clear_restarts_and_notification_failure_retries_without_resetting_sdk(self):
  source=(ROOT/'Core/Src/can_bus.c').read_text();start=source.index('/* Recovery runs only in the foreground.');end=source.index('static can_bus_rx_frame_t g_rx_ring',start);sut=source[start:end]
  code=r'''
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
typedef enum{HAL_OK,HAL_ERROR,HAL_BUSY}HAL_StatusTypeDef;
#define HAL_FDCAN_STATE_BUSY 2
#define HAL_FDCAN_STATE_READY 1
#define HAL_FDCAN_STATE_ERROR 3
#define FDCAN_TX_BUFFER0 1
#define FDCAN_TX_BUFFER1 2
#define FDCAN_TX_BUFFER2 4
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE 1
#define FDCAN_IT_RX_FIFO1_NEW_MESSAGE 2
#define FDCAN_IT_RX_FIFO0_MESSAGE_LOST 4
#define FDCAN_IT_RX_FIFO1_MESSAGE_LOST 8
#define FDCAN_IT_TX_FIFO_EMPTY 16
#define FDCAN_IT_TX_COMPLETE 32
#define CAN_BUS_POLLING 0
typedef struct{int State;}FDCAN_HandleTypeDef;
typedef struct{unsigned BusOff;}FDCAN_ProtocolStatusTypeDef;
static FDCAN_HandleTypeDef handle={HAL_FDCAN_STATE_BUSY},*g_hfdcan=&handle;
static unsigned bus_off,stop_timeout,fail_init,fail_filter,fail_notify,starts,stops,inits,resets,notifications;
static uint32_t now;
static unsigned g_fdcan_bus_off_count,g_fdcan_recovery_count;
static uint8_t g_can_recovering;
static uint32_t HAL_GetTick(void){return now;}
static HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef*h,FDCAN_ProtocolStatusTypeDef*p){(void)h;p->BusOff=bus_off;return HAL_OK;}
static HAL_StatusTypeDef HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef*h,unsigned n){assert(g_can_recovering);(void)h;(void)n;return HAL_OK;}
/* Checked-in ST HAL state contract: Stop timeout leaves ERROR. Start needs READY. */
static HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef*h){assert(g_can_recovering);++stops;if(h->State!=HAL_FDCAN_STATE_BUSY)return HAL_ERROR;if(stop_timeout){stop_timeout=0;h->State=HAL_FDCAN_STATE_ERROR;return HAL_ERROR;}h->State=HAL_FDCAN_STATE_READY;return HAL_OK;}
static HAL_StatusTypeDef HAL_FDCAN_Init(FDCAN_HandleTypeDef*h){assert(g_can_recovering);++inits;if(fail_init)return HAL_ERROR;h->State=HAL_FDCAN_STATE_READY;return HAL_OK;}
static HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef*h){assert(g_can_recovering);++starts;if(h->State!=HAL_FDCAN_STATE_READY)return HAL_ERROR;h->State=HAL_FDCAN_STATE_BUSY;bus_off=0;return HAL_OK;}
static HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef*h,unsigned mask,unsigned buffers){assert(g_can_recovering);assert(h->State==HAL_FDCAN_STATE_BUSY);assert(mask==63&&buffers==7);++notifications;return fail_notify?HAL_ERROR:HAL_OK;}
static void can_tx_queue_reset(void){assert(g_can_recovering);++resets;}
'''+sut+r'''
static HAL_StatusTypeDef can_bus_configure_filters(FDCAN_HandleTypeDef*h){assert(g_can_recovering);assert(h->State==HAL_FDCAN_STATE_READY);return fail_filter?HAL_ERROR:HAL_OK;}
static void fresh(void){handle.State=HAL_FDCAN_STATE_BUSY;bus_off=stop_timeout=fail_init=fail_filter=fail_notify=starts=stops=inits=resets=notifications=0;g_can_restart_pending=g_can_recovery_attempted=g_can_recovering=0;g_can_last_recovery_attempt_ms=0;now=0;}
int main(void){
 fresh();for(unsigned n=0;n<10000;n++){now=n;assert(can_bus_recover_if_bus_off()==HAL_OK);}assert(!starts&&!stops&&!inits&&!notifications); /* healthy fastpath untouched */
 fresh();bus_off=1;stop_timeout=1;assert(can_bus_recover_if_bus_off()==HAL_ERROR);assert(handle.State==HAL_FDCAN_STATE_ERROR&&g_can_recovering);bus_off=0;
 for(now=1;now<100;now++){assert(can_bus_recover_if_bus_off()==HAL_BUSY);}
 assert(!starts&&!inits);
 assert(can_bus_recover_if_bus_off()==HAL_OK);assert(handle.State==HAL_FDCAN_STATE_BUSY&&inits==1&&starts==1&&notifications==1&&!g_can_recovering);assert(g_fdcan_recovery_count==1);
 fresh();handle.State=HAL_FDCAN_STATE_ERROR;fail_init=1;assert(can_bus_recover_if_bus_off()==HAL_ERROR);now=100;fail_init=0;fail_filter=1;assert(can_bus_recover_if_bus_off()==HAL_ERROR);assert(g_can_recovering&&starts==0);now=200;fail_filter=0;fail_notify=1;assert(can_bus_recover_if_bus_off()==HAL_ERROR);assert(handle.State==HAL_FDCAN_STATE_BUSY&&g_can_recovering);now=300;fail_notify=0;assert(can_bus_recover_if_bus_off()==HAL_OK);assert(!g_can_recovering&&notifications==2&&stops==1);
 fresh();handle.State=HAL_FDCAN_STATE_READY;assert(can_bus_recover_if_bus_off()==HAL_OK);assert(starts==1&&notifications==1&&inits==0);
 fresh();now=UINT32_MAX-49;bus_off=1;stop_timeout=1;assert(can_bus_recover_if_bus_off()==HAL_ERROR);bus_off=0;now=49;assert(can_bus_recover_if_bus_off()==HAL_BUSY);now=50;assert(can_bus_recover_if_bus_off()==HAL_OK);assert(starts==1);
 /* A second real bus fault is independently recoverable, with original queue reset. */
 now=150;bus_off=1;assert(can_bus_recover_if_bus_off()==HAL_OK);assert(starts==2&&resets==2);puts("healthy10000passes; StopERROR/clearedBO; repeatedinit/filter/IRQfaults; READY; wrap; repeatedbusoff: PASS");return 0;}
'''
  with tempfile.TemporaryDirectory()as directory:
   c=Path(directory)/'recovery.c';exe=Path(directory)/'recovery';c.write_text(code);subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-omit-frame-pointer',str(c),'-o',str(exe)],check=True);subprocess.run([str(exe)],check=True)
if __name__=='__main__':unittest.main()
