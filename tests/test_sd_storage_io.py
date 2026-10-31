"""Run the production DMA/event adapter with a deterministic HAL/ThreadX model."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

TX = r'''
#ifndef TX_API_H
#define TX_API_H
#include <stdint.h>
typedef unsigned UINT;typedef unsigned char UCHAR;typedef uint32_t ULONG;
typedef struct{ULONG flags;} TX_EVENT_FLAGS_GROUP;
#define TX_SUCCESS 0U
#define TX_NO_WAIT 0U
#define TX_OR 0U
#define TX_OR_CLEAR 1U
UINT tx_event_flags_create(TX_EVENT_FLAGS_GROUP*,char*);
UINT tx_event_flags_get(TX_EVENT_FLAGS_GROUP*,ULONG,UINT,ULONG*,ULONG);
UINT tx_event_flags_set(TX_EVENT_FLAGS_GROUP*,ULONG,UINT);
void tx_thread_sleep(ULONG);
#endif
'''
MAIN = r'''
#ifndef MAIN_H
#define MAIN_H
#include <stdint.h>
typedef unsigned HAL_StatusTypeDef;typedef unsigned HAL_SD_CardStateTypeDef;
typedef struct{volatile uint32_t MASK,IDMACTRL,DCTRL;} SDMMC_TypeDef;
typedef struct{SDMMC_TypeDef *Instance;uint32_t State,ErrorCode,Context;} SD_HandleTypeDef;
#define HAL_OK 0U
#define HAL_SD_CARD_TRANSFER 4U
#define HAL_SD_CARD_PROGRAMMING 7U
#define HAL_SD_CARD_ERROR 255U
#define HAL_SD_STATE_READY 1U
#define HAL_SD_STATE_RESET 0U
#define HAL_SD_ERROR_NONE 0U
#define HAL_SD_ERROR_TIMEOUT 8U
#define SD_CONTEXT_NONE 0U
#define SDMMC_DISABLE_IDMA 0U
#define SDMMC_STATIC_FLAGS 0xffffU
#define SDMMC1_IRQn 49U
void HAL_NVIC_DisableIRQ(unsigned);
void HAL_NVIC_ClearPendingIRQ(unsigned);
void HAL_NVIC_EnableIRQ(unsigned);
uint32_t HAL_GetTick(void);
void HAL_SD_IRQHandler(SD_HandleTypeDef*);
HAL_SD_CardStateTypeDef HAL_SD_GetCardState(SD_HandleTypeDef*);
HAL_StatusTypeDef HAL_SD_WriteBlocks_DMA(SD_HandleTypeDef*,const uint8_t*,uint32_t,uint32_t);
HAL_StatusTypeDef HAL_SD_ReadBlocks_DMA(SD_HandleTypeDef*,uint8_t*,uint32_t,uint32_t);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t);
void barrier(void);
void force_reset(void);
void release_reset(void);
void clear_flags(void);
#define __DSB() barrier()
#define __DMB() barrier()
#define __HAL_RCC_SDMMC1_FORCE_RESET() force_reset()
#define __HAL_RCC_SDMMC1_RELEASE_RESET() release_reset()
#define __HAL_SD_CLEAR_FLAG(h,f) clear_flags()
#endif
'''
PROGRAM = r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "tx_api.h"
#include "fx_api.h"
static SDMMC_TypeDef peripheral;
SD_HandleTypeDef hsd1={&peripheral,HAL_SD_STATE_READY,0,0};
static uint32_t tick,mask;static unsigned in_irq,irq_enabled,dma_owned;
static unsigned scenario,irq_pending,hal_irq_calls,program_until,ready_error,spurious;
static unsigned start_calls,reset_calls,sleep_calls,last_sleep,invalid_reentry;
static uint8_t *dma_buffer;static uint32_t dma_sector,dma_count;static unsigned dma_write;
#include PRODUCTION
uint32_t __get_PRIMASK(void){return mask;}
void __disable_irq(void){mask=1;}
void __set_PRIMASK(uint32_t m){mask=m;}
void barrier(void){}
void force_reset(void){assert(!in_irq);++reset_calls;memset(&peripheral,0,sizeof(peripheral));dma_owned=0;irq_pending=0;}
void release_reset(void){}
void clear_flags(void){assert(!in_irq);}
uint32_t HAL_GetTick(void){return tick;}
void HAL_NVIC_DisableIRQ(unsigned irq){assert(irq==SDMMC1_IRQn);irq_enabled=0;}
void HAL_NVIC_ClearPendingIRQ(unsigned irq){assert(irq==SDMMC1_IRQn);irq_pending=0;}
static void interrupt(void){assert(irq_enabled);in_irq=1;sd_storage_io_irq();in_irq=0;assert(!irq_enabled);}
void HAL_NVIC_EnableIRQ(unsigned irq){assert(irq==SDMMC1_IRQn);irq_enabled=1;if(irq_pending&&scenario==0){irq_pending=0;interrupt();}}
UINT tx_event_flags_create(TX_EVENT_FLAGS_GROUP *g,char *name){assert(!in_irq);(void)name;g->flags=0;return 0;}
UINT tx_event_flags_set(TX_EVENT_FLAGS_GROUP *g,ULONG flags,UINT mode){assert(mode==TX_OR);g->flags|=flags;return 0;}
UINT tx_event_flags_get(TX_EVENT_FLAGS_GROUP *g,ULONG wanted,UINT mode,ULONG *out,ULONG wait){
 assert(!in_irq&&!mask&&mode==TX_OR_CLEAR);
 if(!(g->flags&wanted)&&wait){
  assert(dma_owned);
  if(irq_pending&&scenario!=4){tick+=25;irq_pending=0;interrupt();}
  else tick+=(uint32_t)((uint64_t)wait*1000/TX_TIMER_TICKS_PER_SECOND);
 }
 *out=g->flags&wanted;g->flags&=~wanted;return *out?0:1;
}
void tx_thread_sleep(ULONG ticks){assert(!in_irq&&!mask);assert(ticks>=TX_TIMER_TICKS_PER_SECOND/1000);sleep_calls++;last_sleep=ticks;tick+=(uint32_t)((uint64_t)ticks*1000/TX_TIMER_TICKS_PER_SECOND);}
HAL_SD_CardStateTypeDef HAL_SD_GetCardState(SD_HandleTypeDef *h){assert(!in_irq&&!mask&&h==&hsd1);return ready_error?HAL_SD_CARD_ERROR:tick<program_until?HAL_SD_CARD_PROGRAMMING:HAL_SD_CARD_TRANSFER;}
static void complete(void){
 if(!dma_write)for(unsigned i=0;i<dma_count*512;i++)dma_buffer[i]=(uint8_t)(dma_sector+i/512);
 dma_owned=0;peripheral.IDMACTRL=0;hsd1.State=HAL_SD_STATE_READY;
 program_until=tick+3;
 if(dma_write)HAL_SD_TxCpltCallback(&hsd1);else HAL_SD_RxCpltCallback(&hsd1);
}
static HAL_StatusTypeDef launch(uint8_t *buffer,uint32_t sector,uint32_t count,unsigned write){
 assert(!in_irq&&!mask&&!dma_owned);start_calls++;dma_buffer=buffer;dma_sector=sector;dma_count=count;dma_write=write;
 dma_owned=1;peripheral.IDMACTRL=1;peripheral.MASK=0x7;hsd1.State=2;
 if(invalid_reentry){invalid_reentry=0;assert(sd_storage_io_read(buffer,0,1)==FX_IO_ERROR&&dma_owned);}
 if(scenario==2){complete();return HAL_OK;} /* callback before the event wait */
 if(scenario==5)return 1; /* HAL partially armed IDMA before rejecting command */
 irq_pending=1;return HAL_OK;
}
HAL_StatusTypeDef HAL_SD_WriteBlocks_DMA(SD_HandleTypeDef *h,const uint8_t *p,uint32_t s,uint32_t n){assert(h==&hsd1);return launch((uint8_t*)p,s,n,1);}
HAL_StatusTypeDef HAL_SD_ReadBlocks_DMA(SD_HandleTypeDef *h,uint8_t *p,uint32_t s,uint32_t n){assert(h==&hsd1);return launch(p,s,n,0);}
void HAL_SD_IRQHandler(SD_HandleTypeDef *h){
 assert(!in_irq&&!mask&&h==&hsd1&&!irq_enabled&&dma_owned);hal_irq_calls++;
 /* Model the multiblock STOP running here: this is never an IRQ context. */
 tick+=1;
 if(scenario==6&&spurious){--spurious;irq_pending=1;return;}
 if(scenario==3){hsd1.ErrorCode=16;HAL_SD_ErrorCallback(h);HAL_SD_TxCpltCallback(h);return;}
 complete();
}
static void reinitialize(void){hsd1.State=HAL_SD_STATE_READY;hsd1.ErrorCode=0;ready_error=0;program_until=0;}
static _Alignas(32) UCHAR buffer[24*512];
int main(void){
 sd_storage_io_irq();assert(g_sd_dma_stale_irqs==1);
 assert(sd_storage_io_init()==TX_SUCCESS&&sd_storage_io_init()==TX_SUCCESS);
 assert(sd_storage_io_read(buffer+1,0,1)==FX_IO_ERROR);
 assert(sd_storage_io_read(buffer,0,0)==FX_IO_ERROR&&sd_storage_io_read(buffer,0,9)==FX_IO_ERROR&&!start_calls);
 scenario=0;invalid_reentry=1;assert(sd_storage_io_read(buffer,10,8)==FX_SUCCESS);
 assert(!dma_owned&&!irq_enabled&&hal_irq_calls==1&&g_sd_dma_reads==1);
 for(unsigned i=0;i<4096;i++)assert(buffer[i]==(uint8_t)(10+i/512));
 assert(sleep_calls&&last_sleep>0);
 scenario=1;assert(sd_storage_io_write(buffer,10,8)==FX_SUCCESS&&g_sd_dma_writes==1);
 scenario=2;unsigned before=hal_irq_calls;assert(sd_storage_io_read(buffer,20,1)==FX_SUCCESS&&hal_irq_calls==before);
 scenario=6;spurious=1;before=hal_irq_calls;
 assert(sd_storage_io_read(buffer,30,1)==FX_SUCCESS&&hal_irq_calls==before+2&&!dma_owned);
 scenario=3;assert(sd_storage_io_write(buffer,0,8)==FX_IO_ERROR);
 assert(!dma_owned&&!irq_enabled&&sd_storage_io_requires_reinit()&&reset_calls==1);
 assert(sd_storage_io_read(buffer,0,1)==FX_IO_ERROR); /* cannot reuse reset card */
 reinitialize();scenario=4;assert(sd_storage_io_write(buffer,0,8)==FX_IO_ERROR);
 assert(!dma_owned&&!irq_enabled&&g_sd_dma_timeouts==1&&reset_calls==2);
 unsigned stale=g_sd_dma_stale_irqs;in_irq=1;sd_storage_io_irq();in_irq=0;
 assert(g_sd_dma_stale_irqs==stale+1&&!dma_owned);
 HAL_SD_RxCpltCallback(&hsd1);assert(!g_sd_io_events.flags); /* late old callback */
 reinitialize();scenario=5;assert(sd_storage_io_read(buffer,0,1)==FX_IO_ERROR&&!dma_owned&&reset_calls==3);
 reinitialize();ready_error=1;unsigned starts=start_calls;
 assert(sd_storage_io_read(buffer,0,1)==FX_IO_ERROR&&start_calls==starts&&!dma_owned&&reset_calls==4);
 reinitialize();program_until=tick+10000;unsigned timeouts=g_sd_dma_timeouts;
 assert(sd_storage_io_read(buffer,0,1)==FX_IO_ERROR&&g_sd_dma_timeouts==timeouts+1&&!dma_owned);
 reinitialize();scenario=1;
 for(unsigned sector=0;sector<24;sector+=8)assert(sd_storage_io_read(buffer+sector*512,sector,8)==FX_SUCCESS);
 for(unsigned i=0;i<sizeof(buffer);i++)assert(buffer[i]==(uint8_t)(i/512));
 assert(!sd_storage_io_requires_reinit()&&!dma_owned&&!g_sd_io_active&&!g_sd_io_events.flags);
 printf("DMA isolation tick%uHz: callback-before-wait, delayed/error/timeout/lateIRQ, ownership and3chunks pass; control%zu bytes\n",TX_TIMER_TICKS_PER_SECOND,sizeof(g_sd_io_events));
}
'''

class StorageIoTests(unittest.TestCase):
    def test_dma_isolation_and_faults_at_supported_tick_rates(self):
        with tempfile.TemporaryDirectory() as folder:
            directory=Path(folder)
            (directory/'tx_api.h').write_text(TX)
            (directory/'main.h').write_text(MAIN)
            (directory/'fx_api.h').write_text('#define FX_SUCCESS 0U\n#define FX_IO_ERROR 2U\n')
            production=str(ROOT/'Core/Src/sd_storage_io.c')
            for ticks in (1000,50000):
                binary=directory/f'io-{ticks}'
                result=subprocess.run(['cc','-std=c11','-O2','-Wall','-Wextra','-Werror',
                    '-fsanitize=address,undefined',f'-DTX_TIMER_TICKS_PER_SECOND={ticks}',
                    '-DPRODUCTION="'+production+'"','-I',str(directory),'-I',str(ROOT/'Core/Inc'),
                    '-x','c','-','-o',str(binary)],input=PROGRAM,text=True,capture_output=True)
                self.assertEqual(result.returncode,0,result.stderr)
                subprocess.run([str(binary)],check=True)
