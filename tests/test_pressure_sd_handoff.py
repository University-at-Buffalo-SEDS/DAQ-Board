"""Execute the actual callback handoff and SD writer with bounded mock I/O."""
from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[1]

def function(source, name):
    start = source.index(name + '(')
    start = source.rfind('\n', 0, start) + 1
    brace = source.index('{', start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]

class PressureSdHandoffTests(unittest.TestCase):
    def test_receive_copies_once_and_writer_retains_failed_row_without_duplicate(self):
        source = (ROOT/'Core/Src/sd_card.c').read_text()
        declarations = source[source.index('typedef struct {\n  pressure_sd_batch_t batch;'):source.index('static TX_EVENT_FLAGS_GROUP g_sd_flags;')]
        # Other static queue declarations are not needed by these two functions.
        declarations = declarations.replace('static ULONG g_sd_pressure_queue_storage[SD_PRESSURE_QUEUE_BATCHES * SD_PRESSURE_MESSAGE_WORDS];', '')
        prefix = r'''
#include "pressure_sd_wire.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
typedef uint32_t ULONG; typedef unsigned UINT; typedef unsigned char UCHAR;
typedef struct { unsigned tx_queue_enqueued; ULONG *tx_queue_read; } TX_QUEUE;
static TX_QUEUE g_sd_queue;
static uint32_t g_sd_route_epoch;
static uint32_t __get_PRIMASK(void){return 0;}
static void __disable_irq(void){}
static void __set_PRIMASK(uint32_t value){(void)value;}
#define TX_SUCCESS 0U
#define TX_NO_WAIT 0U
#define FX_SUCCESS 0U
#define SD_LINE_MAX 384U
typedef enum { SD_CARD_STATUS_OK, SD_CARD_STATUS_IO_ERROR, SD_CARD_STATUS_BUSY, SD_CARD_STATUS_BACKPRESSURE } sd_card_status_t;
typedef struct { uint16_t len; unsigned calibration; uint64_t session; uint32_t route_epoch; char line[]; } sd_line_slot_t;
static unsigned g_sd_services_initialized=1, g_sd_launch_closed;
static unsigned g_sd_write_error_count,g_sd_csv_rows_written_count;
static uint64_t sd_run_snapshot(void){return g_sd_launch_closed ? 0 : 42;}
static unsigned formatting,writing,fail_write,accepted; static char written[12][384];
static void sd_format_float(char *out,float v){++formatting;snprintf(out,24,"%.2f",(double)v);}
static int sd_format_csv_sensor_record(char *out,size_t n,uint64_t utc,uint64_t uptime,const char *sensor,const char *raw,const char *converted,uint32_t sequence){
 ++formatting;return snprintf(out,n,"%llu,%llu,%s,%s,%s,%u",(unsigned long long)utc,(unsigned long long)uptime,sensor,raw,converted,sequence);
}
static unsigned sd_calibration_snapshot(unsigned *value){*value=7;return 1;}
static unsigned sd_write_telemetry_row(const sd_line_slot_t *slot){
 ++writing;assert(slot->session==(g_sd_launch_closed ? 0U : 42U) && slot->calibration==7);
 if(fail_write){fail_write=0;return 1;}
 assert(accepted<12);memcpy(written[accepted++],slot->line,slot->len+1);return 0;
}
'''
        queues = r'''
static ULONG messages[2][SD_PRESSURE_MESSAGE_WORDS];static unsigned count;
static UINT tx_queue_send(TX_QUEUE *q,void *data,ULONG wait){assert(q==&g_sd_pressure_queue && wait==TX_NO_WAIT);if(count==2)return 1;memcpy(messages[count++],data,sizeof(messages[0]));return TX_SUCCESS;}
static UINT tx_queue_receive(TX_QUEUE *q,void *data,ULONG wait){assert(q==&g_sd_pressure_queue && wait==TX_NO_WAIT);if(!count)return 1;memcpy(data,messages[0],sizeof(messages[0]));memmove(messages[0],messages[1],sizeof(messages[0]));--count;return TX_SUCCESS;}
'''
        main = r'''
int main(void){
 pressure_sd_batch_t batch={.uptime_ms=100,.unix_ms=1000,.first_sequence=UINT32_MAX-1,.raw={1,2,3,4},.psi={1,2,3,4}};
 assert(sd_card_log_pressure_batch(NULL)==SD_CARD_STATUS_IO_ERROR);
 g_sd_services_initialized=0;assert(sd_card_log_pressure_batch(&batch)==SD_CARD_STATUS_BUSY);g_sd_services_initialized=1;
 assert(sd_card_log_pressure_batch(&batch)==SD_CARD_STATUS_OK);
 batch.uptime_ms=116;batch.unix_ms=1016;batch.first_sequence=2;
 assert(sd_card_log_pressure_batch(&batch)==SD_CARD_STATUS_OK);
 assert(sd_card_log_pressure_batch(&batch)==SD_CARD_STATUS_BACKPRESSURE);
 assert(g_sd_pressure_records_enqueued==8 && g_sd_pressure_records_dropped==8);
 memset(&batch,0xa5,sizeof(batch)); // The receive buffer is no longer owned.
 assert(formatting==0 && writing==0); // No CSV, pool or media work on RX.
 fail_write=1;assert(sd_service_pressure_rows()==1);
 assert(g_sd_pressure_pending && accepted==0 && formatting==2);
 assert(sd_service_pressure_rows()==1);
 assert(accepted==4 && formatting==8 && writing==5);
 assert(!strcmp(written[0],"1000,100,valve_pressure_psi,1,1.00,4294967294"));
 assert(!strcmp(written[3],"1012,112,valve_pressure_psi,4,4.00,1"));
 assert(sd_service_pressure_rows()==1 && accepted==8 && !g_sd_pressure_pending);
 assert(!strcmp(written[4],"1016,116,valve_pressure_psi,1,1.00,2"));
 assert(sd_service_pressure_rows()==0 && writing==9);
 assert(g_sd_pressure_records_written==8 && g_sd_csv_rows_written_count==8 && g_sd_write_error_count==1);
 g_sd_launch_closed=1;
 memset(&batch,0,sizeof(batch));
 assert(sd_card_log_pressure_batch(&batch)==SD_CARD_STATUS_OK);
 assert(sd_service_pressure_rows()==1 && accepted==12); /* Ordinary pressure continues. */
 return 0;
}
'''
        code=prefix+declarations+queues+function(source,'sd_telemetry_epoch_ready')+'\n'+function(source,'sd_service_pressure_rows')+'\n'+function(source,'sd_card_log_pressure_batch')+main
        with tempfile.TemporaryDirectory() as directory:
            executable=Path(directory)/'handoff'
            subprocess.run(['cc','-std=c11','-O1','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I',str(ROOT/'Core/Inc'),'-x','c','-','-o',str(executable)],input=code,text=True,check=True)
            subprocess.run([str(executable)],check=True)

    def test_pressure_queue_buffers_more_than_three_seconds_without_pool_allocation(self):
        source=(ROOT/'Core/Src/sd_card.c').read_text()
        declarations=source[source.index('typedef struct {\n  pressure_sd_batch_t batch;'):source.index('static uint8_t g_sd_pressure_pending;')]
        code=r'''
#include "pressure_sd_wire.h"
#include <assert.h>
#include <stdint.h>
#include <string.h>
typedef uintptr_t ULONG;typedef unsigned UINT;typedef int sd_card_status_t;
typedef struct {ULONG *storage;unsigned count,head,tail,capacity;} TX_QUEUE;
enum{TX_SUCCESS=0,TX_NO_WAIT=0,SD_CARD_STATUS_OK=0,SD_CARD_STATUS_BUSY=1,SD_CARD_STATUS_BACKPRESSURE=2,SD_CARD_STATUS_IO_ERROR=3};
static unsigned mask;
static uint32_t __get_PRIMASK(void){return mask;}
static void __disable_irq(void){mask=1;}
static void __set_PRIMASK(uint32_t value){mask=value;}
static uint64_t sd_run_snapshot(void){assert(mask);return 42;}
static unsigned g_sd_services_initialized=1,g_sd_route_epoch=77;
static unsigned g_sd_pressure_records_dropped,g_sd_pressure_records_enqueued;
'''+declarations+r'''
static UINT tx_queue_send(TX_QUEUE *queue,void *message,ULONG wait){
 assert(mask&&queue==&g_sd_pressure_queue&&wait==TX_NO_WAIT);
 if(queue->count==queue->capacity)return 1;
 memcpy(queue->storage+queue->tail*SD_PRESSURE_MESSAGE_WORDS,message,SD_PRESSURE_MESSAGE_WORDS*sizeof(ULONG));
 queue->tail=(queue->tail+1)%queue->capacity;queue->count++;return 0;
}
'''+function(source,'sd_card_log_pressure_batch')+r'''
int main(void){
 g_sd_pressure_queue.storage=g_sd_pressure_queue_storage;
 g_sd_pressure_queue.capacity=sizeof(g_sd_pressure_queue_storage)/(SD_PRESSURE_MESSAGE_WORDS*sizeof(ULONG));
 assert(g_sd_pressure_queue.capacity==200);
 for(unsigned i=0;i<200;i++){
  pressure_sd_batch_t batch={.uptime_ms=100+i*16,.unix_ms=1700000000000ULL+i*16,.first_sequence=i*4};
  for(unsigned j=0;j<4;j++){batch.raw[j]=i*4+j;batch.psi[j]=(float)(i*4+j)/3;}
  assert(sd_card_log_pressure_batch(&batch)==SD_CARD_STATUS_OK);
 }
 assert(g_sd_pressure_records_enqueued==800&&!g_sd_pressure_records_dropped);
 for(unsigned i=0;i<200;i++){
  sd_pressure_message_t message;
  memcpy(&message,g_sd_pressure_queue.storage+i*SD_PRESSURE_MESSAGE_WORDS,sizeof(message));
  assert(message.batch.uptime_ms==100+i*16&&message.batch.unix_ms==1700000000000ULL+i*16);
  assert(message.session==42&&message.route_epoch==77&&message.batch.first_sequence==i*4);
  for(unsigned j=0;j<4;j++)assert(message.batch.raw[j]==i*4+j&&message.batch.psi[j]==(float)(i*4+j)/3);
 }
 pressure_sd_batch_t overflow={0};assert(sd_card_log_pressure_batch(&overflow)==SD_CARD_STATUS_BACKPRESSURE);
 assert(g_sd_pressure_records_dropped==4);
}
'''
        with tempfile.TemporaryDirectory() as directory:
            executable=Path(directory)/'pressure-reserve'
            subprocess.run(['cc','-std=c11','-O1','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',
                '-I',str(ROOT/'Core/Inc'),'-x','c','-','-o',str(executable)],input=code,text=True,check=True)
            subprocess.run([str(executable)],check=True)

if __name__=='__main__': unittest.main()
