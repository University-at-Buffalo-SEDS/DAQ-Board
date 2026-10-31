"""Execute bounded FileX reserve/trim policy, including failure/retry semantics."""
from pathlib import Path
import subprocess
import tempfile
import unittest
from test_sd_raw_queue import function
ROOT=Path(__file__).resolve().parents[1]
class SdPreallocationTests(unittest.TestCase):
    def test_reserve_preserves_logical_csv_size_and_fragmented_media_fallback(self):
        source=(ROOT/'Core/Src/sd_card.c').read_text()
        helpers=source[source.index('#define SD_FILE_RESERVE_BYTES'):source.index('static UINT sd_media_flush')]
        code=r'''
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#include "sd_file_limit.h"
#include "sd_storage_config.h"
typedef unsigned UINT;typedef uint32_t ULONG;
#define FX_SUCCESS 0U
#define FX_NO_MORE_SPACE 1U
#define FX_IO_ERROR 2U
typedef struct {uint64_t fx_file_current_available_size,fx_file_current_file_offset,fx_file_current_file_size;} FX_FILE;
static FX_FILE g_sd_file,g_sd_telemetry_file;
static uint8_t g_sd_raw_reserve_disabled,g_sd_telemetry_reserve_disabled;
static unsigned g_sd_preallocation_attempts,g_sd_preallocation_fallbacks,g_sd_preallocation_errors,g_sd_max_preallocate_ms;
static unsigned g_sd_raw_records_enqueued_count,g_sd_raw_records_written_count,g_sd_raw_records_io_lost_count;
static unsigned tick,allocation_status,release_status,allocate_calls,release_calls;static ULONG allocation_request;
static uint32_t HAL_GetTick(void){return tick;}
static UINT fx_file_allocate(FX_FILE *file,ULONG bytes){
 assert(bytes&&bytes<=2U*1024*1024);allocate_calls++;allocation_request=bytes;tick+=11;
 if(allocation_status)return allocation_status;
 file->fx_file_current_available_size+=bytes;return FX_SUCCESS;
}
static UINT fx_file_seek(FX_FILE *file,ULONG offset){file->fx_file_current_file_offset=offset;return FX_SUCCESS;}
static UINT fx_file_truncate_release(FX_FILE *file,ULONG bytes){
 release_calls++;if(release_status)return release_status;
 assert(bytes==file->fx_file_current_file_size);file->fx_file_current_available_size=(bytes+32767)/32768*32768;
 return FX_SUCCESS;
}
'''+helpers+r'''
int main(void){
 assert(sd_prepare_file_space(&g_sd_file,160)==FX_SUCCESS);
 assert(allocate_calls==1&&allocation_request==2U*1024*1024);
 assert(!g_sd_file.fx_file_current_file_size&&!g_sd_file.fx_file_current_file_offset);
 assert(g_sd_max_preallocate_ms==11&&g_sd_preallocation_attempts==1);
 g_sd_file.fx_file_current_file_offset=1000;g_sd_file.fx_file_current_file_size=1000;
 assert(sd_prepare_file_space(&g_sd_file,4000)==FX_SUCCESS&&allocate_calls==1);
 g_sd_file.fx_file_current_file_offset=2U*1024*1024-128U*1024;g_sd_file.fx_file_current_file_size=g_sd_file.fx_file_current_file_offset;
 g_sd_raw_records_enqueued_count=300; /* Do not add FAT writes while raw backlog is pressured. */
 assert(sd_prepare_file_space(&g_sd_file,4000)==FX_SUCCESS&&allocate_calls==1);
 g_sd_raw_records_written_count=100;
 uint64_t before=g_sd_file.fx_file_current_file_size,offset=g_sd_file.fx_file_current_file_offset;
 assert(sd_prepare_file_space(&g_sd_file,4000)==FX_SUCCESS&&allocate_calls==2);
 assert(g_sd_file.fx_file_current_file_size==before&&g_sd_file.fx_file_current_file_offset==offset);
 allocation_status=FX_NO_MORE_SPACE;
 assert(sd_prepare_file_space(&g_sd_telemetry_file,4000)==FX_SUCCESS);
 assert(g_sd_telemetry_reserve_disabled&&!g_sd_raw_reserve_disabled&&g_sd_preallocation_fallbacks==1);
 unsigned calls=allocate_calls;
 for(unsigned i=0;i<1000;i++)assert(sd_prepare_file_space(&g_sd_telemetry_file,4000)==FX_SUCCESS);
 assert(allocate_calls==calls); /* Fragmentation fallback is once/file, never once/row. */
 g_sd_telemetry_reserve_disabled=0;allocation_status=FX_IO_ERROR;
 assert(sd_prepare_file_space(&g_sd_telemetry_file,4000)==FX_IO_ERROR&&g_sd_preallocation_errors==1);
 assert(!g_sd_telemetry_reserve_disabled&&!g_sd_telemetry_file.fx_file_current_available_size);
 allocation_status=0;assert(sd_prepare_file_space(&g_sd_telemetry_file,4000)==FX_SUCCESS);
 g_sd_telemetry_file.fx_file_current_file_size=1000;g_sd_telemetry_file.fx_file_current_file_offset=1000;
 release_status=FX_IO_ERROR;assert(sd_release_file_space(&g_sd_telemetry_file)==FX_IO_ERROR);
 assert(g_sd_telemetry_file.fx_file_current_available_size==2U*1024*1024);
 release_status=0;assert(sd_release_file_space(&g_sd_telemetry_file)==FX_SUCCESS);
 assert(g_sd_telemetry_file.fx_file_current_available_size==32768&&g_sd_telemetry_file.fx_file_current_file_size==1000);
 assert(g_sd_telemetry_file.fx_file_current_file_offset==1000&&release_calls==2);
 g_sd_file.fx_file_current_available_size=SD_LOG_PART_LIMIT_BYTES-65536;
 g_sd_file.fx_file_current_file_offset=SD_LOG_PART_LIMIT_BYTES-1024;
 g_sd_file.fx_file_current_file_size=g_sd_file.fx_file_current_file_offset;
 assert(sd_prepare_file_space(&g_sd_file,512)==FX_SUCCESS&&allocation_request==65536);
 calls=allocate_calls;assert(sd_prepare_file_space(&g_sd_file,512)==FX_SUCCESS&&allocate_calls==calls);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            binary=Path(tmp)/'reserve'
            subprocess.run(['cc','-DDAQ_SD_PREALLOCATE_ENABLE=1','-std=c11','-O1','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',
                '-I',str(ROOT/'Core/Inc'),'-x','c','-','-o',str(binary)],input=code,text=True,check=True)
            subprocess.run([str(binary)],check=True)
            off = code.split('int main(void){', 1)[0] + r"""
#include <string.h>
typedef void VOID;
static unsigned g_sd_write_error_count;
static unsigned char g_sd_write_buffer[4096],saved[32];
static size_t g_sd_write_buffer_len;
static UINT fx_file_write(FX_FILE *file,VOID *bytes,ULONG length){
 assert(length<=sizeof(saved));memcpy(saved,bytes,length);
 file->fx_file_current_file_offset+=length;file->fx_file_current_file_size=file->fx_file_current_file_offset;return FX_SUCCESS;
}
""" + function(source, 'sd_flush_pending') + r"""
int main(void){
 assert(DAQ_SD_PREALLOCATE_ENABLE==0);
 assert(sd_prepare_file_space(&g_sd_telemetry_file,123)==FX_SUCCESS);
 memcpy(g_sd_write_buffer,"ordinary CSV\r\n",14);g_sd_write_buffer_len=14;
 assert(sd_flush_pending()==FX_SUCCESS);
 assert(!memcmp(saved,"ordinary CSV\r\n",14)&&g_sd_file.fx_file_current_file_size==14);
 assert(!g_sd_write_buffer_len&&!allocate_calls&&!g_sd_preallocation_attempts&&!g_sd_write_error_count);
 assert(sd_release_file_space(&g_sd_file)==FX_SUCCESS&&!release_calls);
}
"""
            off_binary=Path(tmp)/'ordinary'
            subprocess.run(['cc','-std=c11','-O1','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',
                '-I',str(ROOT/'Core/Inc'),'-x','c','-','-o',str(off_binary)],input=off,text=True,check=True)
            subprocess.run([str(off_binary)],check=True)

