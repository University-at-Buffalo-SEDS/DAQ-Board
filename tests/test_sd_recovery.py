"""Execute production card identity recovery and the no-format-on-I/O-fault guard."""
from pathlib import Path
import subprocess
import tempfile
import unittest
from test_sd_raw_queue import function

ROOT = Path(__file__).resolve().parents[1]
PREFIX = r'''
#include <stdint.h>
#define sd_media_flush() fx_media_flush(&g_sd_media)
#include <stddef.h>
#include <string.h>
#include <assert.h>
typedef unsigned UINT;typedef uint32_t ULONG;typedef unsigned char UCHAR;
#define FX_SUCCESS 0U
#define FX_IO_ERROR 1U
#define FX_NOT_FOUND 2U
#define FX_MEDIA_INVALID 3U
#define FX_BOOT_ERROR 4U
#define FX_FAT_READ_ERROR 5U
#define FX_NULL NULL
static unsigned g_sd_manual_no_format;
static unsigned g_sd_media,g_sd_media_mounted,g_sd_card_identity_mismatches,g_sd_mount_status;
static uint32_t g_sd_mounted_cid[4],g_sd_mounted_blocks;
static UCHAR g_sd_media_cache[4096];
static struct {uint32_t CID[4];struct{uint32_t LogBlockNbr;} SdCard;} hsd1;
static unsigned open_status,marker_status,volume_status,flush_status;
static unsigned opens,closes,formats,markers,volumes,flushes;
static void sd_filex_driver(void){}
static UINT fx_media_open(unsigned *media,const char *name,void(*driver)(void),void *unused,UCHAR *cache,size_t bytes){
 assert(media==&g_sd_media&&name&&driver==sd_filex_driver&&!unused&&cache==g_sd_media_cache&&bytes==sizeof(g_sd_media_cache));
 opens++;return open_status;
}
static UINT fx_media_close(unsigned *media){assert(media==&g_sd_media);closes++;return FX_SUCCESS;}
static UINT fx_media_volume_set(unsigned *media,const char *label){assert(media==&g_sd_media&&!strcmp(label,"SEDS_DAQ"));volumes++;return volume_status;}
static UINT fx_media_flush(unsigned *media){assert(media==&g_sd_media);flushes++;return flush_status;}
static UINT sd_marker_exists(void){markers++;return marker_status;}
static UINT sd_format_and_mark(void){formats++;return FX_SUCCESS;}
'''

class RecoveryTests(unittest.TestCase):
    def execute(self, body):
        source=(ROOT/'Core/Src/sd_card.c').read_text()
        code=PREFIX+function(source,'sd_mount_or_provision')+'\n'+function(source,'sd_resume_card_media')+body
        with tempfile.TemporaryDirectory() as directory:
            executable=Path(directory)/'recovery'
            built=subprocess.run(['cc','-std=c11','-O2','-Wall','-Wextra','-Werror',
                '-fsanitize=address,undefined','-x','c','-','-o',str(executable)],
                input=code,text=True,capture_output=True)
            self.assertEqual(built.returncode,0,built.stderr)
            subprocess.run([str(executable)],check=True)

    def test_same_card_resumes_dirty_media_and_original_handles_without_remount(self):
        self.execute(r'''
int main(void){
 const uint32_t original[]={0x1234,0x5678,0x9abc,0xdef0};
 memcpy(hsd1.CID,original,sizeof(original));hsd1.SdCard.LogBlockNbr=123456;
 assert(sd_resume_card_media()==FX_SUCCESS);
 assert(g_sd_media_mounted&&opens==1&&markers==1&&volumes==1&&flushes==1&&!formats&&!closes);
 assert(!memcmp(g_sd_mounted_cid,original,sizeof(original))&&g_sd_mounted_blocks==123456);
 /* Existing FileX/log objects, cached FAT bytes and pending application bytes
  * must not be closed, mounted over, formatted or advanced during recovery. */
 const struct {unsigned raw_handle,telemetry_handle,raw_offset,csv_offset,pending_length;} state={71,72,123,456,17};
 struct {unsigned raw_handle,telemetry_handle,raw_offset,csv_offset,pending_length;} handles;
 memcpy(&handles,&state,sizeof(handles));
 UCHAR pending[17],before[17];memset(pending,0x5a,sizeof(pending));memcpy(before,pending,sizeof(before));
 memset(g_sd_media_cache,0xa5,sizeof(g_sd_media_cache));
 open_status=FX_IO_ERROR; /* Must never consult mount while this media is owned. */
 for(unsigned i=0;i<100;i++)assert(sd_resume_card_media()==FX_SUCCESS);
 assert(opens==1&&!closes&&!formats&&flushes==1);
 hsd1.CID[2]^=1;assert(sd_resume_card_media()==FX_IO_ERROR&&g_sd_card_identity_mismatches==1);
 assert(g_sd_media_mounted&&opens==1&&!closes&&!formats);
 memcpy(hsd1.CID,original,sizeof(original));hsd1.SdCard.LogBlockNbr++;
 assert(sd_resume_card_media()==FX_IO_ERROR&&g_sd_card_identity_mismatches==2);
 assert(opens==1&&!closes&&!formats&&g_sd_mounted_blocks==123456);
 hsd1.SdCard.LogBlockNbr=123456;assert(sd_resume_card_media()==FX_SUCCESS);
 assert(opens==1&&!closes&&!formats&&flushes==1&&!memcmp(pending,before,sizeof(pending)));
 assert(!memcmp(&handles,&state,sizeof(handles)));
 for(unsigned i=0;i<sizeof(g_sd_media_cache);i++)assert(g_sd_media_cache[i]==0xa5);
}
''')

    def test_initial_mount_and_marker_dma_io_errors_never_trigger_format(self):
        self.execute(r'''
int main(void){
 hsd1.CID[0]=10;hsd1.SdCard.LogBlockNbr=10000;
 open_status=FX_IO_ERROR;
 for(unsigned i=0;i<10;i++)assert(sd_resume_card_media()==FX_IO_ERROR);
 open_status=FX_BOOT_ERROR;assert(sd_resume_card_media()==FX_BOOT_ERROR);
 open_status=FX_FAT_READ_ERROR;assert(sd_resume_card_media()==FX_FAT_READ_ERROR);
 /* Unknown errors must also fail closed; only MEDIA_INVALID authorizes
  * provisioning after an unsuccessful open. */
 open_status=99;assert(sd_resume_card_media()==99);
 assert(opens==13&&!formats&&!markers&&!closes&&!g_sd_media_mounted&&g_sd_mount_status==99);
 /* Read failed while looking up the marker on a mounted filesystem. That is
  * transport failure, not proof of a missing provisioning marker. */
 open_status=FX_SUCCESS;marker_status=FX_IO_ERROR;
 assert(sd_resume_card_media()==FX_IO_ERROR&&!formats&&!g_sd_media_mounted&&closes==1);
 marker_status=FX_SUCCESS;volume_status=FX_IO_ERROR;
 assert(sd_resume_card_media()==FX_IO_ERROR&&!formats&&!g_sd_media_mounted&&closes==2);
 volume_status=FX_SUCCESS;flush_status=FX_IO_ERROR;
 assert(sd_resume_card_media()==FX_IO_ERROR&&!formats&&!g_sd_media_mounted&&closes==3);
 flush_status=FX_SUCCESS;
 assert(sd_resume_card_media()==FX_SUCCESS&&g_sd_media_mounted&&!formats);
 unsigned open_count=opens;
 /* Subsequent peripheral recovery keeps the same open media even if another
  * physical mount would now fail, preserving dirty cache retry state. */
 open_status=FX_IO_ERROR;
 assert(sd_resume_card_media()==FX_SUCCESS&&opens==open_count&&!formats);
}
''')

    def test_manual_retry_never_formats_invalid_or_unmarked_media(self):
        self.execute(r"""
int main(void){
 g_sd_manual_no_format=1;open_status=FX_MEDIA_INVALID;
 for(unsigned i=0;i<10;i++)assert(sd_resume_card_media()==FX_MEDIA_INVALID);
 assert(!formats&&!g_sd_media_mounted);
 open_status=FX_SUCCESS;marker_status=FX_NOT_FOUND;
 for(unsigned i=0;i<10;i++)assert(sd_resume_card_media()==FX_NOT_FOUND);
 assert(!formats&&!g_sd_media_mounted&&closes==10);
 marker_status=FX_SUCCESS;
 assert(sd_resume_card_media()==FX_SUCCESS&&g_sd_media_mounted&&!formats);
}
""")
