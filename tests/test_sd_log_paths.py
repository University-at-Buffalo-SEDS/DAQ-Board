"""Run actual folder/file creation with finite, failing FileX contracts."""
from pathlib import Path
import subprocess
import tempfile
import unittest
from test_sd_optional_retention import production_function
ROOT=Path(__file__).resolve().parents[1]

class LogPathsTests(unittest.TestCase):
    def test_actual_bounded_path_creation_clock_categories_and_media_errors(self):
        source=(ROOT/'Core/Src/sd_card.c').read_text()
        code=r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "sd_log_path.h"
#include "sd_record_format.h"
typedef unsigned UINT;typedef char CHAR;
#define FX_SUCCESS 0U
#define FX_ALREADY_CREATED 1U
#define FX_DIRECTORY_ERROR 2U
#define FX_PTR_ERROR 3U
#define FX_INVALID_NAME 4U
#define FX_NO_MORE_SPACE 5U
#define FX_IO_ERROR 6U
#define FX_DIRECTORY 0x10U
static unsigned g_sd_media,dir_status,attr_status,attrs=FX_DIRECTORY,collisions,creates,dirs;
static char last[128],directory[16];
static UINT fx_directory_create(unsigned*m,CHAR*p){(void)m;dirs++;strcpy(directory,p);return dir_status;}
static UINT fx_directory_attributes_read(unsigned*m,CHAR*p,UINT*a){(void)m;assert(!strcmp(directory,p));*a=attrs;return attr_status;}
static UINT fx_file_create(unsigned*m,char*p){(void)m;creates++;strcpy(last,p);if(collisions){collisions--;return FX_ALREADY_CREATED;}return FX_SUCCESS;}
'''+production_function(source,'sd_create_log_file')+r'''
int main(void){
 char name[64];
 assert(sd_create_log_file(name,sizeof name,"DAQ",0,123)==FX_SUCCESS);
 assert(!strcmp(name,"/NO_UTC/DAQ_123_000.CSV"));
 assert(sd_create_log_file(name,sizeof name,"DAQ_TELEMETRY",1767225600000ULL,456)==FX_SUCCESS);
 assert(!strcmp(name,"/UTC/DAQ_TELEMETRY_1767225600000_000.CSV"));
 assert(sd_create_log_file(name,sizeof name,"DAQ_LAUNCH",0,789)==FX_SUCCESS);
 assert(!strcmp(name,"/LAUNCH/DAQ_LAUNCH_789_000.CSV"));
 assert(sd_create_log_file(name,sizeof name,"DAQ_LAUNCH_TELEMETRY",1767225600000ULL,789)==FX_SUCCESS);
 assert(!strcmp(name,"/LAUNCH/DAQ_LAUNCH_TELEMETRY_1767225600000_000.CSV"));
 /* Invalid nonzero UTC remains explicitly NO_UTC; no bogus calendar label. */
 assert(sd_create_log_file(name,sizeof name,"DAQ",1,999)==FX_SUCCESS);
 assert(!strcmp(name,"/NO_UTC/DAQ_999_000.CSV"));
 dir_status=FX_ALREADY_CREATED;assert(sd_create_log_file(name,sizeof name,"DAQ",0,0)==FX_SUCCESS);
 unsigned before=creates;attrs=0;
 assert(sd_create_log_file(name,sizeof name,"DAQ",0,0)==FX_DIRECTORY_ERROR&&creates==before&&!name[0]);
 attrs=FX_DIRECTORY;attr_status=FX_IO_ERROR;
 assert(sd_create_log_file(name,sizeof name,"DAQ",0,0)==FX_IO_ERROR&&creates==before);
 attr_status=0;dir_status=FX_IO_ERROR;
 assert(sd_create_log_file(name,sizeof name,"DAQ",0,0)==FX_IO_ERROR&&creates==before);
 dir_status=0;collisions=1;
 assert(sd_create_log_file(name,sizeof name,"DAQ",0,123)==FX_SUCCESS);
 assert(!strcmp(name,"/NO_UTC/DAQ_123_001.CSV"));
 collisions=1000;before=creates;
 assert(sd_create_log_file(name,sizeof name,"DAQ",0,123)==FX_NO_MORE_SPACE);
 assert(creates==before+1000&&!name[0]);
 struct {char name[4];char guard[4];} tiny={{0},{1,2,3,4}};
 assert(sd_create_log_file(tiny.name,sizeof tiny.name,"DAQ",0,123)==FX_INVALID_NAME);
 assert(!tiny.name[0]&&tiny.guard[0]==1&&tiny.guard[3]==4);
 assert(sd_create_log_file(0,64,"DAQ",0,1)==FX_PTR_ERROR);
 /* No root-directory or historical-file move/delete API is used. */
 assert(dirs>0);
}
'''
        with tempfile.TemporaryDirectory() as t:
            binary=Path(t)/'paths'
            subprocess.run(['cc','-std=c11','-O1','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I',str(ROOT/'Core/Inc'),'-x','c','-','-o',str(binary)],input=code,text=True,check=True)
            subprocess.run([str(binary)],check=True)

if __name__=='__main__':unittest.main()
