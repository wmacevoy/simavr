/*
  simduino.c

  Copyright 2008, 2009 Michel Pollet <buserror@gmail.com>

  This file is part of simavr.

  simavr is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  simavr is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with simavr.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdlib.h>
#include <stdio.h>
#include <libgen.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "sim_avr.h"
#include "sim_elf.h"
#include "sim_core.h"
#include "sim_gdb.h"
#include "sim_hex.h"
#include "avr_ioport.h"
#include "uart_pty.h"

#include "sim_core_decl.h"

struct avr_flash {
  char avr_flash_path[1024];
  int avr_flash_fd;
};

const char **args = 0;
uart_pty_t uart_pty = {0};
char uart_basename[128]="/tmp/simavr-uart";
avr_t * avr = 0;
elf_firmware_t f={{0}};
uint32_t f_cpu=16*1000*1000;
int trace = 0;
int gdb = 0;
uint32_t loadBase=AVR_SEGMENT_OFFSET_FLASH;
int trace_vectors[8]={0};
int trace_vectors_count=0;
const char *vcd_input=0;
struct avr_flash flash_data={{0}};
char boot_path[1024]={0};
char mmcu[24]="atmega328p";
int debug=0;
int verbose=0;


#define OPTION(COUNT,SHORTNAME,LONGNAME,CODE) { if (op((COUNT),args+argi,(SHORTNAME),(LONGNAME))) { { CODE ; } argi += (COUNT); continue; } }
#define OP(k) (args[argi+(k)])

static int simduino_filename(const char *filename) {
  char * suffix = strrchr(filename, '.');
  if (suffix && !strcasecmp(suffix, ".hex")) {
    if (!mmcu[0] || !f_cpu) {
      fprintf(stderr, "%s: -mcu and -freq are mandatory to load .hex files\n", args[0]);
      return 1;
    }
    ihex_chunk_p chunk = NULL;
    int cnt = read_ihex_chunks(filename, &chunk);
    if (cnt <= 0) {
      fprintf(stderr, "%s: Unable to load IHEX file %s\n",
	      args[0],filename);
      return 1;
    }
    printf("Loaded %d section of ihex\n", cnt);
    for (int ci = 0; ci < cnt; ci++) {
      if (chunk[ci].baseaddr < (1*1024*1024)) {
	f.flash = chunk[ci].data;
	f.flashsize = chunk[ci].size;
	f.flashbase = chunk[ci].baseaddr;
	printf("Load HEX flash %08x, %d\n", f.flashbase, f.flashsize);
      } else if (chunk[ci].baseaddr >= AVR_SEGMENT_OFFSET_EEPROM ||
		 chunk[ci].baseaddr + loadBase >= AVR_SEGMENT_OFFSET_EEPROM) {
	// eeprom!
	f.eeprom = chunk[ci].data;
	f.eesize = chunk[ci].size;
	printf("Load HEX eeprom %08x, %d\n", chunk[ci].baseaddr, f.eesize);
      }
    }
  } else {
    if (elf_read_firmware(filename, &f) == -1) {
      fprintf(stderr, "%s: Unable to load firmware from file %s\n",
	      args[0], filename);
      return 1;
    }
  }
  return 0;
}

static void
simduino_usage() {
  char arg0[4096];
  snprintf(arg0,sizeof(arg0),"%s",args[0]);
  printf("Usage: %s [...] <firmware>\n", basename(arg0));
  printf( "    [--freq|-f <freq>]  Sets the frequency for an .hex firmware\n"
	  "    [--mcu|-m <device>] Sets the MCU type for an .hex firmware\n"
	  "    [--list-cores]      List all supported AVR cores and exit\n"
	  "    [--help|-h]         Display this usage message and exit\n"
	  "    [--trace, -t]       Run full scale decoder trace\n"
	  "    [-ti <vector>]      Add traces for IRQ vector <vector>\n"
	  "    [--gdb|-g]          Listen for gdb connection on port 1234\n"
	  "    [-ff <.hex file>]   Load next .hex file as flash\n"
	  "    [-ee <.hex file>]   Load next .hex file as eeprom\n"
	  "    [--input|-i <file>] A .vcd file to use as input signals\n"
	  "    [-v]                Raise verbosity level\n"
	  "                           (can be passed more than once)\n"
	  "    [--vcd-trace-name <file>]"
	  "    [--add-vcd-trace <name>=<kind>@<addr>/<mask>\n"		
	  "    <firmware>          A .hex or an ELF file. ELF files are\n"
	  "                           prefered, and can include debugging syms\n");
}

static void
list_cores()
{
  printf( "Supported AVR cores:\n");
  for (int i = 0; avr_kind[i]; i++) {
    printf("       ");
    for (int ti = 0; ti < 4 && avr_kind[i]->names[ti]; ti++)
      printf("%s ", avr_kind[i]->names[ti]);
    printf("\n");
  }
  exit(1);
}

static void
sig_int(
	int sign)
{
  printf("signal caught, simavr terminating\n");
  if (avr)
    avr_terminate(avr);
  exit(0);
}



// avr special flash initalization
// here: open and map a file to enable a persistent storage for the flash memory
void avr_special_init( avr_t * avr, void * data)
{
  struct avr_flash *flash_data = (struct avr_flash *)data;

  printf("%s\n", __func__);
  // open the file
  flash_data->avr_flash_fd = open(flash_data->avr_flash_path,
				  O_RDWR|O_CREAT, 0644);
  if (flash_data->avr_flash_fd < 0) {
    perror(flash_data->avr_flash_path);
    exit(1);
  }
  // resize and map the file the file
  (void)ftruncate(flash_data->avr_flash_fd, avr->flashend + 1);
  ssize_t r = read(flash_data->avr_flash_fd, avr->flash, avr->flashend + 1);
  if (r != avr->flashend + 1) {
    fprintf(stderr, "unable to load flash memory\n");
    perror(flash_data->avr_flash_path);
    exit(1);
  }
}

// avr special flash deinitalization
// here: cleanup the persistent storage
void avr_special_deinit( avr_t* avr, void * data)
{
  struct avr_flash *flash_data = (struct avr_flash *)data;

  printf("%s\n", __func__);
  lseek(flash_data->avr_flash_fd, SEEK_SET, 0);
  ssize_t r = write(flash_data->avr_flash_fd, avr->flash, avr->flashend + 1);
  if (r != avr->flashend + 1) {
    fprintf(stderr, "unable to load flash memory\n");
    perror(flash_data->avr_flash_path);
  }
  close(flash_data->avr_flash_fd);
  uart_pty_stop(&uart_pty);
}

int suffix(const char *string, const char *ending) {
  if (string == 0 || ending == 0) return 0;
  
  int endingLen = strlen(ending);
  int stringLen = strlen(string);

  if (endingLen > stringLen) return 0;

  return strcmp(string + stringLen - endingLen, ending) == 0;
}

static int
op(int count, const char **args, const char *shortname, const char *longname) {
  for (int k=0; k<=count; ++k) {
    if (args[k] == 0) return 0;
  }
  return ((shortname != 0 && strcmp(args[0],shortname)==0)
	  || (longname != 0 && strcmp(args[0],longname)==0));
}


static int simduino_add_vcd_trace(const char *arg) {
  struct {
    char     kind[64];
    uint8_t  mask;
    uint16_t addr;
    char     name[64];
  } trace;
  
  const int n_args = sscanf(arg,"%63[^=]=%63[^@]@0x%hx/0x%hhx",
			    &trace.name[0],
			    &trace.kind[0],
			    &trace.addr,
			    &trace.mask
			    );
  if (n_args != 4) {
    fprintf(stderr, "add-vcd-trace: format for %s is name=kind@addr/mask.\n", arg);
    return 1;
  }

  /****/ if (!strcmp(trace.kind, "portpin")) {
    f.trace[f.tracecount].kind = AVR_MMCU_TAG_VCD_PORTPIN;
  } else if (!strcmp(trace.kind, "irq")) {
    f.trace[f.tracecount].kind = AVR_MMCU_TAG_VCD_IRQ;
  } else if (!strcmp(trace.kind, "trace")) {
    f.trace[f.tracecount].kind = AVR_MMCU_TAG_VCD_TRACE;
  } else {
    fprintf(stderr,
	    "%s: unknown trace kind '%s', not one of 'portpin', 'irq', or 'trace'.\n",
	    "add-vcd-trace",
	    trace.kind
	    );
    return 1;
  }
  f.trace[f.tracecount].mask = trace.mask;
  f.trace[f.tracecount].addr = trace.addr;
  strncpy(f.trace[f.tracecount].name, trace.name, sizeof(f.trace[f.tracecount].name));

  printf(
	 "Adding %s trace on address 0x%04x, mask 0x%02x ('%s')\n",
	 f.trace[f.tracecount].kind == AVR_MMCU_TAG_VCD_PORTPIN ? "portpin"
	 : f.trace[f.tracecount].kind == AVR_MMCU_TAG_VCD_IRQ     ? "irq"
	 : f.trace[f.tracecount].kind == AVR_MMCU_TAG_VCD_TRACE   ? "trace"
	 : "unknown",
	 f.trace[f.tracecount].addr,
	 f.trace[f.tracecount].mask,
	 f.trace[f.tracecount].name
	 );
  
  ++f.tracecount;
  return 0;
}

static 
int simduino_init() {
  loadBase = AVR_SEGMENT_OFFSET_FLASH;
  snprintf(boot_path,sizeof(boot_path),"ATmegaBOOT_168_atmega328.ihex");
  snprintf(mmcu,sizeof(mmcu),"atmega328p");


  if (args[1] == 0) {
    simduino_usage();
    return 1;
  }

  for (int argi=1; args[argi] != 0; ++argi) {
    OPTION(0,"-l","--list-cores",list_cores());
    OPTION(0,"-h","--help",simduino_usage());
    OPTION(1,"-m","--mcu",snprintf(mmcu, sizeof(mmcu), "%s", OP(1)));
    OPTION(1,"-f","--freq",f_cpu = atoi(OP(1)));
    OPTION(1,"-i","--input",vcd_input = OP(1));
    OPTION(0,"-t","--trace",++trace);
    OPTION(1,NULL,"--vcd-trace-name",snprintf(f.tracename, sizeof(f.tracename), "%s",  OP(1)));
    OPTION(1,NULL, "--add-vcd-trace",if (!simduino_add_vcd_trace(OP(1))) return 1);
    OPTION(1,NULL, "--vcd-trace-file",snprintf(f.tracename, sizeof(f.tracename), "%s", OP(1)));
    OPTION(0,"-g","--gdb",++gdb);
    OPTION(0,"-v","--verbose",++verbose);
    OPTION(0,"-d","--debug",++debug);
    OPTION(0,"-ee","--eeprom", loadBase = AVR_SEGMENT_OFFSET_EEPROM);
    OPTION(0,"-ff","--flash",  loadBase = AVR_SEGMENT_OFFSET_FLASH);
    OPTION(1,"-u","--uart", snprintf(uart_basename, sizeof(uart_basename), "%s", OP(1)));
    if (!simduino_filename(OP(0))) {
      continue;
    }
    simduino_usage();
    return 1;
  }
  return 0;
}

void
simduino_boot() {
  avr = avr_make_mcu_by_name(mmcu);
  if (!avr) {
    fprintf(stderr, "%s: Error creating the AVR core\n", args[0]);
    exit(1);
  }

  uint8_t *boot;
  uint32_t boot_base, boot_size;

  boot = read_ihex_file(boot_path, &boot_size, &boot_base);
  if (!boot) {
    fprintf(stderr, "%s: Unable to load %s\n", args[0], boot_path);
    exit(1);
  }
  if (boot_base > 32*1024*1024) {
    snprintf(mmcu,sizeof(mmcu),"%s","atmega2560");
    f_cpu = 20000000;
  }
  printf("%s booloader 0x%05x: %d bytes\n", mmcu, boot_base, boot_size);

  snprintf(flash_data.avr_flash_path, sizeof(flash_data.avr_flash_path),
	   "simduino_%s_flash.bin", mmcu);
  flash_data.avr_flash_fd = 0;
  // register our own functions
  avr->custom.init = avr_special_init;
  avr->custom.deinit = avr_special_deinit;
  avr->custom.data = &flash_data;
  avr_init(avr);
  avr->frequency = f_cpu;
  
  memcpy(avr->flash + boot_base, boot, boot_size);
  free(boot);
  avr->pc = boot_base;
  /* end of flash, remember we are writing /code/ */
  avr->codeend = avr->flashend;
  avr->log = 1 + verbose;
  
  // even if not setup at startup, activate gdb if crashing
  avr->gdb_port = 1234;
  if (debug) {
    avr->state = cpu_Stopped;
    avr_gdb_init(avr);
  }
  signal(SIGINT, sig_int);
  signal(SIGTERM, sig_int);
  
  uart_pty_init(avr, &uart_pty);
  uart_pty_connect_basename(&uart_pty, '0', uart_basename);
}
	

int main(int argc, const char *argv[])
{
  args = argv;
  simduino_init();
  simduino_boot();
  while (1) {
    int state = avr_run(avr);
    if ( state == cpu_Done || state == cpu_Crashed)
      break;
  }
  return 0;
}
