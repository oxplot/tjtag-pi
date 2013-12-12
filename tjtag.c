// vim: ts=3:sw=3:expandtab:sts=3
//
// tjtag.c - EJTAG Debrick Utility v3.0.1 - Tornado MOD
// Copyright (C) 2004  HairyDairyMaid (a.k.a. Lightbulb)
//                     <hairydairymaid@yahoo.com>
// Copyright (C) 2004  William "Bill" Henderson (a.k.a. Tornado)
//                     <tornado@tjtag.com> <tornado@odessaua.com>
//                     <tornado@dd-wrt.com>
// Copyright (C) 2013  Mansour Behabadi <mansour@oxplot.com>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

// Default is Compile for Linux (both #define's below should be commented out)
//#define WINDOWS_VERSION   // uncomment only this for Windows Compile / MS Visual C Compiler
//#define __FreeBSD__       // uncomment only this for FreeBSD

#ifdef WINDOWS_VERSION
#include <windows.h>      // Only for Windows Compile
#define strcasecmp  stricmp
#define strncasecmp strnicmp
#include <conio.h>
#define _CRT_SECURE_NO_WARNINGS
#endif

#ifdef WINDOWS_VERSION
#define tnano(seconds) Sleep((seconds) / 1000000)
#define tmicro(seconds) Sleep((seconds) * 1000)
/* Windows sleep is milliseconds, time/1000000 gives us nanoseconds */
#else
//ulseep is in microseconds
#define tnano(seconds) sleep((seconds) / 1000000000)
#define tmicro(seconds) usleep(seconds)
#endif

#include <inttypes.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <assert.h>

#include "tjtag.h"
#include "spi.h"

#define TRUE  1
#define FALSE 0

#ifdef RASPPI
   // I/O access
   volatile unsigned *gpio;

   #define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
   #define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
   #define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

   #define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
   #define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0
   #define GPIO_GET *(gpio+0xd) // get bits
#endif

static unsigned int ctrl_reg;
volatile unsigned int dcounter;

int pfd;
int instruction_length;
int issue_reset      = 1;
int issue_enable_mw  = 1;
int issue_watchdog   = 1;
int issue_break      = 1;
int issue_erase      = 1;
int issue_timestamp  = 1;
int issue_reboot     = 0;
int force_dma        = 0;
int force_nodma      = 0;
int selected_fc      = 0;
unsigned int selected_window  = 0;
unsigned int selected_start   = 0;
unsigned int selected_length  = 0;
int custom_options   = 0;
int silent_mode      = 0;
int skipdetect       = 0;
int instrlen         = 0;
int wiggler          = 0;
int speedtouch       = 0;
int DEBUG            = 0;
int Flash_DEBUG      = 0;
int probe_options    = 0;


unsigned int    flash_size     = 0;
int             block_total    = 0;
unsigned int    block_addr     = 0;
unsigned int    cmd_type       = 0;
int             ejtag_version  = 0;
int             bypass         = 0;
int             USE_DMA        = 0;


char            flash_part[128];
unsigned int    blocks[1024];

char            AREA_NAME[128];
unsigned int    AREA_START;
unsigned int    AREA_LENGTH;
unsigned int    FLASH_MEMORY_START;
unsigned int    vendid;
unsigned int    devid;

unsigned int    data_register;
unsigned int    address_register;
unsigned int    proc_id;
unsigned int    xbit = 0;
unsigned int    delay = 0;
unsigned int    bcmproc = 0;
unsigned int    swap_endian=0;
unsigned int    bigendian=0;


unsigned int spi_flash_read;
unsigned int spi_flash_mmr;
unsigned int spi_flash_mmr_size;
unsigned int spi_flash_ctl;
unsigned int spi_flash_opcode;
unsigned int spi_flash_data;
unsigned int spi_ctl_start;
unsigned int spi_ctl_busy;

#define	CID_ID_MASK		0x0000ffff

struct STAT_REG_BITS p;

typedef struct _processor_chip_type
{
    unsigned int        chip_id;        // Processor Chip ID
    int                 instr_length;   // EJTAG Instruction Length
    char*               chip_descr;     // Processor Chip Description
} processor_chip_type;

processor_chip_type  processor_chip_list[] =
{
    { 0x0471017F, 5, "Broadcom BCM4702 Rev 1 CPU" },
    { 0x9470417F, 8, "Broadcom BCM4704 KPBG Rev 9 CPU"},      // Tornado  WRT150N
    { 0x0470417F, 8, "Broadcom BCM4704 Rev 8 CPU" },          // BCM4704 chip (used in the WRTSL54GS units)
    { 0x1471217F, 8, "Broadcom BCM4712 Rev 1 CPU" },
    { 0x2471217F, 8, "Broadcom BCM4712 Rev 2 CPU" },
    { 0x1471617F, 8, "Broadcom BCM4716 Rev 1 CPU" },          // Eko BCM4718A1KFBG
    { 0x0478517F, 8, "Broadcom BCM4785 Rev 1 CPU" },          // Tornado WRT350N
    { 0x0535017F, 8, "Broadcom BCM5350 Rev 1 CPU" },
    { 0x0535217F, 8, "Broadcom BCM5352 Rev 1 CPU" },
    { 0x1535417F, 8, "Broadcom BCM5354 KFBG Rev 1 CPU" },     // Tornado - WRT54G GV8/GSV7
    { 0x2535417F, 8, "Broadcom BCM5354 KFBG Rev 2 CPU" },     // Tornado - Gv8/GSv7
    { 0x3535417F, 8, "Broadcom BCM5354 KFBG Rev 3 CPU" },     // Tornado - WRG54G2
    { 0x0334517F, 5, "Broadcom BCM3345 KPB Rev 1 CPU" },      // Eko QAMLink  BCM3345 KPB SB4200
    { 0x0536517F, 8, "Broadcom BCM5365 Rev 1 CPU" },          // BCM5365 Not Completely Verified Yet
    { 0x1536517F, 8, "Broadcom BCM5365 Rev 1 CPU" },          // Eko....ASUS WL 500 G Deluxe
    { 0x0634517F, 5, "Broadcom BCM6345 Rev 1 CPU" },          // BCM6345 Not Completely Verified Yet
    { 0x0634817F, 5, "Broadcom BCM6348 Rev 1 CPU" },
    { 0x0633817F, 5, "Broadcom BCM6338 Rev 1 CPU" },		  // Speedtouch
    { 0x0635817F, 5, "Broadcom BCM6358 Rev 1 CPU" },          // brjtag Fully Tested
    { 0x0636817F, 5, "Broadcom BCM6368 Rev 1 CPU" },          // brjtag
    { 0x1432117F, 5, "Broadcom BCM4321 RADIO STOP" },         // Radio JP3 on a WRT300N V1.1
    { 0x3432117F, 5, "Broadcom BCM4321L RADIO STOP"},         // EKO Radio on WRT300n
    { 0x0000100F, 5, "TI AR7 TNETD7x00 Rev 1 CPU" },    // Verified for Linksys ADSL2MUE, same code for 7200 and 7300 chips
    { 0x102002E1, 5, "BRECIS MSP2007-CA-A1 CPU" },            // BRECIS chip - Not Completely Verified Yet
    { 0x0B52D02F, 5, "TI TNETV1060GDW CPU" },                 // Fox WRTP54G
    { 0x00217067, 5, "Linkstation 2 with RISC K4C chip" },    // Not verified
    { 0x00000001, 5, "Atheros AR531X/231X CPU" },             // WHR-HP-AG108
    { 0x19277013, 7, "XScale IXP42X 266mhz" },                // GW2348-2 Eko Gateworks Avila GW234X (IXP42X 266MHz) BE
    { 0x19275013, 7, "XScale IXP42X 400mhz" },
    { 0x19274013, 7, "XScale IXP42X 533mhz" },
    { 0x10940027, 4, "ARM 940T"}, // Eko  Linksys BEFSX41
    { 0x07926041, 4, "Marvell Feroceon 88F5181" },
    { 0x1438000D, 5, "LX4380"},
    {
        0, 0, 0
    }
};


typedef struct _flash_area_type
{
    unsigned int        chip_size;
    char*               area_name;
    unsigned int        area_start;
    unsigned int        area_length;
} flash_area_type;


flash_area_type  flash_area_list[] =
{
    //---------   ----------     -----------  ------------
    //chip_size   area_name      area_start   area_length
    //---------   ----------     -----------  ------------
    { size1MB,    "CFE",         0x1FC00000,  0x40000 },
    { size2MB,    "CFE",         0x1FC00000,  0x40000 },
    { size4MB,    "CFE",         0x1FC00000,  0x40000 },//256Kb
    { size8MB,    "CFE",         0x1C000000,  0x40000 },
    { size16MB,   "CFE",         0x1F000000,  0x40000 }, //tornado - for alice

    { size8MB,    "AR-CFE",         0xA8000000,  0x40000 },
    { size16MB,   "AR-CFE",         0xA8000000,  0x40000 },


    { size1MB,    "CFE128",      0x1FC00000,  0x20000 },
    { size2MB,    "CFE128",      0x1FC00000,  0x20000 },
    { size4MB,    "CFE128",      0x1FC00000,  0x20000 },//128Kb
    { size8MB,    "CFE128",      0x1C000000,  0x20000 },
    { size16MB,   "CFE128",      0x1C000000,  0x20000 },

    { size1MB,    "CF1",         0x1FC00000,  0x2000 },
    { size2MB,    "CF1",         0x1FC00000,  0x2000 },
    { size4MB,    "CF1",         0x1FC00000,  0x2000 },//8Kb
    { size8MB,    "CF1",         0x1C000000,  0x2000 },
    { size16MB,   "CF1",         0x1C000000,  0x2000 },

    { size1MB,    "KERNEL",      0x1FC40000,  0xB0000  },
    { size2MB,    "KERNEL",      0x1FC40000,  0x1B0000 },
    { size4MB,    "KERNEL",      0x1FC40000,  0x3B0000 },//3776Kb
    { size8MB,    "KERNEL",      0x1C040000,  0x7A0000 },
    { size16MB,   "KERNEL",      0x1C040000,  0x7A0000 },
    { size8MB,    "AR-KERNEL",   0xA8040000,  0x7A0000 },
    { size16MB,   "AR-KERNEL",   0xA8040000,  0x7A0000 },

    { size1MB,    "NVRAM",       0x1FCF0000,  0x10000 },
    { size2MB,    "NVRAM",       0x1FDF0000,  0x10000 },
    { size4MB,    "NVRAM",       0x1FFF0000,  0x10000 },//64kb
    { size8MB,    "NVRAM",       0x1C7E0000,  0x20000 },
    { size16MB,   "NVRAM",       0x1C7E0000,  0x20000 },

    { size8MB,    "AR-NVRAM",    0xA87E0000,  0x20000 },
    { size16MB,   "AR-NVRAM",    0xA87E0000,  0x20000 },

    { size2MB,    "WGRV9NVRAM",  0x1FDFC000,  0x4000 },
    { size2MB,    "WGRV9BDATA",  0x1FDFB000,  0x1000 },
    { size4MB,    "WGRV8BDATA",  0x1FFE0000,  0x10000 },//64kb

    { size1MB,    "WHOLEFLASH",  0x1FC00000,  0x100000 },
    { size2MB,    "WHOLEFLASH",  0x1FC00000,  0x200000 },
    { size4MB,    "WHOLEFLASH",  0x1FC00000,  0x400000 },//4Mb
    { size8MB,    "WHOLEFLASH",  0x1C000000,  0x800000 },
//    { size16MB,   "WHOLEFLASH",  0x1C000000,  0x1000000 },
    { size16MB,   "WHOLEFLASH",  0x1F000000,  0x1000000 },
    { size8MB,    "AR-WHOLEFLASH",  0xA8000000,  0x800000 },
    { size16MB,   "AR-WHOLEFLASH",  0xA8000000,  0x1000000 },

    { size1MB,    "BSP",         0x1FC00000,  0x50000 },
    { size2MB,    "BSP",         0x1FC00000,  0x50000 },
    { size4MB,    "BSP",         0x1FC00000,  0x50000 },
    { size8MB,    "BSP",         0x1C000000,  0x50000 },
    { size16MB,   "BSP",         0x1C000000,  0x50000 },

    { size8MB,    "AR-BSP",         0xA8000000,  0x50000 },
    { size16MB,   "AR-BSP",         0xA8000000,  0x50000 },

    { size1MB,    "RED",         0x50000000,  0x50000 },
    { size2MB,    "RED",         0x50000000,  0x50000 },
    { size4MB,    "RED",         0x50000000,  0x50000 },
    { size8MB,    "AR-RED",      0xA8000000,  0x30000 },
    { size8MB,    "RED",         0x50000000,  0x50000 },
    { size16MB,   "RED",         0x50000000,  0x50000 },

/* extras for Ti AR7 */    
   { size1MB,    "MTD2",        0x90000000,  0x10000 },
   { size2MB,    "MTD2",        0x90000000,  0x10000 },
   { size4MB,    "MTD2",        0x90000000,  0x10000 },

   { size1MB,    "MTD3",        0x90010000,  0x10000 },// only for pspboot, its at the other end for adam2
   { size2MB,    "MTD3",        0x90010000,  0x10000 },
   { size4MB,    "MTD3",        0x90010000,  0x10000 },
   
   { size1MB,    "MTD4",        0x90020000,  0xE0000 },
   { size2MB,    "MTD4",        0x90020000,  0x1E0000 },
   { size4MB,    "MTD4",        0x90020000,  0x3E0000 },

   { size1MB,    "FULL",        0x90020000,  0x100000 },
   { size2MB,    "FULL",        0x90020000,  0x200000 },
   { size4MB,    "FULL",        0x90020000,  0x400000 },

    { 0, 0, 0, 0 }
};


typedef struct _flash_chip_type
{
    unsigned int        vendid;         // Manufacturer Id
    unsigned int        devid;          // Device Id
    unsigned int        flash_size;     // Total size in MBytes
    unsigned int        cmd_type;       // Device CMD TYPE
    char*               flash_part;     // Flash Chip Description
    unsigned int        region1_num;    // Region 1 block count
    unsigned int        region1_size;   // Region 1 block size
    unsigned int        region2_num;    // Region 2 block count
    unsigned int        region2_size;   // Region 2 block size
    unsigned int        region3_num;    // Region 3 block count
    unsigned int        region3_size;   // Region 3 block size
    unsigned int        region4_num;    // Region 4 block count
    unsigned int        region4_size;   // Region 4 block size
} flash_chip_type;


flash_chip_type  flash_chip_list[] =
{
    /* AMD, Spansion */
    { 0x00C2, 0x22DA, size1MB, CMD_TYPE_AMD, "MX29LV800BTC 512kx16 TopB  (1MB)"   ,15,size32K,    1,size16K,    2,size4K,   1,size8K  },
    { 0x00C2, 0x225B, size1MB, CMD_TYPE_AMD, "MX29LV800BTC 512kx16 BotB  (1MB)"   ,1,size8K,      2,size4K,     1,size16K,  15,size32K },

    { 0x0001, 0x2249, size2MB, CMD_TYPE_AMD, "AMD 29lv160DB 1Mx16 BotB   (2MB)"   ,1,size16K,    2,size8K,     1,size32K,  31,size64K }, /* bypass */
    { 0x0001, 0x22c4, size2MB, CMD_TYPE_AMD, "AMD 29lv160DT 1Mx16 TopB   (2MB)"   ,31,size64K,   1,size32K,    2,size8K,   1,size16K  },
    { 0x007F, 0x2249, size2MB, CMD_TYPE_AMD, "EON EN29LV160A 1Mx16 BotB  (2MB)"   ,1,size16K,    2,size8K,     1,size32K,  31,size64K }, /* bypass */
    { 0x007F, 0x22C4, size2MB, CMD_TYPE_AMD, "EON EN29LV160A 1Mx16 TopB  (2MB)"   ,31,size64K,   1,size32K,    2,size8K,   1,size16K  },
    { 0x0004, 0x2249, size2MB, CMD_TYPE_AMD, "MBM29LV160B 1Mx16 BotB     (2MB)"   ,1,size16K,    2,size8K,     1,size32K,  31,size64K },
    { 0x0004, 0x22c4, size2MB, CMD_TYPE_AMD, "MBM29LV160T 1Mx16 TopB     (2MB)"   ,31,size64K,   1,size32K,    2,size8K,   1,size16K  },
    { 0x00C2, 0x2249, size2MB, CMD_TYPE_AMD, "MX29LV160CB 1Mx16 BotB     (2MB)"   ,1,size16K,    2,size8K,     1,size32K,  31,size64K },
    { 0x00C2, 0x22c4, size2MB, CMD_TYPE_AMD, "MX29LV160CT 1Mx16 TopB     (2MB)"   ,31,size64K,   1,size32K,    2,size8K,   1,size16K  },
    { 0x00EC, 0x2275, size2MB, CMD_TYPE_AMD, "K8D1716UTC  1Mx16 TopB     (2MB)"   ,31,size64K,    8,size8K,     0,0,        0,0        },
    { 0x00EC, 0x2277, size2MB, CMD_TYPE_AMD, "K8D1716UBC  1Mx16 BotB     (2MB)"   ,8,size8K,      31,size64K,   0,0,        0,0        }, /* bypass */
    { 0x0020, 0x2249, size2MB, CMD_TYPE_AMD, "ST M29W160EB 1Mx16 BotB    (2MB)"   ,1,size16K,    2,size8K,     1,size32K,  31,size64K },
    { 0x0020, 0x22c4, size2MB, CMD_TYPE_AMD, "ST M29W160ET 1Mx16 TopB    (2MB)"   ,31,size64K,   1,size32K,    2,size8K,   1,size16K  },
    { 0x00C2, 0x0014, size2MB, CMD_TYPE_SPI, "Macronix MX25L160A         (2MB) Serial"   ,32,size64K,   0,0,          0,0,        0,0        }, /* new */
    { 0x001f, 0x2600, size2MB, CMD_TYPE_SPI, "Atmel AT45DB161B           (2MB) Serial"   ,512,sizeA4K,   0,0,          0,0,        0,0        }, /* new */
    { 0x0040, 0x0000, size2MB, CMD_TYPE_SPI, "Atmel AT45DB161B           (2MB) Serial"   ,512,sizeA4K,   0,0,          0,0,        0,0        }, /* new */

    { 0x00EC, 0x22A0, size4MB, CMD_TYPE_AMD, "K8D3216UTC  2Mx16 TopB     (4MB)"   ,63,size64K,    8,size8K,     0,0,        0,0        },
    { 0x00EC, 0x22A2, size4MB, CMD_TYPE_AMD, "K8D3216UBC  2Mx16 BotB     (4MB)"   ,8,size8K,      63,size64K,   0,0,        0,0        },

    { 0x00C2, 0x2015, size2MB, CMD_TYPE_SPI, "Macronix MX25L1605D        (2MB) Serial"   ,32,size64K,    0,0,          0,0,        0,0 }, /* new */
    { 0x00C2, 0x2016, size4MB, CMD_TYPE_SPI, "Macronix MX25L3205D        (4MB) Serial"   ,64,size64K,    0,0,          0,0,        0,0 }, /* new */
    { 0x00C2, 0x2017, size8MB, CMD_TYPE_SPI, "Macronix MX25L6405D        (8MB) Serial"   ,128,size64K,   0,0,          0,0,        0,0 }, /* new */


    { 0x0020, 0x2015, size2MB, CMD_TYPE_SPI, "STMicro M25P16             (2MB) Serial"   ,32,size64K,   0,0, 0,0, 0,0 }, /* new */
    { 0x0020, 0x2016, size4MB, CMD_TYPE_SPI, "STMicro M25P32             (4MB) Serial"   ,64,size64K,   0,0, 0,0, 0,0 }, /* new */
    { 0x0020, 0x2017, size8MB, CMD_TYPE_SPI, "STMicro M25P64             (8MB) Serial"   ,128,size64K,  0,0, 0,0, 0,0 }, /* new */
    { 0x0020, 0x2018, size16MB, CMD_TYPE_SPI, "STMicro M25P128           (16MB) Serial"  ,32,size256K,  0,0, 0,0, 0,0 }, /* new */


    { 0x0001, 0x2200, size4MB, CMD_TYPE_AMD, "AMD 29lv320MB 2Mx16 BotB   (4MB)"   ,8,size8K,     63,size64K,   0,0,        0,0        },
    { 0x0001, 0x227E, size4MB, CMD_TYPE_AMD, "AMD 29lv320MT 2Mx16 TopB   (4MB)"   ,63,size64K,   8,size8K,     0,0,        0,0        },
    { 0x0001, 0x2201, size4MB, CMD_TYPE_AMD, "AMD 29lv320MT 2Mx16 TopB   (4MB)"   ,63,size64K,   8,size8K,     0,0,        0,0        },
    { 0x0098, 0x009C, size4MB, CMD_TYPE_AMD, "TC58FVB321 2Mx16 BotB      (4MB)"   ,1,size16K,    2,size8K,     1,size32K,  63,size64K },
    { 0x0098, 0x009A, size4MB, CMD_TYPE_AMD, "TC58FVT321 2Mx16 TopB      (4MB)"   ,63,size64K,   1,size32K,    2,size8K,   1,size16K  },
    { 0x001F, 0x00C0, size4MB, CMD_TYPE_AMD, "AT49BV/LV16X 2Mx16 BotB    (4MB)"   ,8,size8K,     63,size64K,   0,0,        0,0        },
    { 0x001F, 0x00C2, size4MB, CMD_TYPE_AMD, "AT49BV/LV16XT 2Mx16 TopB   (4MB)"   ,63,size64K,   8,size8K,     0,0,        0,0        },
    { 0x0004, 0x2253, size4MB, CMD_TYPE_AMD, "MBM29DL323BE 2Mx16 BotB    (4MB)"   ,8,size8K,     63,size64K,   0,0,        0,0        },
    { 0x0004, 0x2250, size4MB, CMD_TYPE_AMD, "MBM29DL323TE 2Mx16 TopB    (4MB)"   ,63,size64K,   8,size8K,     0,0,        0,0        },
    { 0x0001, 0x22f9, size4MB, CMD_TYPE_AMD, "AMD 29lv320DB 2Mx16 BotB   (4MB)"   ,8,size8K,     63,size64K,   0,0,        0,0        },
    { 0x0001, 0x22f6, size4MB, CMD_TYPE_AMD, "AMD 29lv320DT 2Mx16 TopB   (4MB)"   ,63,size64K,   8,size8K,     0,0,        0,0        },
    { 0x0004, 0x22F9, size4MB, CMD_TYPE_AMD, "MBM29LV320BE 2Mx16 BotB    (4MB)"   ,1,size16K,    2,size8K,     1,size32K,  63,size64K },
    { 0x0004, 0x22F6, size4MB, CMD_TYPE_AMD, "MBM29LV320TE 2Mx16 TopB    (4MB)"   ,63,size64K,   1,size32K,    2,size8K,   1,size16K  },
    { 0x00C2, 0x22A8, size4MB, CMD_TYPE_AMD, "MX29LV320B 2Mx16 BotB      (4MB)"   ,8,size8K,     63,size64K,   0,0,        0,0        },
    { 0x00C2, 0x00A8, size4MB, CMD_TYPE_AMD, "MX29LV320B 2Mx16 BotB      (4MB)"   ,8,size8K,     63,size64K,   0,0,        0,0        },
    { 0x00C2, 0x00A7, size4MB, CMD_TYPE_AMD, "MX29LV320T 2Mx16 TopB      (4MB)"   ,63,size64K,   8,size8K,     0,0,        0,0        },
    { 0x00C2, 0x22A7, size4MB, CMD_TYPE_AMD, "MX29LV320T 2Mx16 TopB      (4MB)"   ,63,size64K,   8,size8K,     0,0,        0,0        },
    { 0x0020, 0x22CB, size4MB, CMD_TYPE_AMD, "ST 29w320DB 2Mx16 BotB     (4MB)"   ,1,size16K,    2,size8K,     1,size32K,  63,size64K },
    { 0x0020, 0x22CA, size4MB, CMD_TYPE_AMD, "ST 29w320DT 2Mx16 TopB     (4MB)"   ,63,size64K,   1,size32K,    2,size8K,   1,size16K  },

    { 0x00C2, 0x22C9, size16MB, CMD_TYPE_AMD, "MX29LV640B 4Mx16 TopB     (16MB)"  ,127,size64K,  8,size8K,      0,0,        0,0        },
    { 0x00C2, 0x22CB, size16MB, CMD_TYPE_AMD, "MX29LV640B 4Mx16 BotB     (16MB)"  ,8,size8K,     127,size64K,   0,0,        0,0        },


    { 0x00DA, 0x22BA, size4MB, CMD_TYPE_AMD, "W19B(L)320ST   2Mx16 TopB  (4MB)"   ,63,size64K,   8,size8K,     0,0,        0,0        }, /* new */
    { 0x00DA, 0x222A, size4MB, CMD_TYPE_AMD, "W19B(L)320SB   2Mx16 BotB  (4MB)"   ,8,size8K,   63,size64K,     0,0,        0,0        }, /* new */
    { 0x22DA, 0x222A, size4MB, CMD_TYPE_AMD, "W19B(L)320SB   2Mx16 BotB  (4MB)"   ,8,size8K,   63,size64K,     0,0,        0,0        },

    { 0x0020, 0x225C, size4MB, CMD_TYPE_AMD, "M29DW324DT 2Mx16 TopB      (4MB)"   ,63,size64K,   8,size8K,     0,0,        0,0        }, /* new */
    { 0x0020, 0x225D, size4MB, CMD_TYPE_AMD, "M29DW324DB 2Mx16 BotB      (4MB)"   ,8,size8K,   63,size64K,     0,0,        0,0        }, /* new */

    { 0x0098, 0x0057, size8MB, CMD_TYPE_AMD, "TC58FVM6T2A  4Mx16 TopB    (8MB)"   ,127,size64K,   8,size8K,     0,0,        0,0       }, /* new */
    { 0x0098, 0x0058, size8MB, CMD_TYPE_AMD, "TC58FVM6B2A  4Mx16 BopB    (8MB)"   ,8,size8K,   127,size64K,     0,0,        0,0       }, /* new */

    { 0x00EC, 0x22E0, size8MB, CMD_TYPE_AMD, "K8D6316UTM  4Mx16 TopB     (8MB)"   ,127,size64K,    8,size8K,     0,0,        0,0        }, /* new */
    { 0x00EC, 0x22E2, size8MB, CMD_TYPE_AMD, "K8D6316UBM  4Mx16 BotB     (8MB)"   ,8,size8K,      127,size64K,   0,0,        0,0        }, /* new */

    /* BSC */
    { 0x0089, 0x8891, size2MB, CMD_TYPE_BSC, "Intel 28F160B3 1Mx16 BotB  (2MB)"   ,8,size8K,     31,size64K,   0,0,        0,0        },
    { 0x0089, 0x8890, size2MB, CMD_TYPE_BSC, "Intel 28F160B3 1Mx16 TopB  (2MB)"   ,31,size64K,   8,size8K,     0,0,        0,0        },
    { 0x0089, 0x88C3, size2MB, CMD_TYPE_BSC, "Intel 28F160C3 1Mx16 BotB  (2MB)"   ,8,size8K,     31,size64K,   0,0,        0,0        },
    { 0x0089, 0x88C2, size2MB, CMD_TYPE_BSC, "Intel 28F160C3 1Mx16 TopB  (2MB)"   ,31,size64K,   8,size8K,     0,0,        0,0        },

    { 0x0089, 0x8897, size4MB, CMD_TYPE_BSC, "Intel 28F320B3 2Mx16 BotB  (4MB)"   ,8,size8K,     63,size64K,   0,0,        0,0        },
    { 0x0089, 0x8896, size4MB, CMD_TYPE_BSC, "Intel 28F320B3 2Mx16 TopB  (4MB)"   ,63,size64K,   8,size8K,     0,0,        0,0        },
    { 0x0089, 0x88C5, size4MB, CMD_TYPE_BSC, "Intel 28F320C3 2Mx16 BotB  (4MB)"   ,8,size8K,     63,size64K,   0,0,        0,0        },
    { 0x0089, 0x88C4, size4MB, CMD_TYPE_BSC, "Intel 28F320C3 2Mx16 TopB  (4MB)"   ,63,size64K,   8,size8K,     0,0,        0,0        },
    { 0x00b0, 0x00e3, size4MB, CMD_TYPE_BSC, "Sharp 28F320BJE 2Mx16 BotB (4MB)"   ,8,size8K,     63,size64K,   0,0,        0,0        },

    { 0x0089, 0x8899, size8MB, CMD_TYPE_BSC, "Intel 28F640B3 4Mx16 BotB  (8MB)"   ,8,size8K,     127,size64K,  0,0,        0,0        },
    { 0x0089, 0x8898, size8MB, CMD_TYPE_BSC, "Intel 28F640B3 4Mx16 TopB  (8MB)"   ,127,size64K,  8,size8K,     0,0,        0,0        },
    { 0x0089, 0x88CD, size8MB, CMD_TYPE_BSC, "Intel 28F640C3 4Mx16 BotB  (8MB)"   ,8,size8K,     127,size64K,  0,0,        0,0        },
    { 0x0089, 0x88CC, size8MB, CMD_TYPE_BSC, "Intel 28F640C3 4Mx16 TopB  (8MB)"   ,127,size64K,  8,size8K,     0,0,        0,0        },

    /* SCS */
    { 0x00b0, 0x00d0, size2MB, CMD_TYPE_SCS, "Intel 28F160S3/5 1Mx16     (2MB)"   ,32,size64K,   0,0,          0,0,        0,0        },

    { 0x0089, 0x0016, size4MB, CMD_TYPE_SCS, "Intel 28F320J3 2Mx16       (4MB)"   ,32,size128K,  0,0,          0,0,        0,0        },
    { 0x0089, 0x0014, size4MB, CMD_TYPE_SCS, "Intel 28F320J5 2Mx16       (4MB)"   ,32,size128K,  0,0,          0,0,        0,0        },
    { 0x00b0, 0x00d4, size4MB, CMD_TYPE_SCS, "Intel 28F320S3/5 2Mx16     (4MB)"   ,64,size64K,   0,0,          0,0,        0,0        },

    { 0x0089, 0x0017, size8MB, CMD_TYPE_SCS, "Intel 28F640J3 4Mx16       (8MB)"   ,64,size128K,  0,0,          0,0,        0,0        },
    { 0x0089, 0x0015, size8MB, CMD_TYPE_SCS, "Intel 28F640J5 4Mx16       (8MB)"   ,64,size128K,  0,0,          0,0,        0,0        },

    { 0x0089, 0x0018, size16MB, CMD_TYPE_SCS, "Intel 28F128J3 8Mx16      (16MB)"  ,128,size128K, 0,0,          0,0,        0,0        },

    /* SST */

    { 0x00BF, 0x234B, size2MB, CMD_TYPE_SST, "SST39VF1601 1Mx16 BotB     (2MB)"   ,8,size8K,    31,size64K,          0,0,        0,0        },
    { 0x00BF, 0x234A, size2MB, CMD_TYPE_SST, "SST39VF1602 1Mx16 TopB     (2MB)"   ,31,size64K,    8,size8K,          0,0,        0,0        },

    { 0x00BF, 0x235B, size4MB, CMD_TYPE_SST, "SST39VF3201 2Mx16 BotB     (4MB)"   ,8,size8K,   63,size64K,          0,0,        0,0        },
    { 0x00BF, 0x235A, size4MB, CMD_TYPE_SST, "SST39VF3202 2Mx16 TopB     (4MB)"   ,63,size64K,   8,size8K,          0,0,        0,0        },

    { 0x00BF, 0x236B, size8MB, CMD_TYPE_SST, "SST39VF6401 4Mx16 BotB     (8MB)"   ,8,size8K,   127,size64K,          0,0,        0,0        },
    { 0x00BF, 0x236A, size8MB, CMD_TYPE_SST, "SST39VF6402 4Mx16 TopB     (8MB)"   ,127,size64K,   8,size8K,          0,0,        0,0        },
    { 0x00BF, 0x236D, size8MB, CMD_TYPE_SST, "SST39VF6401B 4Mx16 BotB    (8MB)"   ,8,size8K,   127,size64K,          0,0,        0,0        },
    { 0x00BF, 0x236C, size8MB, CMD_TYPE_SST, "SST39VF6402B 4Mx16 TopB    (8MB)"   ,127,size64K,   8,size8K,          0,0,        0,0        },

//  See Spansion hack details for reasoning for the unusual vendid
    { 0x017E, 0x1A00, size4MB, CMD_TYPE_AMD, "Spansion S29GL032M BotB    (4MB)"   ,8,size8K,     63,size64K,   0,0,        0,0        },
    { 0x017E, 0x1A01, size4MB, CMD_TYPE_AMD, "Spansion S29GL032M TopB    (4MB)"   ,63,size64K,     8,size8K,   0,0,        0,0        },
    { 0x017E, 0x1000, size8MB, CMD_TYPE_AMD, "Spansion S29GL064M BotB    (8MB)"   ,8,size8K,     127,size64K,   0,0,        0,0        },
    { 0x017E, 0x1001, size8MB, CMD_TYPE_AMD, "Spansion S29GL064M TopB    (8MB)"   ,127,size64K,     8,size8K,   0,0,        0,0        },

// patch from OpenWRT jal2 for ti-ar7 ip8100
	{ 0x017E, 0x1301, size8MB, CMD_TYPE_AMD, "Spansion S29GL064M U       (8MB)"   ,128,size64K,     0,0,   0,0,        0,0        },

    { 0x017E, 0x2101, size16MB, CMD_TYPE_AMD, "Spansion S29GL128P U      (16MB)"   ,128,size128K,     0,0,   0,0,        0,0        },
    { 0x017E, 0x1200, size16MB, CMD_TYPE_AMD, "Spansion S29GL128M U      (16MB)"   ,128,size128K,   0,0,      0,0,        0,0         },
    { 0x017E, 0x2201, size32MB, CMD_TYPE_AMD, "Spansion S29GL256P U      (32MB)"   ,256,size128K,     0,0,   0,0,        0,0        },
    { 0x017E, 0x2301, size64MB, CMD_TYPE_AMD, "Spansion S29GL512P U      (64MB)"   ,512,size128K,     0,0,   0,0,        0,0        },
    { 0x017E, 0x2801, size128MB, CMD_TYPE_AMD, "Spansion S29GL01GP U     (128MB)"   ,1024,size128K,     0,0,   0,0,        0,0        },

    { 0x0001, 0x0214, size2MB, CMD_TYPE_SPI, "Spansion S25FL016A         (2MB) Serial"   ,32,size64K,   0,0,          0,0,        0,0        }, /* new */
    { 0x0001, 0x0215, size4MB, CMD_TYPE_SPI, "Spansion S25FL032A         (4MB) Serial"   ,64,size64K,   0,0,          0,0,        0,0        }, /* new */
    { 0x0001, 0x0216, size8MB, CMD_TYPE_SPI, "Spansion S25FL064A         (8MB) Serial"   ,128,size64K,  0,0,          0,0,        0,0        }, /* new */


// Winbond 3-stage ID chips
    { 0xDA7E, 0x0A00, size4MB, CMD_TYPE_AMD, "Winbond W19B320AB BotB     (4MB)"   ,8,size8K,     63,size64K,   0,0,        0,0        },
    { 0xDA7E, 0x0A01, size4MB, CMD_TYPE_AMD, "Winbond W19B320AT TopB     (4MB)"   ,63,size64K,     8,size8K,   0,0,        0,0        },
    { 0x00EF, 0x3016, size4MB, CMD_TYPE_SPI, "Winbond W25X32             (4MB) Serial"   ,64,size64K,   0,0,          0,0,        0,0        }, /* new */
    { 0x00EF, 0x3017, size8MB, CMD_TYPE_SPI, "Winbond W25X64             (8MB) Serial"   ,128,size64K,   0,0,          0,0,        0,0        }, /* new */
// EON
    { 0x007f, 0x22F9, size4MB, CMD_TYPE_AMD, "EON EN29LV320 2Mx16 BotB   (4MB)"   ,8,size8K,    63,size64K,     0,0,  0,0 }, /* wrt54gl v1.1 */
    { 0x007f, 0x22F6, size4MB, CMD_TYPE_AMD, "EON EN29LV320 2Mx16 TopB   (4MB)"   ,63,size64K,  8,size8K,    0,0,   0,0  }, /* bypass */
    { 0x007F, 0x22C9, size8MB, CMD_TYPE_AMD, "EON EN29LV640 4Mx16 TopB   (8MB)"   ,127,size64K, 8,size8K,   0,0,        0,0        }, /* bypass */
    { 0x007F, 0x22Cb, size8MB, CMD_TYPE_AMD, "EON EN29LV640 4Mx16 BotB   (8MB)"   ,8,size8K,    127,size64K,   0,0,        0,0        }, /* bypass */
// Atmel
    { 0x001F, 0x00C8, size4MB, CMD_TYPE_AMD, "AT49BV322A 2Mx16 BotB      (4MB)"   ,8,size8K,     63,size64K,   0,0,        0,0        },
    { 0x001F, 0x00C9, size4MB, CMD_TYPE_AMD, "AT49BV322A(T) 2Mx16 TopB   (4MB)"   ,63,size64K,     8,size8K,   0,0,        0,0        },
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};


// -----------------------------------------
// ---- Start of Compiler Specific Code ----
// -----------------------------------------

void lpt_openport(void)
{
#ifdef WINDOWS_VERSION    // ---- Compiler Specific Code ----

    HANDLE h;

    h = CreateFile("\\\\.\\giveio", GENERIC_READ, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (h == INVALID_HANDLE_VALUE)
    {
        printf("Couldn't access giveio device\n");
        CloseHandle(h);
        exit(0);
    }
    CloseHandle(h);

#else                     // ---- Compiler Specific Code ----

#ifdef RASPPI          // ---- Compiler Specific Code ----

   int mem_fd;
   char *gpio_mem, *gpio_map;

   /* open /dev/mem */
   if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
      printf("can't open /dev/mem \n");
      exit(-1);
   }

   // Allocate MAP block
   if ((gpio_mem = malloc(BLOCK_SIZE + (PAGE_SIZE - 1))) == NULL) {
      printf("allocation error\n");
      exit(-1);
   }

   // Make sure pointer is on 4K boundary
   if ((unsigned long) gpio_mem % PAGE_SIZE)
      gpio_mem += PAGE_SIZE - ((unsigned long) gpio_mem % PAGE_SIZE);

   /* mmap GPIO */
   gpio_map = mmap(
      (caddr_t) gpio_mem,
      BLOCK_SIZE,
      PROT_READ|PROT_WRITE,
      MAP_SHARED|MAP_FIXED,
      mem_fd,
      GPIO_BASE
   );

   if ((long) gpio_map < 0) {
      printf("mmap error %ld\n", (long) gpio_map);
      exit(-1);
   }

   close(mem_fd);


   // Always use volatile pointer!
   gpio = (volatile unsigned *) gpio_map;

   // Set TDO as input and TMS,TCK and TDI as output
   
   INP_GPIO(TDO);
   INP_GPIO(TMS);
   INP_GPIO(TCK);
   INP_GPIO(TDI);

   OUT_GPIO(TMS);
   OUT_GPIO(TCK);
   OUT_GPIO(TDI);

#else

#ifdef __FreeBSD__     // ---- Compiler Specific Code ----

    pfd = open("/dev/ppi0", O_RDWR);
    if (pfd < 0)
    {
        perror("Failed to open /dev/ppi0");
        exit(0);
    }
    if ((ioctl(pfd, PPEXCL) < 0) || (ioctl(pfd, PPCLAIM) < 0))
    {
        perror("Failed to lock /dev/ppi0");
        close(pfd);
        exit(0);
    }

#else                  // ---- Compiler Specific Code ----

    pfd = open("/dev/parport0", O_RDWR);
    if (pfd < 0)
    {
        perror("Failed to open /dev/parport0");
        exit(0);
    }
    if ((ioctl(pfd, PPEXCL) < 0) || (ioctl(pfd, PPCLAIM) < 0))
    {
        perror("Failed to lock /dev/parport0");
        close(pfd);
        exit(0);
    }

#endif

#endif

#endif
}


void lpt_closeport(void)
{
#ifndef WINDOWS_VERSION   // ---- Compiler Specific Code ----

#ifdef RASPPI

#else

#ifndef __FreeBSD__    // ---- Compiler Specific Code ----

    if (ioctl(pfd, PPRELEASE) < 0)
    {
        perror("Failed to release /dev/parport0");
        close(pfd);
        exit(0);
    }

#endif

    close(pfd);

#endif

#endif
}

// Yoon's extensions for exiting debug mode

static void return_from_debug_mode(void)
{
    ExecuteDebugModule(pracc_return_from_debug);
}


static unsigned char clockin(int tms, int tdi)
{

#ifdef RASPPI
    unsigned int data;
#else
    unsigned char data;
#endif

    tms = tms ? 1 : 0;
    tdi = tdi ? 1 : 0;

#ifdef RASPPI

    data = (tms << TMS) | (tdi << TDI);
    GPIO_CLR = (1 << TCK) | (1 << TMS) | (1 << TDI);
    GPIO_SET = data;
    cable_wait();
    GPIO_SET = 1 << TCK;
    cable_wait();

    data = GPIO_GET;
    data = (data >> TDO) & 1;

#else

// yoon's remark we set wtrst_n to be d4 so we are going to drive it low
    if (wiggler) data = (1 << WTDO) | (0 << WTCK) | (tms << WTMS) | (tdi << WTDI)| (1 << WTRST_N);
    else        data = (1 << TDO) | (0 << TCK) | (tms << TMS) | (tdi << TDI);
    cable_wait();


#ifdef WINDOWS_VERSION   // ---- Compiler Specific Code ----
    _outp(0x378, data);
#else

    ioctl(pfd, PPWDATA, &data);
#endif
    if (wiggler) data = (1 << WTDO) | (1 << WTCK) | (tms << WTMS) | (tdi << WTDI) | (1 << WTRST_N);
    else        data = (1 << TDO) | (1 << TCK) | (tms << TMS) | (tdi << TDI);
    cable_wait();


#ifdef WINDOWS_VERSION   // ---- Compiler Specific Code ----
    _outp(0x378, data);
#else
    ioctl(pfd, PPWDATA, &data);
#endif

#ifdef WINDOWS_VERSION   // ---- Compiler Specific Code ----
    data = (unsigned char)_inp(0x379);
#else
    ioctl(pfd, PPRSTATUS, &data);
#endif

    data ^= 0x80;
    data >>= wiggler?WTDO:TDO;
    data &= 1;

#endif

    return data;
}

// ---------------------------------------
// ---- End of Compiler Specific Code ----
// ---------------------------------------

void test_reset(void)
{
    clockin(1, 0);  // Run through a handful of clock cycles with TMS high to make sure
    clockin(1, 0);  // we are in the TEST-LOGIC-RESET state.
    clockin(1, 0);
    clockin(1, 0);
    clockin(1, 0);
    clockin(0, 0);  // enter runtest-idle
}

static int curinstr = 0xFFFFFFFF;
void set_instr(int instr)
{
    int i;

    if (instr == curinstr)
        return;

    clockin(1, 0);  // enter select-dr-scan
    clockin(1, 0);  // enter select-ir-scan
    clockin(0, 0);  // enter capture-ir
    clockin(0, 0);  // enter shift-ir (dummy)
    for (i=0; i < instruction_length; i++)
    {
        clockin(i==(instruction_length - 1), (instr>>i)&1);
    }
    clockin(1, 0);  // enter update-ir
    clockin(0, 0);  // enter runtest-idle

    curinstr = instr;
}


static unsigned int ReadWriteData(unsigned int in_data)
{
    int i;
    unsigned int out_data = 0;
    unsigned char out_bit;

    if (DEBUG) printf("INSTR: 0x%04x  ", curinstr);
    if (DEBUG) printf("W: 0x%08x ", in_data);

    clockin(1, 0);  // enter select-dr-scan
    clockin(0, 0);  // enter capture-dr
    clockin(0, 0);  // enter shift-dr
    for (i = 0 ; i < 32 ; i++)
    {
        out_bit  = clockin((i == 31), ((in_data >> i) & 1));
        out_data = out_data | (out_bit << i);
    }
    clockin(1,0);   // enter update-dr
    clockin(0,0);   // enter runtest-idle

    if (DEBUG) printf("R: 0x%08x\n", out_data);

    return out_data;
}


static unsigned int ReadData(void)
{
    return ReadWriteData(0x00);
}


void WriteData(unsigned int in_data)
{
    ReadWriteData(in_data);
}

void ShowData(unsigned int value)
{
    unsigned int i;
    for (i=0; i<32; i++)
        printf("%d", (value >> (31-i)) & 1);
    printf(" (%08X)\n", value);
}

unsigned int swap_bytes(unsigned int data, int num_bytes)
{
    unsigned int datab[4], i;

    for (i = 0; i < num_bytes; i++)
    {
        datab[i] = (data >>((num_bytes - i - 1) * 8) & 0xFF);
    }

    data = 0x0;
    for (i = 0; i < num_bytes; i++)
    {
        data = (data | (datab[i] <<(8 * i)));
    }
    return data;
}

unsigned short byteSwap(unsigned short data)
{
    //convert from little to big endian
    unsigned short tmp;
    tmp=(data<<8)|(data>>8);
    return tmp;
}


static unsigned int ejtag_read(unsigned int addr)
{
    if (USE_DMA) return(ejtag_dma_read(addr));
    else return(ejtag_pracc_read(addr));
}

static unsigned int ejtag_read_h(unsigned int addr)
{
    if (USE_DMA) return(ejtag_dma_read_h(addr));
    else return(ejtag_pracc_read_h(addr));
}

void ejtag_write(unsigned int addr, unsigned int data)
{
    if (USE_DMA) ejtag_dma_write(addr, data);
    else ejtag_pracc_write(addr, data);
}

void ejtag_write_h(unsigned int addr, unsigned int data)
{
    if (USE_DMA) ejtag_dma_write_h(addr, data);
    else ejtag_pracc_write_h(addr, data);
}

static unsigned int ejtag_dma_read(unsigned int addr)
{
    unsigned int data;
    int retries = RETRY_ATTEMPTS;

begin_ejtag_dma_read:

    // Setup Address
    set_instr(INSTR_ADDRESS);
    WriteData(addr);

    // Initiate DMA Read & set DSTRT
    set_instr(INSTR_CONTROL);
    ctrl_reg = ReadWriteData(DMAACC | DRWN | DMA_WORD | DSTRT | PROBEN | PRACC);

    // Wait for DSTRT to Clear - Problem Gv8 tornado
    if (!((proc_id & 0xfffffff) == 0x535417f))
    {
    while (ReadWriteData(DMAACC | PROBEN | PRACC) & DSTRT);
    }

    // Read Data
    set_instr(INSTR_DATA);
    data = ReadData();


    // Clear DMA & Check DERR
    set_instr(INSTR_CONTROL);
    if (ReadWriteData(PROBEN | PRACC) & DERR)
    {
        if (retries--)  goto begin_ejtag_dma_read;
        else  printf("DMA Read Addr = %08x  Data = (%08x)ERROR ON READ\n", addr, data);
    }

    return(data);


}

static unsigned int ejtag_dma_read_h(unsigned int addr)
{
    unsigned int data;
    int retries = RETRY_ATTEMPTS;

begin_ejtag_dma_read_h:

    // Setup Address
    set_instr(INSTR_ADDRESS);
    WriteData(addr);

    // Initiate DMA Read & set DSTRT
    set_instr(INSTR_CONTROL);
    ctrl_reg = ReadWriteData(DMAACC | DRWN | DMA_HALFWORD | DSTRT | PROBEN | PRACC);

    // Wait for DSTRT to Clear
    while (ReadWriteData(DMAACC | PROBEN | PRACC) & DSTRT);

    // Read Data
    set_instr(INSTR_DATA);
    data = ReadData();

    // Clear DMA & Check DERR
    set_instr(INSTR_CONTROL);
    if (ReadWriteData(PROBEN | PRACC) & DERR)
    {
        if (retries--)  goto begin_ejtag_dma_read_h;
        else  printf("DMA Read Addr = %08x  Data = (%08x)ERROR ON READ\n", addr, data);
    }
    // Handle the bigendian / littleendian

    if ( addr & 0x2 )
        data = (data>>16)&0xffff;
    else
        data = (data&0x0000ffff);

    return(data);

}

void ejtag_dma_write(unsigned int addr, unsigned int data)
{
    int   retries = RETRY_ATTEMPTS;

begin_ejtag_dma_write:

    // Setup Address
    set_instr(INSTR_ADDRESS);
    WriteData(addr);

    // Setup Data
    set_instr(INSTR_DATA);
    WriteData(data);

    // Initiate DMA Write & set DSTRT
    set_instr(INSTR_CONTROL);
    ctrl_reg = ReadWriteData(DMAACC | DMA_WORD | DSTRT | PROBEN | PRACC);

    // Wait for DSTRT to Clear
    while (ReadWriteData(DMAACC | PROBEN | PRACC) & DSTRT);

    // Clear DMA & Check DERR
    set_instr(INSTR_CONTROL);
    if (ReadWriteData(PROBEN | PRACC) & DERR)
    {
        if (retries--)  goto begin_ejtag_dma_write;
        else  printf("DMA Write Addr = %08x  Data = ERROR ON WRITE\n", addr);
    }
}


void ejtag_dma_write_h(unsigned int addr, unsigned int data)
{
    int   retries = RETRY_ATTEMPTS;

begin_ejtag_dma_write_h:

    // Setup Address
    set_instr(INSTR_ADDRESS);
    WriteData(addr);

    // Setup Data
    set_instr(INSTR_DATA);
    WriteData(data);

    // Initiate DMA Write & set DSTRT
    set_instr(INSTR_CONTROL);
    ctrl_reg = ReadWriteData(DMAACC | DMA_HALFWORD | DSTRT | PROBEN | PRACC);

    // Wait for DSTRT to Clear
    while (ReadWriteData(DMAACC | PROBEN | PRACC) & DSTRT);

    // Clear DMA & Check DERR
    set_instr(INSTR_CONTROL);
    if (ReadWriteData(PROBEN | PRACC) & DERR)
    {
        if (retries--)  goto begin_ejtag_dma_write_h;
        else  printf("DMA Write Addr = %08x  Data = ERROR ON WRITE\n", addr);
    }
}

static unsigned int ejtag_pracc_read(unsigned int addr)
{
    address_register = addr | 0xA0000000;  // Force to use uncached segment
    data_register    = 0x0;
    ExecuteDebugModule(pracc_readword_code_module);
    return(data_register);
}

void ejtag_pracc_write(unsigned int addr, unsigned int data)
{
    address_register = addr | 0xA0000000;  // Force to use uncached segment
    data_register    = data;
    ExecuteDebugModule(pracc_writeword_code_module);
}

static unsigned int ejtag_pracc_read_h(unsigned int addr)
{
    address_register = addr | 0xA0000000;  // Force to use uncached segment
    data_register    = 0x0;
    ExecuteDebugModule(pracc_readhalf_code_module);
    return(data_register);
}


void ejtag_pracc_write_h(unsigned int addr, unsigned int data)
{
    address_register = addr | 0xA0000000;  // Force to use uncached segment
    data_register    = data;
    ExecuteDebugModule(pracc_writehalf_code_module);
}

void setup_memory_4712(void)
{
    printf("Configuring SDRAM... ");

    ejtag_dma_write(0x18006f98,0x00030001); // #define SBTMSTATELOW		offset 0xf00 + 0x98
    ejtag_dma_write(0x18006f98,0x00030000);
    ejtag_dma_write(0x18006f98,0x00010000);
    ejtag_dma_write(0x18006004,0x00048000); // #define MEMC_SD_CONFIG_INIT	0x00048000
    ejtag_dma_write(0x1800601c,0x000754da); // #define MEMC_SD_DRAMTIM3_INIT	0x000754da sbmemc.h
    ejtag_dma_write(0x18006034,0x23232323);
    ejtag_dma_write(0x18006038,0x14500200); // #define MEMC_SD1_WRNCDLCOR_INIT	0x14500200	 For corerev 1 (4712)
    ejtag_dma_write(0x1800603c,0x22021416); // #define MEMC_SD1_MISCDLYCTL_INIT 0x00021416	 For corerev 1 (4712)
    ejtag_dma_write(0x18006000,0x00000002); // #define MEMC_SD_CONTROL_INIT0	0x00000002
    ejtag_dma_write(0x18006000,0x00000008); // #define MEMC_SD_CONTROL_INIT1	0x00000008
    ejtag_dma_write(0x18006000,0x00000004);
    ejtag_dma_write(0x18006000,0x00000004);
    ejtag_dma_write(0x18006000,0x00000004);
    ejtag_dma_write(0x18006000,0x00000004);
    ejtag_dma_write(0x18006000,0x00000004);
    ejtag_dma_write(0x18006000,0x00000004);
    ejtag_dma_write(0x18006000,0x00000004);
    ejtag_dma_write(0x18006000,0x00000004);
    ejtag_dma_write(0x18006008,0x0000840f); // #define MEMC_SD_REFRESH_INIT	0x0000840f
    ejtag_dma_write(0x18006010,0x00000032); // sdram_config=0x0032
    ejtag_dma_write(0x18006000,0x00000010); // #define MEMC_CONTROL_INIT2	0x00000010
    ejtag_dma_write(0x18006000,0x00000001); //


    printf("Done\n\n");
}

void setup_memory_5352(void)
{
    printf("Configuring SDRAM... ");

    ejtag_dma_write(0x18004f98,0x00030001); // SBTMSTATELOW		offset 0xf00 + 0x98
    ejtag_dma_write(0x18004f98,0x00030000);
    ejtag_dma_write(0x18004f98,0x00010000);
    ejtag_dma_write(0x18004004,0x0004810b); // MEMC_SD_CONFIG_INIT	0x00048000
    ejtag_dma_write(0x1800401c,0x000754d9); // MEMC_SD_DRAMTIM3_INIT	0x000754d9 sbmemc.h
    ejtag_dma_write(0x18004034,0x23232323);
    ejtag_dma_write(0x18004038,0x14500200); // MEMC_SD1_WRNCDLCOR_INIT	0x14500200	 For corerev 1 (4712)
    ejtag_dma_write(0x1800403c,0x21021400); // MEMC_SD1_MISCDLYCTL_INIT 0x00021416	 For corerev 1 (4712)
    ejtag_dma_write(0x18004000,0x00000002); // MEMC_SD_CONTROL_INIT0	0x00000002
    ejtag_dma_write(0x18004000,0x00000008); // MEMC_SD_CONTROL_INIT1	0x00000008
    ejtag_dma_write(0x18004000,0x00000004); // MEMC_SD_CONTROL_INIT2
    ejtag_dma_write(0x18004000,0x00000004); // MEMC_SD_CONTROL_INIT2
    ejtag_dma_write(0x18004000,0x00000004); // MEMC_SD_CONTROL_INIT2
    ejtag_dma_write(0x18004000,0x00000004); // MEMC_SD_CONTROL_INIT2
    ejtag_dma_write(0x18004000,0x00000004); // MEMC_SD_CONTROL_INIT2
    ejtag_dma_write(0x18004000,0x00000004); // MEMC_SD_CONTROL_INIT2
    ejtag_dma_write(0x18004000,0x00000004); // MEMC_SD_CONTROL_INIT2
    ejtag_dma_write(0x18004000,0x00000004); // MEMC_SD_CONTROL_INIT2
    ejtag_dma_write(0x18004008,0x0000840f); // MEMC_SD_REFRESH_INIT	0x0000840f
    ejtag_dma_write(0x18004010,0x00000062); // sdram_config=0x0062
    ejtag_dma_write(0x18004000,0x00000010); // MEMC_SD_CONTROL_INIT3
    ejtag_dma_write(0x18004000,0x00000001); // MEMC_SD_CONTROL_INIT4

    printf("Done\n\n");
}

void readmem_4712(void)
{
    int temp;

    printf("Printing SDRAM\n");

    temp = ejtag_read(0x18006f98);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x18006f98);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x18006f98);;
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x18006004);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x1800601c);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x18006034);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x18006038);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x1800603c);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x18006000);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x18006000);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x18006000);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x18006000);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x18006000);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x18006000);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x18006000);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x18006000);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x18006000);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x18006000);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x18006008);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x18006010);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x18006000);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x18006000);
    printf("SDRAM = 0x%08x\n", temp);

}

void readmem_5352(void)
{
    int temp = 0;

    printf("Printing SDRAM\n");

    temp = ejtag_read(0x18004f98);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x18004f98);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x18004f98);;
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x18004004);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x1800401c);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x18004034);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x18004038);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x1800403c);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x18004000);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x18004000);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x18004000);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x18004000);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x18004000);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x18004000);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x18004000);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x18004000);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x18004000);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x18004000);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x18004008);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x18004010);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x18004000);
    printf("SDRAM = 0x%08x\n", temp);
    temp = ejtag_read(0x18004000);
    printf("SDRAM = 0x%08x\n", temp);

}


void ExecuteDebugModule(unsigned int *pmodule)
{
    unsigned int ctrl_reg;
    unsigned int address;
    unsigned int data   = 0;
    unsigned int offset = 0;
    int finished = 0;
    int DEBUGMSG = 0;

    if (DEBUGMSG) printf("DEBUGMODULE: Start module.\n");

    // Feed the chip an array of 32 bit values into the processor via the EJTAG port as instructions.
    while (1)
    {
        // Read the control register.  Make sure an access is requested, then do it.
        while (1)
        {
            set_instr(INSTR_CONTROL);
            ctrl_reg = ReadWriteData(PRACC | PROBEN | SETDEV);
            if (ctrl_reg & PRACC)
                break;
            if (DEBUGMSG) printf("DEBUGMODULE: No memory access in progress!\n");
        }

        set_instr(INSTR_ADDRESS);
        address = ReadData();

        // Check for read or write
        if (ctrl_reg & PRNW) // Bit set for a WRITE
        {
            // Read the data out
            set_instr(INSTR_DATA);
            data = ReadData();

            // Clear the access pending bit (let the processor eat!)
            set_instr(INSTR_CONTROL);
            ctrl_reg = ReadWriteData(PROBEN | SETDEV);

            // Processor is writing to us
            if (DEBUGMSG) printf("DEBUGMODULE: Write 0x%08X to address 0x%08X\n", data, address);
            // Handle Debug Write
            // If processor is writing to one of our psuedo virtual registers then save off data
            if (address == MIPS_VIRTUAL_ADDRESS_ACCESS)  address_register = data;
            if (address == MIPS_VIRTUAL_DATA_ACCESS)     data_register    = data;
        }

        else

        {
            // Check to see if its reading at the debug vector.  The first pass through
            // the module is always read at the vector, so the first one we allow.  When
            // the second read from the vector occurs we are done and just exit.
            if (address == MIPS_DEBUG_VECTOR_ADDRESS)
            {
                if (finished++) // Allows ONE pass
                {
                    if (DEBUGMSG) printf("DEBUGMODULE: Finished module.\n");
                    return;
                }
            }

            // Processor is reading from us
            if (address >= MIPS_DEBUG_VECTOR_ADDRESS)
            {
                // Reading an instruction from our module so fetch the instruction from the module
                offset = (address - MIPS_DEBUG_VECTOR_ADDRESS) / 4;
                data = *(unsigned int *)(pmodule + offset);
                if (DEBUGMSG) printf("DEBUGMODULE: Instruction read at 0x%08X  offset -> %04d  data -> 0x%08X\n", address, offset, data); //fflush(stdout);
            }
            else
            {
                // Reading from our virtual register area
                if (DEBUGMSG) printf("DEBUGMODULE: Read address 0x%08X  data = 0x%08X\n", address, data);
                // Handle Debug Read
                // If processor is reading from one of our psuedo virtual registers then give it data
                if (address == MIPS_VIRTUAL_ADDRESS_ACCESS)  data = address_register;
                if (address == MIPS_VIRTUAL_DATA_ACCESS)     data = data_register;
            }

            // Send the data out
            set_instr(INSTR_DATA);
            data = ReadWriteData(data);

            // Clear the access pending bit (let the processor eat!)
            set_instr(INSTR_CONTROL);
            ctrl_reg = ReadWriteData(PROBEN | SETDEV);

        }
    }
}

void chip_detect(void)
{
    unsigned int id = 0x0;

    processor_chip_type*   processor_chip = processor_chip_list;

    lpt_openport();

    printf("Probing bus ... ");

    if (skipdetect)
    {
        // Manual Override CPU Chip ID
        test_reset();
        instruction_length = instrlen;
        set_instr(INSTR_IDCODE);
        id = ReadData();
        printf("Done\n\n");
        printf("Instruction Length set to %d\n\n",instruction_length);
        printf("CPU Chip ID: ");
        ShowData(id);
        printf("*** CHIP DETECTION OVERRIDDEN ***\n\n");
        return;
    }
    else
    {
        // Auto Detect CPU Chip ID
        while (processor_chip->chip_id)
        {
            test_reset();
            if (instrlen)
                instruction_length = instrlen;
            else
                instruction_length = processor_chip->instr_length;
            set_instr(INSTR_IDCODE);
            id = ReadData();
            if (id == processor_chip->chip_id)
            {
                printf("Done\n\n");
                printf("Instruction Length set to %d\n\n",instruction_length);
                printf("CPU Chip ID: ");
                ShowData(id);
                printf("*** Found a %s chip ***\n\n", processor_chip->chip_descr);
                proc_id = id;

                return;
            }
            processor_chip++;
        }
    }

    printf("Done\n\n");
    printf("Instruction Length set to %d\n\n",instruction_length);
    printf("CPU Chip ID: ");
    ShowData(id);
    printf("*** Unknown or NO CPU Chip ID Detected ***\n\n");

    printf("*** Possible Causes:\n");
    printf("    1) Device is not Connected.\n");
    printf("    2) Device is not Powered On.\n");
    printf("    3) Improper JTAG Cable.\n");
    printf("    4) Unrecognized CPU Chip ID.\n");

    chip_shutdown();;
    exit(0);
}

void check_ejtag_features()
{
    unsigned int features;

    set_instr(INSTR_IMPCODE);
    features = ReadData();

    printf("    - EJTAG IMPCODE ....... : ");
    ShowData(features);

    // EJTAG Version
    ejtag_version = (features >> 29) & 7;
    printf("    - EJTAG Version ....... : ");
    if (ejtag_version == 0)       printf("1 or 2.0\n");
    else if (ejtag_version == 1)  printf("2.5\n");
    else if (ejtag_version == 2)  printf("2.6\n");
    else if (ejtag_version == 3)  printf("3.1\n");
    else                          printf("Unknown (%d is a reserved value)\n", ejtag_version);

    // EJTAG DMA Support
    USE_DMA = !(features & (1 << 14));
    printf("    - EJTAG DMA Support ... : %s\n", USE_DMA ? "Yes" : "No");
    printf( "    - EJTAG Implementation flags:%s%s%s%s%s%s%s\n",
            (features & (1 << 28)) ? " R3k"	: " R4k",
            (features & (1 << 24)) ? " DINTsup"	: "",
            (features & (1 << 22)) ? " ASID_8"	: "",
            (features & (1 << 21)) ? " ASID_6"	: "",
            (features & (1 << 16)) ? " MIPS16"	: "",
            (features & (1 << 14)) ? " NoDMA"	: "",
            (features & (1      )) ? " MIPS64"	: " MIPS32" );

    if (force_dma)
    {
        USE_DMA = 1;
        printf("    *** DMA Mode Forced On ***\n");
    }
    if (force_nodma)
    {
        USE_DMA = 0;
        printf("    *** DMA Mode Forced Off ***\n");
    }

    printf("\n");
}


void chip_shutdown(void)
{
    fflush(stdout);
    test_reset();
    lpt_closeport();
}

void
cable_wait( void )
{
    if (!delay)
      return;

    for (dcounter = 0; dcounter < delay; dcounter++);

}


void unlock_bypass(void)
{
    ejtag_write_h(FLASH_MEMORY_START + (0x555 << 1), 0x00900090 ); /* unlock bypass reset */
    ejtag_write_h(FLASH_MEMORY_START + (0x555 << 1), 0x00aa00aa ); /* unlock bypass */
    ejtag_write_h(FLASH_MEMORY_START + (0x2aa << 1), 0x00550055 );
    ejtag_write_h(FLASH_MEMORY_START + (0x555 << 1), 0x00200020 );
    printf("\nEntered Unlock Bypass mode->\n");
}


void unlock_bypass_reset(void)
{
    ejtag_write_h(FLASH_MEMORY_START + (0x555 << 1), 0x00900090 ); /* unlock bypass reset */
    ejtag_write_h(FLASH_MEMORY_START + (0x000), 0x00000000 );
}

void run_backup(char *filename, unsigned int start, unsigned int length)
{
    unsigned int addr, data;
    FILE *fd;
    int counter = 0;
    int percent_complete = 0;
    char newfilename[128] = "";
//    int swp_endian = (cmd_type == CMD_TYPE_SPI);
    time_t start_time = time(0);
    time_t end_time, elapsed_seconds;

    struct tm* lt = localtime(&start_time);
    char time_str[16];

    sprintf(time_str, "%04d%02d%02d_%02d%02d%02d",
            lt->tm_year + 1900, lt->tm_mon + 1, lt->tm_mday,
            lt->tm_hour, lt->tm_min, lt->tm_sec
           );

    printf("*** You Selected to Backup the %s ***\n\n",filename);

    strcpy(newfilename,filename);
    strcat(newfilename,".SAVED");
    if (issue_timestamp)
    {
        strcat(newfilename,"_");
        strcat(newfilename,time_str);
    }

    fd = fopen(newfilename, "wb" );
    if (fd<=0)
    {
        fprintf(stderr,"Could not open %s for writing\n", newfilename);
        exit(1);
    }

    printf("=========================\n");
    printf("Backup Routine Started\n");
    printf("=========================\n");

    printf("\nSaving %s to Disk...\n",newfilename);
    for (addr=start; addr<(start+length); addr+=4)
    {
        counter += 4;
        percent_complete = (counter * 100 / length);
        if (!silent_mode)
            if ((addr&0xF) == 0)  printf("[%3d%% Backed Up]   %08x: ", percent_complete, addr);

        data = ejtag_read(addr);


        if (swap_endian) data = byteSwap_32(data);
        fwrite( (unsigned char*) &data, 1, sizeof(data), fd);

        if (silent_mode)  printf("%4d%%   bytes = %d\r", percent_complete, counter);
        else              printf("%08x%c", data, (addr&0xF)==0xC?'\n':' ');

        fflush(stdout);
    }



    fclose(fd);

    printf("Done  (%s saved to Disk OK)\n\n",newfilename);

    printf("bytes written: %d\n", counter);

    printf("=========================\n");
    printf("Backup Routine Complete\n");
    printf("=========================\n");

    time(&end_time);
    elapsed_seconds = difftime(end_time, start_time);
    printf("elapsed time: %d seconds\n", (int)elapsed_seconds);
}

void run_erase(char *filename, unsigned int start, unsigned int length)
{
    time_t start_time = time(0);
    time_t end_time, elapsed_seconds;

    printf("*** You Selected to Erase the %s ***\n\n",filename);

    printf("=========================\n");
    printf("Erasing Routine Started\n");
    printf("=========================\n");

    sflash_erase_area(start,length);
    sflash_reset();

    printf("=========================\n");
    printf("Erasing Routine Complete\n");
    printf("=========================\n");

    time(&end_time);
    elapsed_seconds = difftime(end_time, start_time);
    printf("elapsed time: %d seconds\n", (int)elapsed_seconds);
}


void identify_flash_part(void)
{
    flash_chip_type*   flash_chip = flash_chip_list;
    flash_area_type*   flash_area = flash_area_list;

    // Important for these to initialize to zero
    block_addr  = 0;
    block_total = 0;
    flash_size  = 0;
    cmd_type    = 0;
    strcpy(flash_part,"");

    /*     Spansion ID workaround (kb1klk) 30 Dec 2007

         Vendor ID altered to 017E to avoid ambiguity in lookups.
         Spansion uses vend 0001 dev 227E for numerous chips.
         This code reads device SubID from its address and combine this into
         the unique devid that will be used in the lookup table. */

    if (((vendid & 0x00ff) == 0x0001) && (devid == 0x227E))
    {
        unsigned int devsubid_m, devsubid_l;
        vendid = 0x017E;
        devsubid_m = 0x00ff & ejtag_read_h(FLASH_MEMORY_START+0x1C);  // sub ID step 1
        devsubid_l = 0x00ff & ejtag_read_h(FLASH_MEMORY_START+0x1E);  // sub ID step 2
        devid = (0x0100 * devsubid_m) + (0x0000 + devsubid_l);
    }


    /* WinBond extended ID query - 27 Jan 2008 */

    if (((vendid & 0x00ff) == 0x00DA) && ((devid & 0x00ff) == 0x007E))
    {
        unsigned int devsubid_m, devsubid_l;
        vendid = 0xDA7E;
        devsubid_m = 0x00ff & ejtag_read_h(FLASH_MEMORY_START+0x1C);  // sub ID step 1
        devsubid_l = 0x00ff & ejtag_read_h(FLASH_MEMORY_START+0x1E);  // sub ID step 2
        devid = (0x0100 * devsubid_m) + (0x0000 + devsubid_l);
    }



    while (flash_chip->vendid)
    {
        if ((flash_chip->vendid == vendid) && (flash_chip->devid == devid))
        {
            flash_size = flash_chip->flash_size;
            cmd_type   = flash_chip->cmd_type;
            strcpy(flash_part, flash_chip->flash_part);

            if (strcasecmp(AREA_NAME,"CUSTOM")==0)
            {
                FLASH_MEMORY_START = selected_window;
            }
            else
            {
                switch (proc_id)
                {

                case IXP425_266:
                case IXP425_400:
                    //   case IXP425_533:
                    //       FLASH_MEMORY_START = 0x50000000;
                    //       break;
                case ARM_940T:
                    FLASH_MEMORY_START = 0x00400000;
                    break;
                case 0x0635817F:
                    FLASH_MEMORY_START = 0x1F000000;
                    break;
//    case ATH_PROC:
//        FLASH_MEMORY_START = 0xA8000000;
//        break;

               case 0x0000100F: //Ti AR7
                    FLASH_MEMORY_START = 0x90000000;
                    break;

                default:
                    if (flash_size >= size8MB )
                    {

                        FLASH_MEMORY_START = 0x1C000000;
                    }
                    else
                    {

                        FLASH_MEMORY_START = 0x1FC00000;
                    }

                }
            }



            if (proc_id == 0x00000001)
            {

                if ((strcasecmp(AREA_NAME,"CFE")==0)  && flash_size >= size8MB)
                    strcpy(AREA_NAME, "AR-CFE");

                if ((strcasecmp(AREA_NAME,"NVRAM")==0) && flash_size >= size8MB)
                    strcpy(AREA_NAME, "AR-NVRAM");

                if ((strcasecmp(AREA_NAME,"KERNEL")==0) && flash_size >= size8MB)
                    strcpy(AREA_NAME, "AR-KERNEL");

                if ((strcasecmp(AREA_NAME,"WHOLEFLASH")==0) && flash_size >= size8MB)
                    strcpy(AREA_NAME, "AR-WHOLEFLASH");

                if ((strcasecmp(AREA_NAME,"BSP")==0) && flash_size >= size8MB)
                    strcpy(AREA_NAME, "AR-BSP");

                if ((strcasecmp(AREA_NAME,"RED")==0) && flash_size >= size8MB)
                    strcpy(AREA_NAME, "AR-RED");
            }


            while (flash_area->chip_size)
            {
                if ((flash_area->chip_size == flash_size) && (strcasecmp(flash_area->area_name, AREA_NAME)==0))
                {

                    strcat(AREA_NAME,".BIN");
                    AREA_START  = flash_area->area_start;
                    AREA_LENGTH = flash_area->area_length;
                    break;
                }
                flash_area++;
            }

            if (strcasecmp(AREA_NAME,"CUSTOM")==0)
            {
                strcat(AREA_NAME,".BIN");
                FLASH_MEMORY_START = selected_window;
                AREA_START         = selected_start;
                AREA_LENGTH        = selected_length;
            }

            if (flash_chip->region1_num)  define_block(flash_chip->region1_num, flash_chip->region1_size);
            if (flash_chip->region2_num)  define_block(flash_chip->region2_num, flash_chip->region2_size);
            if (flash_chip->region3_num)  define_block(flash_chip->region3_num, flash_chip->region3_size);
            if (flash_chip->region4_num)  define_block(flash_chip->region4_num, flash_chip->region4_size);

            sflash_reset();

            printf("Done\n\n");
            printf("Flash Vendor ID: ");
            ShowData(vendid);
            printf("Flash Device ID: ");
            ShowData(devid);
            if (selected_fc != 0)
                printf("*** Manually Selected a %s Flash Chip ***\n\n", flash_part);
            else
                printf("*** Found a %s Flash Chip ***\n\n", flash_part);

            printf("    - Flash Chip Window Start .... : %08x\n", FLASH_MEMORY_START);
            printf("    - Flash Chip Window Length ... : %08x\n", flash_size);
            printf("    - Selected Area Start ........ : %08x\n", AREA_START);
            printf("    - Selected Area Length ....... : %08x\n\n", AREA_LENGTH);

            break;
        }
        flash_chip++;
    }

}

void define_block(unsigned int block_count, unsigned int block_size)
{
    unsigned int  i;

    if (block_addr == 0)  block_addr = FLASH_MEMORY_START;

    for (i = 1; i <= block_count; i++)
    {
        block_total++;
        blocks[block_total] = block_addr;
        block_addr = block_addr + block_size;

    }
}


void sflash_config(void)
{
    flash_chip_type*   flash_chip = flash_chip_list;
    int counter = 0;

    printf("\nManual Flash Selection ... ");

    while (flash_chip->vendid)
    {
        counter++;
        if (counter == selected_fc)
        {
            vendid = flash_chip->vendid;
            devid  = flash_chip->devid;
            identify_flash_part();
            break;
        }
        flash_chip++;
    }

    if (strcasecmp(flash_part,"")==0)
        printf("*** Unknown or NO Flash Chip Selected ***\n");

}

unsigned char ATstatus_reg(void)
{
    unsigned char status;

    ejtag_write(0x1fc00000, 0x57);
    status = ejtag_read(0x1fc00000 &0x80);
    printf("id bit = 0x%08x\n", status&0x3c);
    ShowData(status);
    return status;
}

unsigned int ATready(void)
{

    int status;
    for (;;)
    {
        status = ATstatus_reg();
        status = (status&0xF<<7);
//        printf("status1 = 0x%08x\n", status);
//        ShowData(status);

        if (status != 0x80)
        {
            //  printf("Status BSY 0x%08x\n", status);
            status = 0;
            tnano(10000);

        }

        if (status == 0x80)	/* RDY/nBSY */


            return status;

    }
}


void mscan(void)
{
    unsigned int addr = 0x18000000, val=0, i;

    for (i=0; i < 20000; i++)
    {

        // val = ((ejtag_read(addr)) &CID_ID_MASK);
        val = ejtag_dma_read(addr);
        // if (val != NULL && val != 0x0);
        printf("data 0x%08x addr 0x%08x\n", val, addr);
        addr += 0x100;


    }

}


void isbrcm(void)
{
    /*
                if ((proc_id & 0xfff) == 0x17f)
                {
                    struct chipcregs *cc;

                    uint32_t reg;
                    unsigned long osh = 0x18000000; //0x18000000;
                    cc = (chipcregs_t *)osh;

                    reg = ejtag_read((uintptr_t) &cc->chipid) &CID_ID_MASK;
                    printf("\n\nChip id %x\n", reg);

                    reg = ejtag_read((uintptr_t) &cc->chipid) &CID_REV_MASK;
                    reg = (reg >> CID_REV_SHIFT);
                    printf("Chip Rev %x\n", reg);

                    reg = ejtag_read((uintptr_t) &cc->chipid) &CID_PKG_MASK;
                    reg = (reg >> CID_PKG_SHIFT);
                    printf("Package Options %x\n", reg);

                    reg = ejtag_read((uintptr_t) &cc->chipid) &CID_CC_MASK;
                    reg = (reg >> CID_CC_SHIFT);
                    printf("Number of Cores %x\n", reg );

                    reg = ejtag_read((uintptr_t) &cc->chipid) & 0x00007000;
                    reg = ((reg >> 8) |  0x0000000F);
                    printf("Core Revision %x\n",  reg);

                    reg = ejtag_read((uintptr_t) &cc->chipid) & 0x00008FF0;
                    printf("Core Type %x\n", reg);

                    reg = ejtag_read((uintptr_t) &cc->chipid) & 0xFFFF0000;
                    printf("Core Vendor ID %x\n", reg);

                    reg = ejtag_read((uintptr_t) &cc->capabilities) &CC_CAP_FLASH_MASK;
                    printf("Flash Type %x\n", reg);

                    printf("REG = %lu CC = %lu \n", sizeof(reg ), sizeof(cc->chipid) );

                    switch (reg)
                    {
                    case FLASH_NONE:
                        printf("Flash Type = FLASH_NONE\n");
                        break;
                    case SFLASH_ST:
                        printf("Flash Type = SFLASH_ST\n");
                        break;
                    case SFLASH_AT:
                        printf("Flash Type = FLASH_AT\n");
                        break;
                    case PFLASH:
                        printf("Flash Type = PFLASH\n");
                        break;
                    default:
                        break;
                    }

                    reg = ejtag_read((unsigned long) &cc->capabilities) &CC_CAP_MIPSEB;
                    if (reg == 0)
                    {
                        printf("Endian Type is LE %x\n", reg);
                    }
                    else
                        printf("Endian Type is BE %x\n", reg);

                    reg = ejtag_read((unsigned long) &cc->capabilities) &CC_CAP_PLL_MASK;
                    printf("PLL Type %08x\n", reg);

                }
    */
    if ((proc_id & 0x00000fff) == 0x17f)
    {
        bcmproc = 1;

        spi_flash_read = BRCM_SPI_READ;
        spi_flash_mmr = BRCM_SPI_MMR;
        spi_flash_mmr_size = BRCM_SPI_MMR_SIZE;
        spi_flash_ctl = BRCM_SPI_CTL;
        spi_flash_opcode = BRCM_SPI_OPCODE;
        spi_flash_data = BRCM_SPI_DATA;
        spi_ctl_start = BRCM_SPI_CTL_START;
        spi_ctl_busy = BRCM_SPI_CTL_BUSY;


    }
    else
    {
        spi_flash_read = AR531XPLUS_SPI_READ;
        spi_flash_mmr = AR531XPLUS_SPI_MMR;
        spi_flash_mmr_size = AR531XPLUS_SPI_MMR_SIZE;
        spi_flash_ctl = AR531XPLUS_SPI_CTL;
        spi_flash_opcode = AR531XPLUS_SPI_OPCODE;
        spi_flash_data = AR531XPLUS_SPI_DATA;
        spi_ctl_start = AR_SPI_CTL_START;
        spi_ctl_busy = AR_SPI_CTL_BUSY;
    }

    if (Flash_DEBUG)
    {
        printf("spi_flash_read 0x%08x\n", spi_flash_read);
        printf("spi_flash_mmr  0x%08x\n", spi_flash_mmr);
        printf("spi_flash_mmr_size 0x%08x\n", spi_flash_mmr_size);
        printf("spi_flash_ctl  0x%08x\n", spi_flash_ctl);
        printf("spi_flash_opcode 0x%08x\n", spi_flash_opcode);
        printf("spi_flash_data 0x%08x\n", spi_flash_data);
        printf("spi_ctl_start 0x%08x\n", spi_ctl_start );
        printf("spi_ctl_busy 0x%08x\n", spi_ctl_busy);
    }
}


int spiflash_poll(void)
{
    int reg, finished;

    /* Check for ST Write In Progress bit */
    spiflash_sendcmd(SPI_RD_STATUS);
    reg = ejtag_read(spi_flash_data);
    if (!(reg & SPI_STATUS_WIP))
    {
        finished = TRUE;
        printf("REG SPIFLASH_POLL 0x%08x\n", reg);
    }
    while (!finished);
    return 0;
}


uint32_t spiflash_regread32(int reg)
{

    uint32_t data = ejtag_read( spi_flash_mmr + reg );
    if (Flash_DEBUG)
        printf("REGREAD32 data 0x%08x spi_flash_mmr 0x%08x reg 0x%08x\n", data, spi_flash_mmr, reg );

    return data;
}

static void spiflash_regwrite32(int reg, uint32_t data)
{

    ejtag_write(spi_flash_mmr + reg, data );
    if (Flash_DEBUG)
        printf("REG 0x%08x REGWRITE32 0x%08x\n", reg, data);

    return;
}


uint32_t spiflash_sendcmd(int op)
{
    uint32_t reg, mask;
    struct opcodes *ptr_opcode;

    ptr_opcode = &stm_opcodes[op];

    if (bcmproc)
        ejtag_write(0x18000040, 0x0000);


    /* wait for CPU spiflash activity. */
    do
    {
        reg = spiflash_regread32(spi_flash_ctl);

    }
    while (reg & spi_ctl_busy);

    /* send the command */

    spiflash_regwrite32(spi_flash_opcode, ptr_opcode->code);

    if (Flash_DEBUG)
        printf("SPI_FLASH_OPCODE 0x%08x PTR_OPCODE 0x%08x\n", spi_flash_opcode, ptr_opcode->code);

    if (bcmproc)
        reg = (reg  & ~SPI_CTL_TX_RX_CNT_MASK) | ptr_opcode->code | spi_ctl_start;
    else
        reg = (reg & ~SPI_CTL_TX_RX_CNT_MASK) | ptr_opcode->tx_cnt | (ptr_opcode->rx_cnt << 4)| spi_ctl_start;

    spiflash_regwrite32(spi_flash_ctl, reg);

    if (Flash_DEBUG)
        printf("SPI_FLASH_CTL SEND -> 0x%08x reg 0x%08x\n", spi_flash_ctl, reg);


    /* wait for CPU spiflash activity */
    do
    {
        reg = spiflash_regread32(spi_flash_ctl);

    }
    while (reg & spi_ctl_busy);

    if (ptr_opcode->rx_cnt > 0)
    {
        reg = (uint32_t) spiflash_regread32(spi_flash_data);

        switch (ptr_opcode->rx_cnt)
        {
        case 1:
            mask = 0x000000ff;
            break;
        case 2:
            mask = 0x0000ffff;
            break;
        case 3:
            mask = 0x00ffffff;
            break;
        default:
            mask = 0xffffffff;
            break;
        }
        reg &= mask;
    }
    else
    {
        reg = 0;
    }

    return reg;
}

int spi_chiperase(uint32_t offset)
{

    ejtag_write(0x18000040, 0x0000);

    spiflash_sendcmd(SPI_WRITE_ENABLE);

    ejtag_write(BRCM_FLASHADDRESS, offset);

    ejtag_write(0x18000040, 0x800000c7);

    return 0;

}


static int spiflash_erase_block( uint32_t addr )
{

    struct opcodes *ptr_opcode;
    uint32_t temp, reg;
    int finished = FALSE;

    /* We are going to do 'sector erase', do 'write enable' first. */
    if (bcmproc)
        ptr_opcode = &stm_opcodes[BCM_SPI_SECTOR_ERASE];
    else
        ptr_opcode = &stm_opcodes[SPI_SECTOR_ERASE];

    if (bcmproc)
        ejtag_write(0x18000040, 0x0000);

    spiflash_sendcmd(SPI_WRITE_ENABLE);

    /* we are not really waiting for CPU spiflash activity, just need the value of the register. */

    do
    {
        reg = spiflash_regread32(spi_flash_ctl);

    }
    while (reg & spi_ctl_busy);

    /* send our command */
    if (bcmproc)
        temp = ((uint32_t) addr) | (uint32_t)(ptr_opcode->code);
    else
        temp = ((uint32_t) addr << 8) | (uint32_t)(ptr_opcode->code);

    spiflash_regwrite32(spi_flash_opcode, temp);

    if (bcmproc)
        reg = (reg & ~SPI_CTL_TX_RX_CNT_MASK) | ptr_opcode->code | spi_ctl_start;
    else
        reg = (reg & ~SPI_CTL_TX_RX_CNT_MASK) | ptr_opcode->tx_cnt | spi_ctl_start;

    spiflash_regwrite32(spi_flash_ctl, reg );

    if (bcmproc)
        ejtag_write(0x18000040, 0x0000);

    /* wait for CPU spiflash activity */


    do
    {
        reg = spiflash_regread32(spi_flash_ctl);

    }
    while (reg & spi_ctl_busy);

    /* wait for 'write in progress' to clear */
    do
    {
        if (bcmproc)
            reg = spiflash_sendcmd(BCM_SPI_RD_STATUS);
        else
            reg = spiflash_sendcmd(SPI_RD_STATUS);
        if (!(reg & SPI_STATUS_WIP)) finished = TRUE;
    }
    while (!finished);

    return (0);
}

void spiflash_write_word(uint32_t addr, uint32_t data)
{
    int finished;
    uint32_t reg, opcode;


    if (bcmproc)
    {
        ejtag_write(spi_flash_ctl, 0x000);
        spiflash_sendcmd(BCM_SPI_WRITE_ENABLE);
    }
    else
        spiflash_sendcmd(SPI_WRITE_ENABLE);

    /* we are not really waiting for CPU spiflash activity, just need the value of the register. */
    do
    {
        reg = spiflash_regread32(spi_flash_ctl);

    }
    while (reg & spi_ctl_busy);

    /* send write command */

    spiflash_regwrite32(spi_flash_data, data);

    if (bcmproc)
        opcode = (addr);
    else
        opcode = STM_OP_PAGE_PGRM | (addr << 8);

    spiflash_regwrite32(spi_flash_opcode, opcode);

    if (bcmproc)
        reg = (reg & ~SPI_CTL_TX_RX_CNT_MASK) | 0x0402 | spi_ctl_start;
    else
        reg = (reg & ~SPI_CTL_TX_RX_CNT_MASK) | (4 + 4) | spi_ctl_start;

    spiflash_regwrite32(spi_flash_ctl, reg);

    if (Flash_DEBUG)
        printf("spi_flash_ctl 0x%08x reg 0x%08x\n", spi_flash_ctl, reg);

    finished = 0;

    /* wait CPU spi activity */
    do
    {
        reg = spiflash_regread32(spi_flash_ctl);


    }
    while (reg & spi_ctl_busy);

    do
    {
        if (bcmproc)
            reg = spiflash_sendcmd(BCM_SPI_RD_STATUS);
        else
            reg = spiflash_sendcmd(SPI_RD_STATUS);

        if (!(reg & SPI_STATUS_WIP))
        {
            finished = TRUE;
        }
    }
    while (!finished);
}


void run_flash(char *filename, unsigned int start, unsigned int length)
{
    unsigned int addr, data ;
    FILE *fd ;
    int counter = 0;
    int percent_complete = 0;
    time_t start_time = time(0);
    time_t end_time, elapsed_seconds;

    printf("*** You Selected to Flash the %s ***\n\n",filename);

    fd=fopen(filename, "rb" );
    if (fd<=0)
    {
        fprintf(stderr,"Could not open %s for reading\n", filename);
        exit(1);
    }

    printf("=========================\n");
    printf("Flashing Routine Started\n");
    printf("=========================\n");

    if (issue_erase) sflash_erase_area(start,length);

    if (bypass)
    {
        unlock_bypass();
    }

    printf("\nLoading %s to Flash Memory...\n",filename);
    for (addr=start; addr<(start+length); addr+=4)
    {
        counter += 4;
        percent_complete = (counter * 100 / length);
        if (!silent_mode)
            if ((addr&0xF) == 0)  printf("[%3d%% Flashed]   %08x: ", percent_complete, addr);

        fread( (unsigned char*) &data, 1,sizeof(data), fd);

        // Erasing Flash Sets addresses to 0xFF's so we can avoid writing these (for speed)
        if (issue_erase)
        {
            if (!(data == 0xFFFFFFFF))
                sflash_write_word(addr, data);
        }
        else
            sflash_write_word(addr, data);  // Otherwise we gotta flash it all


        // original  if (silent_mode)  printf("%4d%%   bytes = %d\r", percent_complete, counter);
        if (silent_mode)  printf("%4d%%   bytes = %d (%08x)@(%08x)=%08x\r", percent_complete, counter, counter, addr, data);
        else              printf("%08x%c", data, (addr&0xF)==0xC?'\n':' ');


        fflush(stdout);
        data = 0xFFFFFFFF;  // This is in case file is shorter than expected length
    }

    fclose(fd);
    printf("Done  (%s loaded into Flash Memory OK)\n\n",filename);

    sflash_reset();


    printf("=========================\n");
    printf("Flashing Routine Complete\n");
    printf("=========================\n");

    time(&end_time);
    elapsed_seconds = difftime(end_time, start_time);
    printf("elapsed time: %d seconds\n", (int)elapsed_seconds);
}

void run_load(char *filename, unsigned int start)
{
    unsigned int addr, data ;
    FILE *fd ;
    int counter = 0;
    int percent_complete = 0;
    unsigned int length = 0;
    time_t start_time = time(0);
    time_t end_time, elapsed_seconds;

    printf("*** You Selected to program the %s ***\n\n",filename);

    fd=fopen(filename, "rb" );
    if (fd<=0)
    {
        fprintf(stderr,"Could not open %s for reading\n", filename);
        exit(1);
    }

    // get file size
    fseek(fd, 0, SEEK_END);
    length = ftell(fd);
    fseek(fd, 0, SEEK_SET);


    printf("===============================\n");
    printf("Programming RAM Routine Started\n");
    printf("===============================\n");

    printf("\nLoading %s to RAM...\n",filename);
    for (addr=start; addr<(start+length); addr+=4)
    {
        counter += 4;
        percent_complete = (counter * 100 / length);
        if (!silent_mode)
            if ((addr&0xF) == 0)  printf("[%3d%%]   %08x: ", percent_complete, addr);

        fread( (unsigned char*) &data, 1,sizeof(data), fd);
        ejtag_write(addr, data);

        if (silent_mode)  printf("%4d%%   bytes = %d (%08x)@(%08x)=%08x\r", percent_complete, counter, counter, addr, data);
        else              printf("%08x%c", data, (addr&0xF)==0xC?'\n':' ');

        fflush(stdout);
        data = 0xFFFFFFFF;  // This is in case file is shorter than expected length
    }
    fclose(fd);
    printf("Done  (%s loaded into Memory OK)\n\n",filename);

    sflash_reset();


    printf("================================\n");
    printf("Programming RAM Routine Complete\n");
    printf("================================\n");

    time(&end_time);
    elapsed_seconds = difftime(end_time, start_time);
    printf("elapsed time: %d seconds\n", (int)elapsed_seconds);

    /*
        printf("Resuming Processor ... ");
        set_instr(INSTR_CONTROL);
        ReadWriteData((PRACC | PROBEN | PROBTRAP) & ~JTAGBRK );
        if (ReadWriteData(PRACC | PROBEN | PROBTRAP) & BRKST)
            printf("<Processor is in the Run Mode!> ... ");
        else
            printf("<Processor is in the Debug Mode!> ... ");

        ReadWriteData(PRRST | PERRST);
        printf("Done\n");
    */
}


void sflash_probe(void)
{
    int retries = 0;


    if (strcasecmp(AREA_NAME,"CUSTOM")==0)
    {
        FLASH_MEMORY_START = selected_window;
    }
    else
    {
        switch (proc_id)
        {

        case IXP425_266:
        case IXP425_400:
            //   case IXP425_533:
            //       FLASH_MEMORY_START = 0x50000000;
            //       break;
        case ARM_940T:
            FLASH_MEMORY_START = 0x00400000;
            break;
        case 0x0635817F:
            FLASH_MEMORY_START = 0x1F000000;
            break;
//    case ATH_PROC:
//        FLASH_MEMORY_START = 0xA8000000;
//        break;

        case 0x0000100F: //Ti AR7
            FLASH_MEMORY_START = 0x90000000;
            break;
        
        default:
            if (flash_size >= size8MB )
            {

                FLASH_MEMORY_START = 0x1c000000;
            }
            else
            {

                FLASH_MEMORY_START = 0x1FC00000;
            }

        }
    }

    printf("\nProbing Flash at (Flash Window: 0x%08x) ... \n", FLASH_MEMORY_START);

again:

    strcpy(flash_part,"");

    // Probe using cmd_type for AMD

    if (strcasecmp(flash_part,"")==0)
    {

        cmd_type = CMD_TYPE_AMD;
        sflash_reset();

        ejtag_write_h(FLASH_MEMORY_START + (0x555 << 1), 0x00AA00AA);
        ejtag_write_h(FLASH_MEMORY_START + (0x2AA << 1), 0x00550055);
        ejtag_write_h(FLASH_MEMORY_START + (0x555 << 1), 0x00900090);
        vendid = ejtag_read_h(FLASH_MEMORY_START);
        devid  = ejtag_read_h(FLASH_MEMORY_START+2);

        if (Flash_DEBUG)
        {
            printf("\nDebug AMD Vendid :    ");
            ShowData (vendid);
            printf("Debug AMD Devdid :    ");
            ShowData (devid);

        }

        identify_flash_part();
    }


    if (strcasecmp(flash_part,"")==0)
    {

        cmd_type = CMD_TYPE_SST;
        sflash_reset();
        ejtag_write_h(FLASH_MEMORY_START + (0x5555 << 1), 0x00AA00AA);
        ejtag_write_h(FLASH_MEMORY_START + (0x2AAA << 1), 0x00550055);
        ejtag_write_h(FLASH_MEMORY_START + (0x5555 << 1), 0x00900090);
        vendid = ejtag_read_h(FLASH_MEMORY_START);
        devid  = ejtag_read_h(FLASH_MEMORY_START+2);

        if (Flash_DEBUG)
        {
            printf("\nDebug SST Vendid :    ");
            ShowData (vendid);
            printf("Debug SST Devdid :    ");
            ShowData (devid);

        }

        identify_flash_part();

    }


    // Probe using cmd_type for BSC & SCS

    if (strcasecmp(flash_part,"")==0)
    {

        cmd_type = CMD_TYPE_BSC;
        sflash_reset();

        ejtag_write_h(FLASH_MEMORY_START, 0x00900090);
        vendid = ejtag_read(FLASH_MEMORY_START);
        devid  = ejtag_read(FLASH_MEMORY_START+2);


        if (Flash_DEBUG)
        {
            printf("\nDebug BSC-SCS Vendid :");
            ShowData (vendid);
            printf("Debug BCS-SCS Devdid :");
            ShowData (devid);

        }

        identify_flash_part();
    }


    if (strcasecmp(flash_part,"")==0)
    {
        int id;

        cmd_type = CMD_TYPE_SPI;

        if (bcmproc)
        {
            id = spiflash_sendcmd(BCM_SPI_RD_ID);
            //id2 = spiflash_sendcmd(BCM_SPI_RD_RES);

        }
        else
            id = spiflash_sendcmd(SPI_RD_ID);

        id <<= 8;
        id = byteSwap_32(id);
        vendid = id >> 16;
        devid  = id & 0x0000ffff;

        if (Flash_DEBUG)
        {
            printf("\nDebug SPI id :    ");
            ShowData (id);
            printf("\nDebug SPI Vendid :    ");
            ShowData (vendid);
            printf("Debug SPI Devdid :    ");
            ShowData (devid);


        }

        identify_flash_part();
    }


    if (strcasecmp(flash_part,"")==0)
    {
        if (retries--)
            goto again;
        else
        {
            printf("Done\n\n");
            printf("*** Unknown or NO Flash Chip Detected ***");
        }
    }
    return;
}

void sflash_poll(unsigned int addr, unsigned int data)
{
    if ((cmd_type == CMD_TYPE_BSC) || (cmd_type == CMD_TYPE_SCS))
    {
        // Wait Until Ready
        while ( (ejtag_read_h(FLASH_MEMORY_START) & STATUS_READY) != STATUS_READY );
    }
    else
    {
        // Wait Until Ready
        while ( (ejtag_read_h(addr) & STATUS_READY) != (data & STATUS_READY) );
    }

}


void sflash_erase_area(unsigned int start, unsigned int length)
{
    int cur_block;
    int tot_blocks;
    unsigned int reg_start;
    unsigned int reg_end;

    reg_start = start;
    reg_end   = reg_start + length;

    tot_blocks = 0;

    for (cur_block = 1;  cur_block <= block_total;  cur_block++)
    {
        block_addr = blocks[cur_block];
        if ((block_addr >= reg_start) && (block_addr < reg_end))  tot_blocks++;
    }

    printf("Total Blocks to Erase: %d\n\n", tot_blocks);

    for (cur_block = 1;  cur_block <= block_total;  cur_block++)
    {
        block_addr = blocks[cur_block];

        if ((block_addr >= reg_start) && (block_addr < reg_end))
        {

            printf("Erasing block: %d (addr = %08x)...", cur_block, block_addr);
            fflush(stdout);
            sflash_erase_block(block_addr);
            printf("Done\n");
            fflush(stdout);
        }
    }

}


void sflash_erase_block(unsigned int addr)
{
    if (cmd_type == CMD_TYPE_SPI)
    {
        spiflash_erase_block(addr);
    }

    if (cmd_type == CMD_TYPE_AMD)
    {

            //Unlock Block
        ejtag_write_h(FLASH_MEMORY_START+(0x555 << 1), 0x00AA00AA);
        ejtag_write_h(FLASH_MEMORY_START+(0x2AA << 1), 0x00550055);
        ejtag_write_h(FLASH_MEMORY_START+(0x555 << 1), 0x00800080);

        //Erase Block
        ejtag_write_h(FLASH_MEMORY_START+(0x555 << 1), 0x00AA00AA);
        ejtag_write_h(FLASH_MEMORY_START+(0x2AA << 1), 0x00550055);
        ejtag_write_h(addr, 0x00300030);


        // Wait for Erase Completion
        sflash_poll(addr, 0xFFFF);

    }

    if (cmd_type == CMD_TYPE_SST)
    {

        //Unlock Block
        ejtag_write_h(FLASH_MEMORY_START+(0x5555 << 1), 0x00AA00AA);
        ejtag_write_h(FLASH_MEMORY_START+(0x2AAA << 1), 0x00550055);
        ejtag_write_h(FLASH_MEMORY_START+(0x5555 << 1), 0x00800080);

        //Erase Block
        ejtag_write_h(FLASH_MEMORY_START+(0x5555 << 1), 0x00AA00AA);
        ejtag_write_h(FLASH_MEMORY_START+(0x2AAA << 1), 0x00550055);
        ejtag_write_h(addr, 0x00500050);

        // Wait for Erase Completion
        sflash_poll(addr, 0xFFFF);

    }

    if ((cmd_type == CMD_TYPE_BSC) || (cmd_type == CMD_TYPE_SCS))
    {

        //Unlock Block
        ejtag_write_h(addr, 0x00500050);     // Clear Status Command
        ejtag_write_h(addr, 0x00600060);     // Unlock Flash Block Command
        ejtag_write_h(addr, 0x00D000D0);     // Confirm Command
        ejtag_write_h(addr, 0x00700070);


        // Wait for Unlock Completion
        sflash_poll(addr, STATUS_READY);

        //Erase Block
        ejtag_write_h(addr, 0x00500050);     // Clear Status Command
        ejtag_write_h(addr, 0x00200020);     // Block Erase Command
        ejtag_write_h(addr, 0x00D000D0);     // Confirm Command
        ejtag_write_h(addr, 0x00700070);

        // Wait for Erase Completion
        sflash_poll(addr, STATUS_READY);

    }

    sflash_reset();

}

void chip_erase(void)
{

    printf("Chip Erase\n");

//    FLASH_MEMORY_START = (0x1fc0000);
    //Unlock Block
    ejtag_write_h(FLASH_MEMORY_START+(0x555 << 1), 0x00AA00AA);
    ejtag_write_h(FLASH_MEMORY_START+(0x2AA << 1), 0x00550055);
    ejtag_write_h(FLASH_MEMORY_START+(0x555 << 1), 0x00800080);

    //Erase Block
    ejtag_write_h(FLASH_MEMORY_START+(0x555 << 1), 0x00AA00AA);
    ejtag_write_h(FLASH_MEMORY_START+(0x2AA << 1), 0x00550055);

    ejtag_write_h(0x1fc00000, 0x00100010);

}

void sflash_reset(void)
{

    if ((cmd_type == CMD_TYPE_AMD) || (cmd_type == CMD_TYPE_SST))
    {

            ejtag_write_h(FLASH_MEMORY_START, 0x00F000F0);    // Set array to read mode
    }

    if ((cmd_type == CMD_TYPE_BSC) || (cmd_type == CMD_TYPE_SCS))
    {
        ejtag_write_h(FLASH_MEMORY_START, 0x00500050);    // Clear CSR
        ejtag_write_h(FLASH_MEMORY_START, 0x00ff00ff);    // Set array to read mode
    }

}

void sflash_write_word(unsigned int addr, unsigned int data)
{
    unsigned int data_lo, data_hi;


    if (USE_DMA)
    {
        // DMA Uses Byte Lanes
        data_lo = data;
        data_hi = data;

    }
    else
    {
        // PrAcc Does Not
        // Speedtouch does not accept flashing with DMA, so you have to use /nodma

        data_lo = (data & 0xFFFF);
        data_hi = ((data >> 16) & 0xFFFF);

    }

    if (cmd_type == CMD_TYPE_SPI)
    {
        spiflash_write_word(addr, data);
    }

    if (cmd_type == CMD_TYPE_AMD)
    {

        if (bypass)
        {
            if (proc_id == 0x00000001)
            {
                ejtag_write_h(FLASH_MEMORY_START+(0x555 << 1), 0x00A000A0);
                ejtag_write_h(addr+2, data_lo);
                tnano(100);


                ejtag_write_h(FLASH_MEMORY_START+(0x555 << 1), 0x00A000A0);
                ejtag_write_h(addr, data_hi);
                tnano(100);
            }
            else
            {

                ejtag_write_h(FLASH_MEMORY_START+(0x555 << 1), 0x00A000A0);
                ejtag_write_h(addr, data_lo);

                ejtag_write_h(FLASH_MEMORY_START+(0x555 << 1), 0x00A000A0);
                ejtag_write_h(addr+2, data_hi);

            }
        }
        else

            if (speedtouch || proc_id == 0x00000001)
            {
                // Speedtouch uses a different flash address pattern.
                // Handle Half Of Word
                ejtag_write_h(FLASH_MEMORY_START+(0x555 << 1), 0x00AA00AA);
                ejtag_write_h(FLASH_MEMORY_START+(0x2AA << 1), 0x00550055);
                ejtag_write_h(FLASH_MEMORY_START+(0x555 << 1), 0x00A000A0);
                ejtag_write_h(addr+2, data_lo);

                // Wait for Completion
                sflash_poll(addr, (data & 0xffff));

                // Now Handle Other Half Of Word
                ejtag_write_h(FLASH_MEMORY_START+(0x555 << 1), 0x00AA00AA);
                ejtag_write_h(FLASH_MEMORY_START+(0x2AA << 1), 0x00550055);
                ejtag_write_h(FLASH_MEMORY_START+(0x555 << 1), 0x00A000A0);
                ejtag_write_h(addr, data_hi);

                // Wait for Completion
                sflash_poll(addr+2, ((data >> 16) & 0xffff));
            }

            else
            {
                ejtag_write_h(FLASH_MEMORY_START+(0x5555 << 1), 0x00AA00AA);
                ejtag_write_h(FLASH_MEMORY_START+(0x2AAA << 1), 0x00550055);
                ejtag_write_h(FLASH_MEMORY_START+(0x5555 << 1), 0x00A000A0);
                ejtag_write_h(addr, data_lo);

                // Wait for Completion
                sflash_poll(addr, (data & 0xffff));

                // Now Handle Other Half Of Word
                ejtag_write_h(FLASH_MEMORY_START+(0x5555 << 1), 0x00AA00AA);
                ejtag_write_h(FLASH_MEMORY_START+(0x2AAA << 1), 0x00550055);
                ejtag_write_h(FLASH_MEMORY_START+(0x5555 << 1), 0x00A000A0);
                ejtag_write_h(addr+2, data_hi);

                // Wait for Completion
                sflash_poll(addr+2, ((data >> 16) & 0xffff));
            }


    }

    if (cmd_type == CMD_TYPE_SST)
    {
        // Handle Half Of Word
        ejtag_write_h(FLASH_MEMORY_START+(0x5555 << 1), 0x00AA00AA);
        ejtag_write_h(FLASH_MEMORY_START+(0x2AAA << 1), 0x00550055);
        ejtag_write_h(FLASH_MEMORY_START+(0x5555 << 1), 0x00A000A0);
        ejtag_write_h(addr, data_lo);

        // Wait for Completion
        sflash_poll(addr, (data & 0xffff));

        // Now Handle Other Half Of Word
        ejtag_write_h(FLASH_MEMORY_START+(0x5555 << 1), 0x00AA00AA);
        ejtag_write_h(FLASH_MEMORY_START+(0x2AAA << 1), 0x00550055);
        ejtag_write_h(FLASH_MEMORY_START+(0x5555 << 1), 0x00A000A0);
        ejtag_write_h(addr+2, data_hi);

        // Wait for Completion
        sflash_poll(addr+2, ((data >> 16) & 0xffff));
    }

    if ((cmd_type == CMD_TYPE_BSC) || (cmd_type == CMD_TYPE_SCS))
    {


        // Handle Half Of Word
        ejtag_write_h(addr, 0x00500050);     // Clear Status Command
        ejtag_write_h(addr, 0x00400040);     // Write Command
        ejtag_write_h(addr, data_lo);        // Send HalfWord Data
//       ejtag_write_h(addr, 0x00700070);     // Check Status Command

        // Wait for Completion
        sflash_poll(addr, STATUS_READY);

        // Now Handle Other Half Of Word
        ejtag_write_h(addr+2, 0x00500050);   // Clear Status Command
        ejtag_write_h(addr+2, 0x00400040);   // Write Command
        ejtag_write_h(addr+2, data_hi);      // Send HalfWord Data
        //     ejtag_write_h(addr+2, 0x00700070);   // Check Status Command


        sflash_poll(addr, STATUS_READY);
    }
}





void show_usage(void)
{

    flash_chip_type*      flash_chip = flash_chip_list;
    processor_chip_type*  processor_chip = processor_chip_list;
    int counter = 0;

    printf( " ABOUT: This program reads/writes flash memory on the WRT54G/GS and\n"
            "        compatible routers via EJTAG using either DMA Access routines\n"
            "        or PrAcc routines (slower/more compatible).  Processor chips\n"
            "        supported in this version include the following chips:\n\n"
            "            Supported Chips\n"
            "            ---------------\n");

    while (processor_chip->chip_id)
    {
        printf("            %-40.40s\n", processor_chip->chip_descr);
        processor_chip++;
    }

    printf( "\n\n");
    printf( " USAGE: tjtag [parameter] </noreset> </noemw> </nocwd> </nobreak> </noerase>\n"
            "                      </notimestamp> </dma> </nodma>\n"
            "                      <start:XXXXXXXX> </length:XXXXXXXX>\n"
            "                      </silent> </skipdetect> </instrlen:XX> </fc:XX> /bypass /st5\n\n"

            "            Required Parameter\n"
            "            ------------------\n"
            "            -backup:cfe\n"
            "            -backup:nvram\n"
            "            -backup:kernel\n"
            "            -backup:wholeflash\n"
            "            -backup:custom\n"
            "            -backup:bsp\n"
            "            -erase:cfe\n"
            "            -erase:nvram\n"
            "            -erase:kernel\n"
            "            -erase:wholeflash\n"
            "            -erase:custom\n"
            "            -erase:bsp\n"
            "            -flash:cfe\n"
            "            -flash:nvram\n"
            "            -flash:kernel\n"
            "            -flash:wholeflash\n"
            "            -flash:custom\n"
            "            -flash:bsp\n"
            "            -probeonly\n"
            "            -probeonly:custom\n"
            " and for Ti AR7 \n"
            "            -backup:mtd2\n"
            "            -backup:mtd3\n"
            "            -backup:mtd4\n"
            "            -backup:full\n"
            "            -erase:mtd2\n"
            "            -erase:mtd3\n"
            "            -erase:mtd4\n"
            "            -erase:full\n"
            "            -flash:mtd2\n"
            "            -flash:mtd3\n"
            "            -flash:mtd4\n"
            "            -flash:full\n\n"
            
	        "		 Optional with -backup:, -erase:, -flash: wgrv8bdata, wgrv9bdata, cfe128\n\n"

            "            Optional Switches\n"
            "            -----------------\n"
            "            /noreset ........... prevent Issuing EJTAG CPU reset\n"
            "            /noemw ............. prevent Enabling Memory Writes\n"
            "            /nocwd ............. prevent Clearing CPU Watchdog Timer\n"
            "            /nobreak ........... prevent Issuing Debug Mode JTAGBRK\n"
            "            /noerase ........... prevent Forced Erase before Flashing\n"
            "            /notimestamp ....... prevent Timestamping of Backups\n"
            "            /dma ............... force use of DMA routines\n"
            "            /nodma ............. force use of PRACC routines (No DMA)\n"
            "            /window:XXXXXXXX ... custom flash window base (in HEX)\n"
            "            /start:XXXXXXXX .... custom start location (in HEX)\n"
            "            /length:XXXXXXXX ... custom length (in HEX)\n"
            "            /silent ............ prevent scrolling display of data\n"
            "            /skipdetect ........ skip auto detection of CPU Chip ID\n"
            "            /instrlen:XX ....... set instruction length manually\n"
            "            /wiggler ........... use wiggler cable\n"
            "            /bypass ............ Unlock Bypass command & disable polling\n"
            "            /delay:XXXXXX ...... add delay to communication\n"
            "            /st5 ............... Use Speedtouch ST5xx flash routines instead of WRT routines\n"
            "            /reboot............. sets the process and reboots\n"
	        "		 /swap_endian........ swap endianess during backup - most Atheros based routers\n"
	        "		 /flash_debug........ flash chip debug messages, show flash MFG and Device ID\n\n"

            "            /fc:XX = Optional (Manual) Flash Chip Selection\n"
            "            -----------------------------------------------\n");

    while (flash_chip->vendid)
    {
        printf("            /fc:%02d ............. %-40.40s\n", ++counter, flash_chip->flash_part);
        flash_chip++;
    }

    printf( "\n\n");
    printf( " NOTES: 1) If 'flashing' - the source filename must exist as follows:\n"
            "           CFE.BIN, NVRAM.BIN, KERNEL.BIN, WHOLEFLASH.BIN or CUSTOM.BIN\n"
            "           BSP.BIN\n\n"
            "...........or for Ti AR7 MTD2.BIN, MTD3.BIN\n\n"

            "        2) If you have difficulty auto-detecting a particular flash part\n"
            "           you can manually specify your exact part using the /fc:XX option.\n\n"

            "        3) If you have difficulty with the older bcm47xx chips or when no CFE\n"
            "           is currently active/operational you may want to try both the\n"
            "           /noreset and /nobreak command line options together.  Some bcm47xx\n"
            "           chips *may* always require both these options to function properly.\n\n"

            "        4) When using this utility, usually it is best to type the command line\n"
            "           out, then plug in the router, and then hit <ENTER> quickly to avoid\n"
            "           the CPUs watchdog interfering with the EJTAG operations.\n\n"

            "        5) /bypass - enables Unlock bypass command for some AMD/Spansion type\n"
            "           flashes, it also disables polling\n\n"

            " ***************************************************************************\n"
            " * Flashing the KERNEL or WHOLEFLASH will take a very long time using JTAG *\n"
            " * via this utility.  You are better off flashing the CFE & NVRAM files    *\n"
            " * & then using the normal TFTP method to flash the KERNEL via ethernet.   *\n"
            " ***************************************************************************\n\n");
}


int main(int argc, char** argv)
{
    char choice[128];
    int run_option;
    int j;

    printf("\n");
    printf("==============================================\n");
    printf(" EJTAG Debrick Utility v3.0.1 Tornado-MOD \n");
    printf("==============================================\n\n");



    if (argc < 2)
    {
        show_usage();
        exit(1);
    }

    strcpy(choice,argv[1]);

    run_option = 0;

    if (strcasecmp(choice,"-backup:cfe")==0)
    {
        run_option = 1;
        strcpy(AREA_NAME, "CFE");
    }
    if (strcasecmp(choice,"-backup:cf1")==0)
    {
        run_option = 1;
        strcpy(AREA_NAME, "CF1");
    }
    if (strcasecmp(choice,"-backup:cfe128")==0)
    {
        run_option = 1;
        strcpy(AREA_NAME, "CFE128");
    }

    if (strcasecmp(choice,"-backup:nvram")==0)
    {
        run_option = 1;
        strcpy(AREA_NAME, "NVRAM");
    }
    if (strcasecmp(choice,"-backup:kernel")==0)
    {
        run_option = 1;
        strcpy(AREA_NAME, "KERNEL");
    }
    if (strcasecmp(choice,"-backup:wholeflash")==0)
    {
        run_option = 1;
        strcpy(AREA_NAME, "WHOLEFLASH");
    }
    if (strcasecmp(choice,"-backup:custom")==0)
    {
        run_option = 1;
        strcpy(AREA_NAME, "CUSTOM");
        custom_options++;
    }
    if (strcasecmp(choice,"-backup:bsp")==0)
    {
        run_option = 1;
        strcpy(AREA_NAME, "BSP");
    }
    if (strcasecmp(choice,"-backup:red")==0)
    {
        run_option = 1;
        strcpy(AREA_NAME, "RED");
    }
    if (strcasecmp(choice,"-backup:wgrv8bdata")==0)
    {
        run_option = 1;
        strcpy(AREA_NAME, "WGRV8BDATA");
    }
    if (strcasecmp(choice,"-backup:wgrv9bdata")==0)
    {
        run_option = 1;
        strcpy(AREA_NAME, "WGRV9BDATA");
    }
    if (strcasecmp(choice,"-erase:cfe")==0)
    {
        run_option = 2;
        strcpy(AREA_NAME, "CFE");
    }
    if (strcasecmp(choice,"-erase:wgrv9bdata")==0)
    {
        run_option = 2;
        strcpy(AREA_NAME, "WGRV9BDATA");
    }
    if (strcasecmp(choice,"-erase:wgrv9nvram")==0)
    {
        run_option = 2;
        strcpy(AREA_NAME, "WGRV9NVRAM");
    }
    if (strcasecmp(choice,"-erase:wgrv8bdata")==0)
    {
        run_option = 2;
        strcpy(AREA_NAME, "WGRV8BDATA");
    }
    if (strcasecmp(choice,"-erase:cf1")==0)
    {
        run_option = 2;
        strcpy(AREA_NAME, "CF1");
    }
    if (strcasecmp(choice,"-erase:cfe128")==0)
    {
        run_option = 2;
        strcpy(AREA_NAME, "CFE128");
    }
    if (strcasecmp(choice,"-erase:nvram")==0)
    {
        run_option = 2;
        strcpy(AREA_NAME, "NVRAM");
    }
    if (strcasecmp(choice,"-erase:kernel")==0)
    {
        run_option = 2;
        strcpy(AREA_NAME, "KERNEL");
    }
    if (strcasecmp(choice,"-erase:wholeflash")==0)
    {
        run_option = 2;
        strcpy(AREA_NAME, "WHOLEFLASH");
    }
    if (strcasecmp(choice,"-erase:custom")==0)
    {
        run_option = 2;
        strcpy(AREA_NAME, "CUSTOM");
        custom_options++;
    }
    if (strcasecmp(choice,"-erase:bsp")==0)
    {
        run_option = 2;
        strcpy(AREA_NAME, "BSP");
    }
    if (strcasecmp(choice,"-spi_chiperase")==0)
    {
        run_option = 6;

    }
    if (strcasecmp(choice,"-erase:red")==0)
    {
        run_option = 2;
        strcpy(AREA_NAME, "RED");
    }
    if (strcasecmp(choice,"-flash:cfe")==0)
    {
        run_option = 3;
        strcpy(AREA_NAME, "CFE");
    }
    if (strcasecmp(choice,"-flash:cf1")==0)
    {
        run_option = 3;
        strcpy(AREA_NAME, "CF1");
    }
    if (strcasecmp(choice,"-flash:cfe128")==0)
    {
        run_option = 3;
        strcpy(AREA_NAME, "CFE128");
    }
    if (strcasecmp(choice,"-flash:nvram")==0)
    {
        run_option = 3;
        strcpy(AREA_NAME, "NVRAM");
    }
    if (strcasecmp(choice,"-flash:wgrv8bdata")==0)
    {
        run_option = 3;
        strcpy(AREA_NAME, "WGRV8BDATA");
    }
    if (strcasecmp(choice,"-flash:wgrv9bdata")==0)
    {
        run_option = 3;
        strcpy(AREA_NAME, "WGRV9BDATA");
    }
    if (strcasecmp(choice,"-flash:kernel")==0)
    {
        run_option = 3;
        strcpy(AREA_NAME, "KERNEL");
    }
    if (strcasecmp(choice,"-flash:wholeflash")==0)
    {
        run_option = 3;
        strcpy(AREA_NAME, "WHOLEFLASH");
    }
    if (strcasecmp(choice,"-flash:custom")==0)
    {
        run_option = 3;
        strcpy(AREA_NAME, "CUSTOM");
        custom_options++;
    }
    if (strcasecmp(choice,"-flash:bsp")==0)
    {
        run_option = 3;
        strcpy(AREA_NAME, "BSP");
    }

    if (strcasecmp(choice,"-flash:red")==0)
    {
        run_option = 3;
        strcpy(AREA_NAME, "RED");
    }
    if (strcasecmp(choice,"-probeonly")==0)
    {
        run_option = 4;
    }
    if (strcasecmp(choice,"-probeonly:custom")==0)
    {
        run_option = 4;
        strcpy(AREA_NAME, "CUSTOM");
    }

    if (strncasecmp(choice,"-load:", 5)==0)
    {
        run_option = 5;
        strcpy(AREA_NAME, &choice[6]);
    }

/* Extras for AR7 */
    if (strcasecmp(choice,"-backup:mtd2")==0)        { run_option = 1;  strcpy(AREA_NAME, "MTD2");       }
    if (strcasecmp(choice,"-backup:mtd3")==0)        { run_option = 1;  strcpy(AREA_NAME, "MTD3");       }
    if (strcasecmp(choice,"-backup:mtd4")==0)        { run_option = 1;  strcpy(AREA_NAME, "MTD4");       }
    if (strcasecmp(choice,"-backup:full")==0)        { run_option = 1;  strcpy(AREA_NAME, "FULL");       }
    if (strcasecmp(choice,"-erase:mtd2")==0)         { run_option = 2;  strcpy(AREA_NAME, "MTD2");       }
    if (strcasecmp(choice,"-erase:mtd3")==0)         { run_option = 2;  strcpy(AREA_NAME, "MTD3");       }
    if (strcasecmp(choice,"-erase:mtd4")==0)         { run_option = 2;  strcpy(AREA_NAME, "MTD4");       }
    if (strcasecmp(choice,"-erase:full")==0)         { run_option = 2;  strcpy(AREA_NAME, "FULL");       }
    if (strcasecmp(choice,"-flash:mtd2")==0)         { run_option = 3;  strcpy(AREA_NAME, "MTD2");       }
    if (strcasecmp(choice,"-flash:mtd3")==0)         { run_option = 3;  strcpy(AREA_NAME, "MTD3");       }
    if (strcasecmp(choice,"-flash:mtd4")==0)         { run_option = 3;  strcpy(AREA_NAME, "MTD4");       }
    if (strcasecmp(choice,"-flash:full")==0)         { run_option = 3;  strcpy(AREA_NAME, "FULL");       }
/* end AR7 */

    if (run_option == 0)
    {
        show_usage();
        printf("\n*** ERROR - Invalid [option] specified ***\n\n");
        exit(1);
    }

    if (argc > 2)
    {
        j = 2;
        while (j < argc)
        {
            strcpy(choice,argv[j]);

            if (strcasecmp(choice,"/noreset")==0)              issue_reset = 0;
            else if (strcasecmp(choice,"/noemw")==0)           issue_enable_mw = 0;
            else if (strcasecmp(choice,"/nocwd")==0)           issue_watchdog = 0;
            else if (strcasecmp(choice,"/nobreak")==0)         issue_break = 0;
            else if (strcasecmp(choice,"/noerase")==0)         issue_erase = 0;
            else if (strcasecmp(choice,"/notimestamp")==0)     issue_timestamp = 0;
            else if (strcasecmp(choice,"/dma")==0)             force_dma = 1;
            else if (strcasecmp(choice,"/nodma")==0)           force_nodma = 1;
            else if (strncasecmp(choice,"/fc:",4)==0)          selected_fc = strtoul(((char *)choice + 4),NULL,10);
            else if (strcasecmp(choice,"/bypass")==0)          bypass = 1;
            else if (strcasecmp(choice, "/reboot")==0)         issue_reboot = 1;
            else if (strncasecmp(choice,"/window:",8)==0)
            {
                selected_window = strtoul(((char *)choice + 8),NULL,16);
                custom_options++;
                probe_options++;
            }
            else if (strncasecmp(choice,"/start:",7)==0)
            {
                selected_start  = strtoul(((char *)choice + 7),NULL,16);
                custom_options++;
            }
            else if (strncasecmp(choice,"/length:",8)==0)
            {
                selected_length = strtoul(((char *)choice + 8),NULL,16);
                custom_options++;
            }
            else if (strcasecmp(choice,"/silent")==0)          silent_mode = 1;
            else if (strcasecmp(choice,"/skipdetect")==0)      skipdetect = 1;
            else if (strncasecmp(choice,"/instrlen:",10)==0)   instrlen = strtoul(((char *)choice + 10),NULL,10);
            else if (strcasecmp(choice,"/wiggler")==0)         wiggler = 1;
            else if (strcasecmp(choice,"/st5")==0)			   speedtouch = 1;
            else if (strcasecmp(choice,"/flash_debug")==0)     Flash_DEBUG = 1;
            else if (strncasecmp(choice,"/delay:",7)==0)       delay = strtoul(((char *)choice + 7),NULL,10);
            else if (strcasecmp(choice,"/xbit")==0)            xbit = 1;
            else if (strcasecmp(choice,"/swap_endian")==0)      swap_endian = 1;
            else
            {
                show_usage();
                printf("\n*** ERROR - Invalid <option> specified ***\n\n");
                exit(1);
            }

            j++;
        }
    }

    if (strcasecmp(AREA_NAME,"CUSTOM")==0)
    {
        if ((run_option != 4) && (custom_options != 0) && (custom_options != 4))
        {
            show_usage();
            printf("\n*** ERROR - 'CUSTOM' also requires '/window' '/start' and '/length' options ***\n\n");
            exit(1);
        }

        if ((run_option == 4) && (probe_options != 1))
        {
            show_usage();
            printf("\n*** ERROR - 'PROBEONLY:CUSTOM' requires '/window' option ***\n\n");
            exit(1);
        }
    }


    // ----------------------------------
    // Detect CPU
    // ----------------------------------

    chip_detect();


    // ----------------------------------
    // Find Implemented EJTAG Features
    // ----------------------------------
    check_ejtag_features();



    // ----------------------------------
    // Reset State Machine For Good Measure
    // ----------------------------------
    test_reset();


    printf("Issuing Processor / Peripheral Reset ... ");
    if (issue_reset)
    {
        if ((proc_id & 0xfffffff) == 0x535417f)
        {
        set_instr(INSTR_CONTROL);
        WriteData(PRRST | PERRST);
        WriteData(0);
        printf("Done\n");
        }
        else
        {
        set_instr(INSTR_CONTROL);
        ctrl_reg = ReadWriteData(PRRST | PERRST);
        printf("Done\n");
        }

     }
    else printf("Skipped\n");

    // ----------------------------------
    // Enable Memory Writes
    // ----------------------------------
    // Always skip for EJTAG versions 2.5 and 2.6 since they do not support DMA transactions.
    // Memory Protection bit only applies to EJTAG 2.0 based chips.

    if (ejtag_version != 0)  issue_enable_mw = 0;
    printf("Enabling Memory Writes ... ");
    if (issue_enable_mw)
    {
        // Clear Memory Protection Bit in DCR
        ejtag_dma_write(0xff300000, (ejtag_dma_read(0xff300000) & ~(1<<2)) );
        printf("Done\n");
    }
    else printf("Skipped\n");


    // ----------------------------------
    // Put into EJTAG Debug Mode
    // ----------------------------------
    printf("Halting Processor ... ");
    if (issue_break)
    {
        set_instr(INSTR_CONTROL);
        ctrl_reg = ReadWriteData(PRACC | PROBEN | SETDEV | JTAGBRK );
        if (ReadWriteData(PRACC | PROBEN | SETDEV) & BRKST)
            printf("<Processor Entered Debug Mode!> ... ");
        else
            printf("<Processor did NOT enter Debug Mode!> ... ");
        printf("Done\n");
    }
    else printf("Skipped\n");


    // ----------------------------------
    // Clear Watchdog
    // ----------------------------------

    printf("Clearing Watchdog ... ");

    if (issue_watchdog)
    {
        if (proc_id == 0x00000001)
        {

            ejtag_write(0xbc003000, 0xffffffff);

            ejtag_write(0xbc003008, 0); // Atheros AR5312

            ejtag_write(0xbc004000, 0x05551212);

            printf("Done\n");
        }
        else
        {
            ejtag_write(0xb8000080,0);
            printf("Done\n");
        }
    }
    else printf("Skipped\n");


    isbrcm();
    //mscan();

    //------------------------------------
    // Enable Flash Read/Write for Atheros
    //------------------------------------

    if (proc_id == 0x00000001)
    {
        printf("\nEnabling Atheros Flash Read/Write ... ");
        //ejtag_write(0xb8400000, 0x000e3ce1); //8bit
        ejtag_write(0xb8400000, 0x100e3ce1); //16bit
        printf("Done\n");
    }


    //----------------------------------------------------------------
    // Enable Flash Read/Write for Atheros and check Revision Register
    //
    // Ejtag IDCODE does not tell us what Atheros Processor it has found..
    // so lets try to detect via the Revision Register


    if (proc_id == 0x00000001)
    {
        printf("\n.RE-Probing Atheros processor....");

        uint32_t ARdevid;

        ARdevid = (ejtag_read(AR5315_SREV) &AR5315_REV_MAJ) >> AR5315_REV_MAJ_S;


        switch (ARdevid)
        {
        case 0x9:
            printf("\n..Found a Atheros AR2317\n");
            break;
            /* FIXME: how can we detect AR2316? */
        case 0x8:
            printf("\n..Found a Atheros AR2316\n");
            break;
        default:
            //mips_machtype = ATHEROS AR2315;
            break;
        }

    }

    // ----------------------------------
    // Flash Chip Detection
    // ----------------------------------
    if (selected_fc != 0)
        sflash_config();
    else
        sflash_probe();


    // ----------------------------------
    // Execute Requested Operation
    // ----------------------------------

    if ((flash_size > 0) && (AREA_LENGTH > 0))
    {
        if (run_option == 1 )  run_backup(AREA_NAME, AREA_START, AREA_LENGTH);
        if (run_option == 2 )  run_erase(AREA_NAME, AREA_START, AREA_LENGTH);
        if (run_option == 3 )  run_flash(AREA_NAME, AREA_START, AREA_LENGTH);
        //  if (run_option == 4 ) {};  // Probe was already run so nothing else needed
    }

    if (run_option == 5 )  run_load(AREA_NAME, 0x80040000);
    if (run_option == 6 )  spi_chiperase(0x1fc00000);


    printf("\n\n *** REQUESTED OPERATION IS COMPLETE ***\n\n");


    if (issue_reboot)
    {
        printf("Reset Processor ...\n");

        ExecuteDebugModule(pracc_read_depc);
        printf("DEPC: 0x%08x\n", data_register);

        address_register = 0xA0000000;
        data_register    = 0xBFC00000;
        //data_register    -= 4;
        ExecuteDebugModule(pracc_write_depc);

        // verify my return address
        ExecuteDebugModule(pracc_read_depc);
        printf("DEPC: 0x%08x\n", data_register);

    }


    if (proc_id == 0x00000001)
    {

        printf("Resuming Processor ...\n");
        return_from_debug_mode();

        set_instr(INSTR_RESET);
        test_reset();

        set_instr(INSTR_CONTROL);
        ctrl_reg = ReadWriteData(PRRST | PERRST);
        printf(" ECR: 0x%08x\n", ctrl_reg);
    }

    chip_shutdown();

    return 0;
}


// **************************************************************************
// End of File
// **************************************************************************
