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

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>

#ifndef WINDOWS_VERSION

#include <unistd.h>
#include <sys/ioctl.h>

#ifdef __FreeBSD__
#include <dev/ppbus/ppi.h>
#include <dev/ppbus/ppbconf.h>
#define PPWDATA PPISDATA
#define PPRSTATUS PPIGSTATUS
#else
#include <linux/ppdev.h>
#endif
#endif
/*
#define uint32			        unsigned int
#define uint16                  unsigned short
#define uint8                   unsigned char
*/

#ifdef RASPPI
   #include <sys/mman.h>
#endif

#define TRUE  1
#define FALSE 0

#define DATA_PORT 0x378
#define STATUS_PORT 0x378+1


#define WORD unsigned short
#define BYTE unsigned char
#define DWORD unsigned int

#define BCM15354 0x1535417F
#define BCM25354 0x2535417F
#define BCM35354 0x3535417F
#define BRCM_FLASH_CONFIG   0x18000128

/* IXP4XX ----------------------------------------- */
#define IXP425_266  0x19277013 // XScale IXP42X 266mhz
#define IXP425_400  0x19275013 // XScale IXP42X 400mhz
#define IXP425_533  0x19274013 // XScale IXP42X 533mhz
#define ARM_940T    0x10940027 // ARM 940T
#define ATH_PROC    0x00000001
#define IXP4XX_EXP_BUS_CS_EN		(1L << 31)

#define TEST    0x0001

#define RETRY_ATTEMPTS 16

/*
kuseg   0x00000000 - 0x7fffffff  User virtual mem,  mapped
kseg0   0x80000000 - 0x9fffffff  Physical memory, cached, unmapped
kseg1   0xa0000000 - 0xbfffffff  Physical memory, uncached, unmapped
kseg2   0xc0000000 - 0xffffffff  kernel-virtual,  mapped

#define	MIPS_KUSEG_START		0x0
#define	MIPS_KSEG0_START		0x80000000
#define	MIPS_KSEG1_START		0xa0000000
#define	MIPS_KSEG2_START		0xc0000000
#define	MIPS_MAX_MEM_ADDR		0xbe000000
#define	MIPS_RESERVED_ADDR		0xbfc80000

#define	MIPS_PHYS_MASK			0x1fffffff


*/
#ifdef RASPPI

   // --- Pi Cable ---
   #define TDI     24
   #define TCK     22
   #define TMS     23
   #define TDO     17

   // --- Pi GPIO ---
   #define BCM2708_PERI_BASE  0x20000000
   #define GPIO_BASE          (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */
   #define PAGE_SIZE          (4 * 1024)
   #define BLOCK_SIZE         (4 * 1024)

#else

   // --- Xilinx Type Cable ---

   #define TDI     0
   #define TCK     1
   #define TMS     2
   #define TDO     4

   // ---Wiggler Type Cable ---

   #define WTDI      3
   #define WTCK      2
   #define WTMS      1
   #define WTDO      7
   #define WTRST_N   4
   #define WSRST_N   0

#endif

#define JTAG_ENABLE 0x10
#define TDOMASK 0x10
#define TDOHIGH 0x10
#define TCK_MASK  0x02
#define TMS_MASK  0x04
#define TDI_MASK  0x01

// --- Some EJTAG Instruction Registers ---
#define INSTR_EXTEST    0x00
#define INSTR_IDCODE    0x01
#define INSTR_SAMPLE    0x02
#define INSTR_IMPCODE   0x03
#define INSTR_ADDRESS   0x08
#define INSTR_DATA      0x09
#define INSTR_CONTROL   0x0A
#define INSTR_ALL       0x0B
#define INSTR_EJTAGBOOT 0x0C
#define INSTR_RESET     0x0D     // makes the processor execute the reset handler after reset
#define INSTR_FASTDATA		0x0E
#define INSTR_TCBCONTROLA	0x10
#define INSTR_TCBCONTROLB	0x11
#define INSTR_TCBDATA		0x12
#define INSTR_EJWATCH   0x1C
#define INSTR_BYPASS    0xFF

#define INSTR_DCSR       0x09 //0b01001
#define INSTR_BS_EXTEST  0x00
#define INSTR_DBGRX      0x02
#define INSTR_DBGTX      0x10
#define INSTR_BS_CLAMPZ  0x49
#define INSTR_HIGHZ      0x4a
#define INSTR_IXSAMPLE   0x01
#define INSTR_IXBYPASS   0x7f
#define INSTR_LDIC       0x07
#define INSTR_XIDCODE    0x7e


// --- Some EJTAG Bit Masks ---
#define TOF             (1 << 1 )
#define TIF             (1 << 2 )
#define BRKST           (1 << 3 )
#define DLOCK           (1 << 5 )
#define DRWN            (1 << 9 )
#define DERR            (1 << 10)
#define DSTRT           (1 << 11)
#define JTAGBRK         (1 << 12)
#define SETDEV          (1 << 14)
#define PROBTRAP        (1 << 14)
#define PROBEN          (1 << 15)
#define PRRST           (1 << 16)
#define DMAACC          (1 << 17)
#define PRACC           (1 << 18)
#define PRNW            (1 << 19)
#define PERRST          (1 << 20)
#define HALT            (1 << 21)
#define DOZE            (1 << 22)
#define SYNC            (1 << 23)
#define DNM             (1 << 28)
#define ROCC            (1 << 31)

#define IXP42X_HALT     (1 << 30)
#define IXP42X_GE       (1 << 31)

/* Debug Register (CP0 Register 23, Select 0) */

#define DEBUG_DSS			(1 << 0)
#define DEBUG_DBP			(1 << 1)
#define DEBUG_DDBL  		(1 << 2)
#define DEBUG_DDBS  		(1 << 3)
#define DEBUG_DIB			(1 << 4)
#define DEBUG_DINT  		(1 << 5)
#define DEBUG_OFFLINE		(1 << 7)
#define DEBUG_SST			(1 << 8)
#define DEBUG_NOSST 		(1 << 9)
#define DEBUG_DDBLIMPR  	(1 << 18)
#define DEBUG_DDBSIMPR  	(1 << 19)
#define DEBUG_IEXI  		(1 << 20)
#define DEBUG_DBUSEP		(1 << 21)
#define DEBUG_CACHEEP		(1 << 22)
#define DEBUG_MCHECKP		(1 << 23)
#define DEBUG_IBUSEP		(1 << 24)
#define DEBUG_COUNTDM		(1 << 25)
#define DEBUG_HALT  		(1 << 26)
#define DEBUG_DOZE  		(1 << 27)
#define DEBUG_LSNM  		(1 << 28)
#define DEBUG_NODCR 		(1 << 29)
#define DEBUG_DM			(1 << 30)
#define DEBUG_DBD			(1 << 31)

/* implementaion register bits */
#define IMP_NODMA			(1 << 14)
#define IMP_MIPS16  		(1 << 16)


#define DMA_BYTE        0x00000000  //DMA tranfser size BYTE
#define DMA_HALFWORD    0x00000080  //DMA transfer size HALFWORD
#define DMA_WORD        0x00000100  //DMA transfer size WORD
#define DMA_TRIPLEBYTE  0x00000180  //DMA transfer size TRIPLEBYTE

#define  size4K        0x1000
#define  sizeA4K       0x1080
#define  size8K        0x2000
#define  size16K       0x4000
#define  size32K       0x8000
#define  size64K       0x10000
#define  size128K      0x20000
#define  size256K      0x40000
#define  size512K      0x80000

#define  size1MB       0x100000
#define  size2MB       0x200000
#define  size4MB       0x400000
#define  size8MB       0x800000
#define  size16MB      0x1000000
#define  size32MB      0x2000000
#define  size64MB      0x4000000
#define  size128MB     0x8000000
#define  size256MB     0x10000000
#define  size512MB     0x20000000

#define  CMD_TYPE_BSC  0x01
#define  CMD_TYPE_SCS  0x02
#define  CMD_TYPE_AMD  0x03
#define  CMD_TYPE_SST  0x04
#define  CMD_TYPE_SPI  0x05

#define  STATUS_READY  0x0080
#define MaxIR_ChainLength 1000

// EJTAG DEBUG Unit Vector on Debug Break
#define MIPS_DEBUG_VECTOR_ADDRESS           0xFF200200
// Our 'Pseudo' Virtual Memory Access Registers
#define MIPS_VIRTUAL_ADDRESS_ACCESS         0xFF200000
#define MIPS_VIRTUAL_DATA_ACCESS            0xFF200004

// Bit PRNW is not implemented into the all processors with EJTAG ver. < 2.5 ( for example BCM5354 rev.3 is not ).
// Therefore added new address 0xFF200008 for the story data and modified all read debug modules
#define MIPS_VIRTUAL_DATA_STORY_ACCESS		0xFF200008



/* breakpoint support */
#define EJTAG_DCR				0xFF300000
#define EJTAG_IBS				0xFF301000
#define EJTAG_IBA1				0xFF301100
#define EJTAG_DBS				0xFF302000
#define EJTAG_DBA1				0xFF302100


#define dMIPS32_ECR_CPU_RESET_OCCURED  0x80000000 // Set to 1 if a reset occured
#define dMIPS32_ECR_ACCESS_SIZE        0x60000000 // See below note 1
#define dMIPS32_ECR_DOZE               0x00400000 // Set to 1 if processor is in low power mode
#define dMIPS32_ECR_HALT               0x00200000 // Set to 1 if system bus clock is stopped
#define dMIPS32_ECR_PERIPHERAL_RESET   0x00100000 // Set to 1 to reset peripherals (optional)
#define dMIPS32_ECR_ACCESS_RW          0x00080000 // Set to 1 if pending access is write, 0 for read
#define dMIPS32_ECR_ACCESS_PENDING     0x00040000 // Set to 1 to indicate access pending
#define dMIPS32_ECR_CPU_RESET          0x00010000 // Set to 1 to reset CPU (optional)
#define dMIPS32_ECR_PROBE_ENABLE       0x00008000 // Set to 1 to enable remote hosted memory
#define dMIPS32_ECR_PROBE_VECTOR       0x00004000 // Set to 1 to use dmseg location for remote memory
#define dMIPS32_ECR_EJTAG_BREAK        0x00001000 // Set to 1 to request a debug interrupt exception
#define dMIPS32_ECR_IN_DEBUG_MODE      0x00000008 // Set to 1 to indicate in debug mode

// Note 1


//#define AR5315_DSLBASE          0xB1000000      /* RESET CONTROL MMR */
#define AR5315_SREV             (AR5315_DSLBASE + 0x0014)
#define AR5315_REV_MAJ          0x00f0
#define AR5315_REV_MAJ_S        4

//Redboot says
/*
 * Revision Register - Initial value is 0x3010 (WMAC 3.0, AR531X 1.0).
 */
#define AR2316_DSLBASE          0xB1000000      /* RESET CONTROL MMR */
#define AR2316_SREV             (AR2316_DSLBASE + 0x0014)

#define AR531X_REV      (AR531X_RESETTMR + 0x0090)
#define REV_MAJ         0x00f0
#define REV_MAJ_S       4
#define REV_MIN         0x000f
#define REV_MIN_S       0
#define REV_CHIP        (REV_MAJ|REV_MIN)


/* Major revision numbers, bits 7..4 of Revision ID register */
#define REV_MAJ_AR5311              0x01
#define REV_MAJ_AR5312              0x04
#define REV_MAJ_AR5315              0x0B

#define AR531X_REV_MAJ_AR2313          0x5


struct STAT_REG_BITS
{

unsigned stat_r:
    1;
unsigned stat_bls:
    1;
unsigned stat_pss:
    1;
unsigned stat_vpps:
    1;
unsigned stat_ps:
    1;
unsigned stat_es:
    1;
unsigned stat_ess:
    1;
unsigned stat_wsms:
    1;

};


/* AR2313 has CPU minor rev. 10 */
/*if ((current_cpu_data.processor_id & 0xff) == 0x0a)
	mips_machtype = MACH_ATHEROS_AR2313;
*/

// --- Uhh, Just Because I Have To ---
void sflash_write_byte(unsigned int addr, unsigned int data);
void chip_detect(void);
void chip_shutdown(void);
static unsigned char clockin(int tms, int tdi);
void define_block(unsigned int block_count, unsigned int block_size);
static unsigned int ejtag_read(unsigned int addr);
static unsigned int ejtag_read_h(unsigned int addr);
//static unsigned int ejtag_read_b(unsigned int addr);
void ejtag_write(unsigned int addr, unsigned int data);
void ejtag_write_h(unsigned int addr, unsigned int data);
void ejtag_write_b(unsigned int addr, unsigned int data);
static unsigned int ejtag_dma_read(unsigned int addr);
static unsigned int ejtag_dma_read_h(unsigned int addr);
//static unsigned int ejtag_dma_read_b(unsigned int addr);
void ejtag_dma_write(unsigned int addr, unsigned int data);
void ejtag_dma_write_h(unsigned int addr, unsigned int data);
void ejtag_dma_write_b(unsigned int addr, unsigned int data);
static unsigned int ejtag_pracc_read(unsigned int addr);
static unsigned int ejtag_pracc_read_h(unsigned int addr);
//static unsigned int ejtag_pracc_read_b(unsigned int addr);
void ejtag_pracc_write(unsigned int addr, unsigned int data);
void ejtag_pracc_write_h(unsigned int addr, unsigned int data);
void ejtag_pracc_write_b(unsigned int addr, unsigned int data);
void identify_flash_part(void);
void lpt_closeport(void);
void lpt_openport(void);
static unsigned int ReadData(void);
static unsigned int ReadWriteData(unsigned int in_data);
void run_backup(char *filename, unsigned int start, unsigned int length);
void run_erase(char *filename, unsigned int start, unsigned int length);
void run_flash(char *filename, unsigned int start, unsigned int length);
void set_instr(int instr);
void sflash_config(void);
void sflash_erase_area(unsigned int start, unsigned int length);
void sflash_erase_block(unsigned int addr);
void sflash_probe(void);
void sflash_reset(void);
void sflash_write_word(unsigned int addr, unsigned int data);
void show_usage(void);
void ShowData(unsigned int value);
void test_reset(void);
void WriteData(unsigned int in_data);
void ExecuteDebugModule(unsigned int *pmodule);
void check_ejtag_features(void);
void unlock_bypass(void);
void unlock_bypass_reset(void);
void spi_fast(unsigned int addr);
void cable_wait( void );


unsigned int pracc_readbyte_code_module[] =
{
    // #
    // # HairyDairyMaid's Assembler PrAcc Read Byte Routine
    // #
    // start:
    //
    // # Load R1 with the address of the pseudo-address register
    0x3C01FF20,  // lui $1,  0xFF20
    0x34210000,  // ori $1,  0x0000
    //
    // # Load R2 with the address for the read
    0x8C220000,  // lw $2,  ($1)
    //
    // # Load R3 with the byte @R2
    0x90430000,  // lbu $3, 0($2)
    //
    // # Store the value into the pseudo-data register
    0xAC230008,  // sw $3, 8($1)
    //
    0x00000000,  // nop
    0x1000FFF9,  // beq $0, $0, start
    0x00000000,
    0x00000000
}; // nop

unsigned int pracc_writebyte_code_module[] =
{
    // #
    // # HairyDairyMaid's Assembler PrAcc Write Byte Routine
    // #
    // start:
    //
    // # Load R1 with the address of the pseudo-address register
    0x3C01FF20,  // lui $1,  0xFF20
    0x34210000,  // ori $1,  0x0000
    //
    // # Load R2 with the address for the write
    0x8C220000,  // lw $2,  ($1)
    //
    // # Load R3 with the data from pseudo-data register
    0x8C230004,  // lw $3, 4($1)
    //
    // # Store the byte at @R2 (the address)
    0xA0430000,  // sb $3,  ($2)
    //
    0x00000000,  // nop
    0x1000FFF9,  // beq $0, $0, start
    0x00000000,
    0x00000000
}; // nop


unsigned int pracc_readword_code_module[] =
{
    // #
    // # HairyDairyMaid's Assembler PrAcc Read Word Routine
    // #
    // start:
    //
    // # Load R1 with the address of the pseudo-address register
    0x3C01FF20,  // lui $1,  0xFF20
    0x34210000,  // ori $1,  0x0000
    //
    // # Load R2 with the address for the read
    0x8C220000,  // lw $2,  ($1)
    //
    // # Load R3 with the word @R2
    0x8C430000,  // lw $3, 0($2)
    //
    // # Store the value into the pseudo-data register
    0xAC230004,  // sw $3, 4($1)
    //
    0x00000000,  // nop
    0x1000FFF9,  // beq $0, $0, start
    0x00000000,
    0x00000000
}; // nop


unsigned int pracc_writeword_code_module[] =
{
    // #
    // # HairyDairyMaid's Assembler PrAcc Write Word Routine
    // #
    // start:
    //
    // # Load R1 with the address of the pseudo-address register
    0x3C01FF20,  // lui $1,  0xFF20
    0x34210000,  // ori $1,  0x0000
    //
    // # Load R2 with the address for the write
    0x8C220000,  // lw $2,  ($1)
    //
    // # Load R3 with the data from pseudo-data register
    0x8C230004,  // lw $3, 4($1)
    //
    // # Store the word at @R2 (the address)
    0xAC430000,  // sw $3,  ($2)
    //
    0x00000000,  // nop
    0x1000FFF9,  // beq $0, $0, start
    0x00000000,
    0x00000000
}; // nop


unsigned int pracc_readhalf_code_module[] =
{
    // #
    // # HairyDairyMaid's Assembler PrAcc Read HalfWord Routine
    // #
    // start:
    //
    // # Load R1 with the address of the pseudo-address register
    0x3C01FF20,  // lui $1,  0xFF20
    0x34210000,  // ori $1,  0x0000
    //
    // # Load R2 with the address for the read
    0x8C220000,  // lw $2,  ($1)
    //
    // # Load R3 with the half word @R2
    0x94430000,  // lhu $3, 0($2)
    //
    // # Store the value into the pseudo-data register
    0xAC230004,  // sw $3, 4($1)
    //
    0x00000000,  // nop
    0x1000FFF9,  // beq $0, $0, start
    0x00000000,
    0x00000000
}; // nop


unsigned int pracc_writehalf_code_module[] =
{
    // #
    // # HairyDairyMaid's Assembler PrAcc Write HalfWord Routine
    // #
    // start:
    //
    // # Load R1 with the address of the pseudo-address register
    0x3C01FF20,  // lui $1,  0xFF20
    0x34210000,  // ori $1,  0x0000
    //
    // # Load R2 with the address for the write
    0x8C220000,  // lw $2,  ($1)
    //
    // # Load R3 with the data from pseudo-data register
    0x8C230004,  // lw $3, 4($1)
    //
    // # Store the half word at @R2 (the address)
    0xA4430000,  // sh $3,  ($2)
    //
    0x00000000,  // nop
    0x1000FFF9,  // beq $0, $0, start
    0x00000000,
    0x00000000
}; // nop

unsigned int pracc_return_from_debug[] =
{
    // start:
    //
    // # Load R1 with the address of the pseudo-address register
//  0x3C010000,  // lui   $1,   0x8000
//  0x34210000,  // ori   $1,   0x0000   # $1 <-  ($1 | 0x0000)
    0x00000000,  // nop
    0x00000000,  // nop
    //
    // # Load R1 to R24(DEPC) of CP0
//  0x4101C000,  // mtc0 $24,   0, 0     # DEPC <- $1
    0x00000000,  // nop
    0x00000000,  // nop

    0x4200001F,  // deret
    0x00000000,  // nop
    0x1000FFF9,  // beq $0, $0, start
    0x00000000
};


unsigned int pracc_read_depc[] =
{
    // #
    // # HairyDairyMaid's Assembler PrAcc Read Word Routine
    // #
    // start:
    //
    // # Load R1 with the address of the pseudo-address register
    0x3C01FF20,  // lui $1,  0xFF20
    0x34210000,  // ori $1,  0x0000
    //
    // # Load R2 with the DEPC
    0x4002C000,  // mfc0 $2, $24(DEPC)   (fetch R2 <- DEFC to R2)
    //
    // # nop
    0x00000000,  //
    //
    // # Store the R2 value into the pseudo-data register
    0xAC220004,  // sw $2, 4($1)
    //
    0x00000000,  // nop
    0x1000FFF9,  // beq $0, $0, start
    0x00000000
};

unsigned int pracc_write_depc[] =
{
    // # Load R1 with the address of the pseudo-address register
    0x3C01FF20,  // lui $1,  0xFF20
    0x34210000,  // ori $1,  0x0000
    //
    // # Load R2 with the data from pseudo-data register
    0x8C220004,  // lw $3, 4($1)
    //0x3C02F800,  // lui $2,  0x8000
    //0x34420000,  // ori $2,  0x0000
    //
    // # store DEPC from R2
    0x4082C000,  // mtc0 $24(DEPC),   $2, 0     # DEPC <- $2
    0x00000000,  // nop
    0x00000000,  // nop (put some delay for DEPC update to take place)
    //
    0x1000FFF9,  // beq $0, $0, start
    0x00000000
};

//   **************** hugebird new code ************************

unsigned int pracc_init_dreg[] =
{
    // #
    // # hugebird: PrAcc init data register
    // #
    // start:
    // # keep $1 for all R/W operation
    0x3C01FF20,  // lui  $1,0xFF20  # li $1, MIPS_VIRTUAL_DATA_BASE
    0x00000000,
    0x1000FFFD,  // b  start
    0x00000000   // nop
};


unsigned int pracc_readword_new[] =
{
    // #
    // # hugebird: PrAcc Read Word Routine (4 sets)
    // #
    // start:
    //
    // # Load R1 with the address of the pseudo-address register
// 0x3C01FF20,// lui $1,  0xFF20
// 0x34210000,// ori $1,  0x0000
    //
    // # Load R2 with the address %hi
    0x3C021F01,  // lui $2,  addr_hi   # p+0
    //
    // # Load R3 with the word @R2
    0x8C438100,  // lw $3, addr_lo($2) #p+1        <-   100011 00010 00011
    //
    // # Store the value into the pseudo-data register
    0xAC230004,  // sw $3, 4($1)       #p+2
    //
    0x00000000,  // nop   ,delay must have, wait data arrive.
    0x1000FFFB,  // b  start
    0x00000000
}; // nop

unsigned int pracc_readhalf_new[] =
{
    // #
    // # hugebird: PrAcc Read Word Routine(4 sets)
    // #
    // start:
    //
    // # Load R1 with the address of the pseudo-address register
// 0x3C01FF20,// lui $1,  0xFF20
// 0x34210000,// ori $1,  0x0000
    //
    // # Load R2 with the address %hi
    0x3C021F01,  // lui $2,  addr_hi    # p+0
    //
    // # Load R3 with the word @R2
    0x94438100,  // lhu $3, addr_lo($2) # p+1         <- 100101 00010 00011
    //
    // # Store the value into the pseudo-data register
    0xAC230004,  // sw $3, 4($1)        # p+2
    //
    0x00000000,  // nop   ,delay must have, wait wait data arrive
    0x1000FFFB,  // b  start
    0x00000000
}; // nop

unsigned int pracc_writehalf_new[] =
{
    // #
    // # hugebird : PrAcc Write Word Routine (5 sets)
    // #
    // start:
    //
    // # Load R1 with the address of the pseudo-address register
    //0x3C01FF20,  // lui $1,  0xFF20
    //0x34210000,  // ori $1,  0x0000
    //
    // # Load R2 with the address %hi
    0x3C021F01,  // lui $2,  addr_hi    # p+0
    //
    // # Load R3 with the data
    0x34030010,  // li $3, data         # p+1
    //
    // # Store the half word at @R2 (the address)
    0xA4438100,  // sh $3,  addr_lo($2) # p+2    <- 101001 00010 00011
    //
    0x1000FFFC,  // beq $0, $0, start
    0x00000000
}; // nop

unsigned int pracc_writeword_new[] =
{
    // #
    // # hugebird : PrAcc Write Word Routine (5 sets)
    // #
    // start:
    //
    // # Load R1 with the address of the pseudo-address register
    //0x3C01FF20,  // lui $1,  0xFF20
    //0x34210000,  // ori $1,  0x0000
    //
    // # Load R2 with the address %hi
    0x3C021F01,  // lui $2,  addr_hi        # p+0
    //
    // # Load R3 with the data
    0x3C030001,  // lui $3, data_hi         # p+1
    0x34631000,	 // ori	$3,$3,data_lo       # p+2      <-001101 00011 00011
    //
    // # Store the half word at @R2 (the address)
    0xAC438100,  // sw $3, addr_lo($2)      # p+3
    //
    0x1000FFFB,  // beq $0, $0, start
    0x00000000
}; // nop

// **************************************************************************
//     hugeird : add cpu init configuration code below.
//               don't forget add point to CPU structure.
// **************************************************************************
/*
unsigned int init_6358[] = {
               // #
               // # hugebird: initialize 6358 to fully access 16MB flash
               // #
               // start:
               //
  0x00000000,  // nop
  0x0000e021,  // move	gp,zero
  0x3c09ff40,  // lui	$9, 0xff40
  0x3529000c,  // ori	$9, $9, 0x0c
  0x4089b006,  // mtc0	$9, $22, 6
               //
  0x00000000,  // nop
  0x1000FFF9,  // beq $0, $0, start
  0x00000000   // nop
  };
*/
unsigned int init_5352[] =
{
    // #
    // # hugebird: initialize 6358 to fully access 16MB flash
    // #
    // start:
    //
    0x00000000,  // nop
    0x0000e021,  // move	gp,zero
    0x3c09ff40,  // lui	$9, 0xff40
    0x3529000c,  // ori	$9, $9, 0x0c
    0x40826000,  // mtc0	$9, $22, 6
    //
    0x00000000,  // nop
    0x1000FFF9,  // beq $0, $0, start
    0x00000000   // nop
};

unsigned int cpu_ejtag[] =
{
    //
    0x081c0818, 	// j	80702060 <cpu_reset>
    0x4200001f, 	// deret
    0x00000000      // nop
};
// **************************************************************************
// End of File
// **************************************************************************
