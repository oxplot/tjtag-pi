// for SPI FLASH

#define AR5315_DSLBASE          0xB1000000     /* RESET CONTROL MMR */
/* GPIO */
#define AR5315_GPIO_DI          (AR5315_DSLBASE + 0x0088)
#define AR5315_GPIO_DO          (AR5315_DSLBASE + 0x0090)
#define AR5315_GPIO_CR          (AR5315_DSLBASE + 0x0098)
#define AR5315_GPIO_INT         (AR5315_DSLBASE + 0x00a0)


#define AR531XPLUS_SPI_READ     0x1fc00000
#define AR531XPLUS_SPI_MMR      0x11300000
#define AR531XPLUS_SPI_MMR_SIZE 12
#define AR531XPLUS_SPI_CTL      0x00
#define AR531XPLUS_SPI_OPCODE   0x04
#define AR531XPLUS_SPI_DATA     0x08

#define BRCM_SPI_READ           0x1fc00000
#define BRCM_SPI_MMR            0x00000000
#define BRCM_SPI_MMR_SIZE       0
#define BRCM_SPI_CTL            0x18000040
#define BRCM_FLASHADDRESS       0x18000044
#define BRCM_SPI_DATA           0x18000048
#define BRCM_SPI_OPCODE         0x18000044

#define STM_PAGE_SIZE           256

#define STM_8MBIT_SIGNATURE     0x13
#define STM_M25P80_BYTE_COUNT   1048576
#define STM_M25P80_SECTOR_COUNT 16
#define STM_M25P80_SECTOR_SIZE  0x10000

#define STM_16MBIT_SIGNATURE    0x14
#define STM_M25P16_BYTE_COUNT   2097152
#define STM_M25P16_SECTOR_COUNT 32
#define STM_M25P16_SECTOR_SIZE  0x10000

#define STM_32MBIT_SIGNATURE    0x15
#define STM_M25P32_BYTE_COUNT   4194304
#define STM_M25P32_SECTOR_COUNT 64
#define STM_M25P32_SECTOR_SIZE  0x10000

#define STM_64MBIT_SIGNATURE    0x16 	// guess work blah!!

#define STM_1MB_BYTE_COUNT   STM_M25P80_BYTE_COUNT
#define STM_1MB_SECTOR_COUNT STM_M25P80_SECTOR_COUNT
#define STM_1MB_SECTOR_SIZE  STM_M25P80_SECTOR_SIZE
#define STM_2MB_BYTE_COUNT   STM_M25P16_BYTE_COUNT
#define STM_2MB_SECTOR_COUNT STM_M25P16_SECTOR_COUNT
#define STM_2MB_SECTOR_SIZE  STM_M25P16_SECTOR_SIZE
#define STM_4MB_BYTE_COUNT   STM_M25P32_BYTE_COUNT
#define STM_4MB_SECTOR_COUNT STM_M25P32_SECTOR_COUNT
#define STM_4MB_SECTOR_SIZE  STM_M25P32_SECTOR_SIZE

#define AR_SPI_CTL_START           0x00000100
#define AR_SPI_CTL_BUSY            0x00010000

//#define SPI_CTL_START           0x00000100
//#define SPI_CTL_BUSY            0x00010000

#define BRCM_SPI_CTL_START      0x80000000
#define BRCM_SPI_CTL_BUSY       0x80000000

#define SPI_CTL_TXCNT_MASK      0x0000000f
#define SPI_CTL_RXCNT_MASK      0x000000f0
#define SPI_CTL_TX_RX_CNT_MASK  0x000000ff
#define SPI_CTL_SIZE_MASK       0x00060000
#define SPI_CTL_CLK_SEL_MASK    0x03000000
#define SPI_OPCODE_MASK         0x000000ff

/*
 * ST Microelectronics Opcodes for Serial Flash
 */

#define STM_OP_WR_ENABLE       0x06    /* Write Enable */
#define STM_OP_WR_DISABLE      0x04     /* Write Disable */
#define STM_OP_RD_STATUS       0x05     /* Read Status */
#define STM_OP_WR_STATUS       0x01     /* Write Status */
#define STM_OP_RD_DATA         0x03     /* Read Data */
#define STM_OP_FAST_RD_DATA    0x0b     /* Fast Read Data */
#define STM_OP_PAGE_PGRM       0x02     /* Page Program */
#define STM_OP_SECTOR_ERASE    0xd8     /* Sector Erase */
#define STM_OP_BULK_ERASE      0xc7     /* Bulk Erase */
#define STM_OP_DEEP_PWRDOWN    0xb9     /* Deep Power-Down Mode */
#define STM_OP_RD_SIG          0xab     /* Read Electronic Signature */
#define STM_OP_RD_ID           0x9f     /* Read Identification */

#define BCM_STM_OP_WR_ENABLE    0x0006   /* Write Enable */
#define BCM_STM_OP_RD_STATUS    0x0105   /* Read Status */
#define BCM_STM_OP_PAGE_PGRM    0x0402   /* Page Program */
#define BCM_STM_OP_SECTOR_ERASE 0x02d8   /* Sector Erase */
#define BCM_STM_OP_RD_ID        0x049f

#define SPI_WRITE_ENABLE    0
#define SPI_WRITE_DISABLE   1
#define SPI_RD_STATUS       2
#define SPI_WR_STATUS       3
#define SPI_RD_DATA         4
#define SPI_FAST_RD_DATA    5
#define SPI_PAGE_PROGRAM    6
#define SPI_SECTOR_ERASE    7
#define SPI_BULK_ERASE      8
#define SPI_DEEP_PWRDOWN    9
#define SPI_RD_SIG          10
#define SPI_RD_ID           11

#define BCM_SPI_WRITE_ENABLE    12
#define BCM_SPI_RD_STATUS       13
#define BCM_SPI_PAGE_PROGRAM    14
#define BCM_SPI_SECTOR_ERASE    15
#define BCM_SPI_RD_ID           16
#define SPI_MAX_OPCODES     17

#define STM_STATUS_WIP       0x01       /* Write-In-Progress */
#define STM_STATUS_WEL       0x02       /* Write Enable Latch */
#define STM_STATUS_BP0       0x04       /* Block Protect 0 */
#define STM_STATUS_BP1       0x08       /* Block Protect 1 */
#define STM_STATUS_BP2       0x10       /* Block Protect 2 */
#define STM_STATUS_SRWD      0x80       /* Status Register Write Disable */

#define SPI_STATUS_WIP          STM_STATUS_WIP

#define FLASH_1MB  1
#define FLASH_2MB  2
#define FLASH_4MB  3
#define MAX_FLASH  4

#define SF_PAGESIZE 528 /* bytes */


#define byteSwap_32(value)         \
    (((((uint32_t)value)<<24) & 0xFF000000) | \
     ((((uint32_t)value)<< 8) & 0x00FF0000) | \
     ((((uint32_t)value)>> 8) & 0x0000FF00) | \
     ((((uint32_t)value)>>24) & 0x000000FF))

struct opcodes
{
    uint16_t code;
    uint8_t tx_cnt;
    uint8_t rx_cnt;
}
stm_opcodes[] =
{
    {STM_OP_WR_ENABLE,              1, 0},
    {STM_OP_WR_DISABLE,             1, 0},
    {STM_OP_RD_STATUS,              1, 1},
    {STM_OP_WR_STATUS,              1, 0},
    {STM_OP_RD_DATA,                4, 4},
    {STM_OP_FAST_RD_DATA,           1, 0},
    {STM_OP_PAGE_PGRM,              8, 0},
    {STM_OP_SECTOR_ERASE,           4, 0},
    {STM_OP_BULK_ERASE,             1, 0},
    {STM_OP_DEEP_PWRDOWN,           1, 0},
    {STM_OP_RD_SIG,                 4, 1},
    {STM_OP_RD_ID,                  1, 3},

    {BCM_STM_OP_WR_ENABLE,          1, 0},
    {BCM_STM_OP_RD_STATUS,          1, 1},
    {BCM_STM_OP_PAGE_PGRM,          8, 0},
    {BCM_STM_OP_SECTOR_ERASE,       4, 0},
    {BCM_STM_OP_RD_ID,              1, 3}

};

uint32_t spiflash_sendcmd(int op);

/* Atmel Opcodes AT45DB161 */

#define SF_PAGE_READ            0x52
#define SF_BUFFER1_READ         0x54
#define SF_BUFFER2_READ         0x56
#define SF_PAGE_CACHE1          0x53
#define SF_PAGE_CACHE2          0x55
#define SF_PAGE_COMPARE1        0x60
#define SF_PAGE_COMPARE2        0x61
#define SF_BUFFER1_WRITE        0x84
#define SF_BUFFER2_WRITE        0x87
#define SF_BUFFER1_FLUSH_PERASE 0x83
#define SF_BUFFER2_FLUSH_PERASE 0x86
#define SF_BUFFER1_FLUSH        0x88
#define SF_BUFFER2_FLUSH        0x89
#define SF_PAGE_ERASE           0x81
#define SF_BLOCK_ERASE          0x50
#define SF_PAGE_PROGRAM1        0x82
#define SF_PAGE_PROGRAM2        0x85
#define SF_PAGE_REWRITE1        0x58
#define SF_PAGE_REWRITE2        0x59
#define SF_AT_READ_STATUS       0x57

/* End of Atmel Opcodes and Defines */


/* flashcontrol action+opcodes for Atmel flashes */
#define SFLASH_AT_READ				0x07e8
#define SFLASH_AT_PAGE_READ			0x07d2
#define SFLASH_AT_BUF1_READ
#define SFLASH_AT_BUF2_READ
#define SFLASH_AT_STATUS			0x01d7
#define SFLASH_AT_BUF1_WRITE			0x0384
#define SFLASH_AT_BUF2_WRITE			0x0387
#define SFLASH_AT_BUF1_ERASE_PROGRAM		0x0283
#define SFLASH_AT_BUF2_ERASE_PROGRAM		0x0286
#define SFLASH_AT_BUF1_PROGRAM			0x0288
#define SFLASH_AT_BUF2_PROGRAM			0x0289
#define SFLASH_AT_PAGE_ERASE			0x0281
#define SFLASH_AT_BLOCK_ERASE			0x0250
#define SFLASH_AT_BUF1_WRITE_ERASE_PROGRAM	0x0382
#define SFLASH_AT_BUF2_WRITE_ERASE_PROGRAM	0x0385
#define SFLASH_AT_BUF1_LOAD			0x0253
#define SFLASH_AT_BUF2_LOAD			0x0255
#define SFLASH_AT_BUF1_COMPARE			0x0260
#define SFLASH_AT_BUF2_COMPARE			0x0261
#define SFLASH_AT_BUF1_REPROGRAM		0x0258
#define SFLASH_AT_BUF2_REPROGRAM		0x0259

/* Status register bits for Atmel flashes */

/* Status bits (MSB order)
 */




#define SF_PAGESIZE 528 /* bytes */
/*
#ifdef AT45DB161B
#define DF_DENSITY_MBIT                 16
#define DF_PAGES                                4096
#define DF_BYTES_PER_PAGE               528
#define DF_PAGE_ADDRESS_BITS    10
#define DF_NUMBER_OF_BUFFERS    2
#endif
*/
/*void dfBlockErase(u16 blockAddr)
{
        // assert chip select
        cbi(DF_CS_PORT,DF_CS_PIN);

        // issue the command
        spiTransferByte(DF_CMD_BLOCK_ERASE);
        spiTransferByte((u08)(blockAddr >> (16 - DF_PAGE_ADDRESS_BITS)));
        spiTransferByte((u08)(blockAddr << (DF_PAGE_ADDRESS_BITS - 5)));
        spiTransferByte(0x00);

        // release chip select
        sbi(DF_CS_PORT,DF_CS_PIN);

        // wait until transfer finished
        while(!(dfStatusRegRead() & (1<<DF_RDY)));
}
*/
