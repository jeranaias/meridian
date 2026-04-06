/* STM32H743 Memory Layout for Meridian
 *
 * Bootloader occupies first 128KB of flash (0x08000000-0x0801FFFF).
 * Meridian firmware starts at 0x08020000.
 * Parameters stored in last 2 sectors (pages 14-15, 0x081C0000-0x081FFFFF).
 *
 * CRITICAL: DMA cannot access DTCM or ITCM.
 * All DMA buffers must be in AXISRAM, SRAM1, SRAM2, SRAM3, or SRAM4.
 * SRAM4 is mapped uncached for bidirectional DShot.
 */

MEMORY
{
    /* Flash: 2MB total, bootloader uses first 128KB */
    FLASH    (rx)  : ORIGIN = 0x08020000, LENGTH = 1792K  /* 2048K - 128K bootloader - 128K params */

    /* Parameter storage: last 2 sectors */
    FLASH_PARAMS (r) : ORIGIN = 0x081C0000, LENGTH = 256K

    /* ITCM RAM: 64KB, fastest for code execution (if XIP from RAM needed) */
    ITCM     (rwx) : ORIGIN = 0x00000000, LENGTH = 64K

    /* DTCM RAM: 128KB, fastest for stack and local variables */
    /* WARNING: NOT accessible by DMA! */
    DTCM     (rw)  : ORIGIN = 0x20000000, LENGTH = 128K

    /* AXI SRAM: 512KB, DMA-safe, main working memory */
    AXISRAM  (rw)  : ORIGIN = 0x24000000, LENGTH = 512K

    /* SRAM1: 128KB, DMA-safe */
    SRAM1    (rw)  : ORIGIN = 0x30000000, LENGTH = 128K

    /* SRAM2: 128KB, DMA-safe */
    SRAM2    (rw)  : ORIGIN = 0x30020000, LENGTH = 128K

    /* SRAM3: 32KB, DMA-safe */
    SRAM3    (rw)  : ORIGIN = 0x30040000, LENGTH = 32K

    /* SRAM4: 64KB, uncached (for bidirectional DShot + ethernet) */
    SRAM4    (rw)  : ORIGIN = 0x38000000, LENGTH = 64K

    /* Backup SRAM: 4KB, battery-backed */
    BKPSRAM  (rw)  : ORIGIN = 0x38800000, LENGTH = 4K

    /* Default RAM for stack + .data + .bss = DTCM (fastest, but not DMA) */
    RAM      (rw)  : ORIGIN = 0x20000000, LENGTH = 128K
}

/* Stack in DTCM (fastest, no DMA needed for stack) */
_stack_start = ORIGIN(DTCM) + LENGTH(DTCM);

/* Sections for DMA-safe memory regions */
SECTIONS
{
    /* DMA buffers for SPI (IMU reads) — must be in AXI SRAM */
    .axisram (NOLOAD) : ALIGN(4)
    {
        *(.axisram .axisram.*)
    } > AXISRAM

    /* DMA buffers for UART RX/TX bounce buffers */
    .sram1 (NOLOAD) : ALIGN(4)
    {
        *(.sram1 .sram1.*)
    } > SRAM1

    /* Additional DMA buffers */
    .sram2 (NOLOAD) : ALIGN(4)
    {
        *(.sram2 .sram2.*)
    } > SRAM2

    /* Uncached region for bidirectional DShot */
    .sram4 (NOLOAD) : ALIGN(4)
    {
        *(.sram4 .sram4.*)
    } > SRAM4

    /* Parameter storage in flash */
    .param_flash (NOLOAD) : ALIGN(4)
    {
        *(.param_flash .param_flash.*)
    } > FLASH_PARAMS

    /* Battery-backed SRAM for persistent data across resets */
    .bkpsram (NOLOAD) : ALIGN(4)
    {
        *(.bkpsram .bkpsram.*)
    } > BKPSRAM
}
