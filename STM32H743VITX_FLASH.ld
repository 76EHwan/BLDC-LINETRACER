/*
******************************************************************************
** File        : LinkerScript.ld
** Author      : STM32CubeIDE
** Abstract    : Linker script for STM32H7 series
*****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(RAM_D1) + LENGTH(RAM_D1);
/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size = 0x200;
_Min_Stack_Size = 0x400;

/* Specify the memory areas */
MEMORY
{
  FLASH (rx)           : ORIGIN = 0x08000000, LENGTH = 2048K
  DTCMRAM (xrw)        : ORIGIN = 0x20000000, LENGTH = 128K
  RAM_D1 (xrw)         : ORIGIN = 0x24000000, LENGTH = 512K
  RAM_D2 (xrw)         : ORIGIN = 0x30000000, LENGTH = 256K    /* SRAM1 + SRAM2 */
  RAM_D2_NOCACHE (xrw) : ORIGIN = 0x30040000, LENGTH = 32K     /* SRAM3 (Non-Cacheable) */
  RAM_D3 (xrw)         : ORIGIN = 0x38000000, LENGTH = 64K     /* SRAM4 (Non-Cacheable) */
  ITCMRAM (xrw)        : ORIGIN = 0x00000000, LENGTH = 64K
}

/* Define output sections */
SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
  } >FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    KEEP (*(.init))
    KEEP (*(.fini))
    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab (READONLY) : 
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } >FLASH

  .ARM (READONLY) : 
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array (READONLY) : 
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH

  .init_array (READONLY) : 
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH

  .fini_array (READONLY) : 
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH

  _sidata = LOADADDR(.data);
  
  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    *(.RamFunc)
    *(.RamFunc*)
    . = ALIGN(4);
    _edata = .;
  } >RAM_D1 AT> FLASH

  . = ALIGN(4);
  .bss :
  {
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM_D1

  /* Non-Cacheable SDMMC/FatFs Buffers */
  .ram_d2_nocache (NOLOAD) :
  {
    . = ALIGN(32);
    *(.ram_d2_nocache)
    . = ALIGN(32);
  } >RAM_D2_NOCACHE

  /* Non-Cacheable BDMA Buffers (ADC3) */
  .ram_d3 (NOLOAD) :
  {
    . = ALIGN(32);
    *(.ram_d3)
    . = ALIGN(32);
  } >RAM_D3

  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM_D1

  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}