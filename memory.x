/* STM32F429ZI memory layout (2MB Flash, 192KB SRAM) */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 2048K
  RAM   : ORIGIN = 0x20000000, LENGTH = 192K
}

/* This is consumed by cortex-m-rt */
PROVIDE(_stack_start = ORIGIN(RAM) + LENGTH(RAM));
