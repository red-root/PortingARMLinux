OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(_start)
SECTIONS
{
 . = 0x00000000;
 . = ALIGN(4);
 .text :
 {
   board/freescale/mx6q_zeus/flash_header.o (.text.flasheader)
   cpu/arm_cortexa8/start.o
   board/freescale/mx6q_zeus/libmx6q_zeus.a (.text)
   lib_arm/libarm.a (.text)
   net/libnet.a (.text)
   drivers/mtd/libmtd.a (.text)
   drivers/mmc/libmmc.a (.text)
   . = DEFINED(env_offset) ? env_offset : .;
   common/env_embedded.o(.text)
   *(.text)
 }
 . = ALIGN(4);
 .rodata : { *(SORT_BY_ALIGNMENT(SORT_BY_NAME(.rodata*))) }
 . = ALIGN(4);
 .data : { *(.data) }
 . = ALIGN(4);
 .got : { *(.got) }
 . = .;
 __u_boot_cmd_start = .;
 .u_boot_cmd : { *(.u_boot_cmd) }
 __u_boot_cmd_end = .;
 . = ALIGN(4);
 _end_of_copy = .;
 . = ALIGN(0x1000);
 __hab_data = .;
 . = . + 0x2000;
 __data_enc_key = .;
 . = . + 0x200;
 __hab_data_end = .;
 __bss_start = .;
 .bss : { *(.bss) }
 _end = .;
}
