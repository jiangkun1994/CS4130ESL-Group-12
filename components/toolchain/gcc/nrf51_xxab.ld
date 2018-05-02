<<<<<<< HEAD
/* Linker script to configure memory regions. */

=======
/* Linker script to configure memory regions. */

>>>>>>> 80fe7f94de4e32ab8d8fe62623cdab8dd6d93acd
SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x20000
  RAM (rwx) :  ORIGIN = 0x20000000, LENGTH = 0x4000
<<<<<<< HEAD
}


=======
}


>>>>>>> 80fe7f94de4e32ab8d8fe62623cdab8dd6d93acd
INCLUDE "nrf51_common.ld"
