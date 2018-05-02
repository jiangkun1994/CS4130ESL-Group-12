<<<<<<< HEAD
/* Linker script to configure memory regions. */

=======
/* Linker script to configure memory regions. */

>>>>>>> 80fe7f94de4e32ab8d8fe62623cdab8dd6d93acd
SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x40000
  RAM (rwx) :  ORIGIN = 0x20000000, LENGTH = 0x8000
<<<<<<< HEAD
}


INCLUDE "nrf51_common.ld"
=======
}


INCLUDE "nrf51_common.ld"
>>>>>>> 80fe7f94de4e32ab8d8fe62623cdab8dd6d93acd
