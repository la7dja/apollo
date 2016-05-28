#include <avr/io.h>


FUSES = {
	.low = LFUSE_DEFAULT, 
	.high = HFUSE_DEFAULT | ~FUSE_JTAGEN, // disable JTAG
	.extended = EFUSE_DEFAULT
};
