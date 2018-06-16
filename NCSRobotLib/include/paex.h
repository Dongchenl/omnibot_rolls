#ifndef PAEX_H
#define PAEX_H

#include <sys/types.h>

// registers
#define CONTROL 0
#define VERSION 31

// IOCTL commands
#define AER_READ 1000
#define AER_WRITE 1001
#define AER_READ_REG 1002
#define AER_SET_REG 1003
#define AER_SET_BIAS 1004
#define AER_WRITE_INBUFFER 1005

typedef struct
{
	uint32_t address;
	uint32_t value;
} aer_mem_t;

#endif
