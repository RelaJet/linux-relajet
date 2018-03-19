#ifndef CPUCOMM_API_H
#define CPUCOMM_API_H

struct cpu_comm_transfer_data {
	unsigned long  command;
	unsigned long  phy_addr;
	unsigned long  size;
	unsigned long  seq;	
	unsigned long  result;
#define CPUCOMM_FLAG_RESULT_OK 		(1 << 0)	/* Receiver need response */	
	unsigned long  flag;
#define CPUCOMM_FLAG_WAIT_FOR_RESP	(1 << 0)	/* Receiver need response */
#define CPUCOMM_FLAG_ACK				(1 << 1)	/* Receiver need response */
#define CPUCOMM_FLAG_CMDSND			(1 << 2)	/* Receiver need response */

	unsigned long  reserved;	
};

#endif
