#ifndef _LOADER_ARGS_H
#define _LOADER_ARGS_H

#include <stdint.h>

/* USB mass storage class - command block */
struct usb_msc_cbw {
	uint32_t sign;		/* Signature: 0x43555342 "USBC" */
	uint32_t tag;		/* Request tag */
	uint32_t xfer_len;	/* Transfer length */
	uint8_t flags;		/* Flags: bit7 = direction (0:in, 1:out) */
	uint8_t lun;		/* Logical unit number */
	uint8_t cb_len;		/* Length of the command block */
	uint8_t cb[16];		/* Command block */
};

typedef int (*msd_send_func)(const void *ptr, int len);
typedef int (*msd_recv_func)(void *ptr, int len);
typedef int (*msd_hook_func)(struct usb_msc_cbw *cbw, void *pbuff);

typedef void (*loader_call_v1)(msd_send_func, msd_recv_func, msd_hook_func *);

/*==================================================================================================*/

struct jl_loader_args {
	/* Send data to host */
	msd_send_func msd_send;
	/* Receive data from host */
	msd_recv_func msd_recv;
	/* MSC/SCSI command hook */
	msd_hook_func *msd_hook;
	/* Passed argument (really 16-bit) */
	uint32_t arg;
};

typedef void (*loader_call_v2)(struct jl_loader_args *);

#endif
