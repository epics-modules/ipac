/*******************************************************************************

Project:
    Gemini/UKIRT CAN Bus Driver for EPICS

File:
    drvVipc616_01.c

Description:
    IPAC Carrier Driver for the GreenSpring VIPC616-01 Quad IndustryPack 
    Carrier VME board, provides the interface between IPAC driver and the 
    hardware.  This carrier is 6U high and can support VME Extended mode 
    addresses, but not 32-bit access to dual-slot IP modules.  Note the
    VIPC616-01 fixes the IRQ levels to be equivalent to two VIPC310
    carriers, different to the VIPC616.

Author:
    Andrew Johnson
Created:
    8 March 2001
Version:
    $Id: drvVipc616_01.c,v 1.1 2001-03-08 20:14:36 anj Exp $

*******************************************************************************/

#include <vxWorks.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <vme.h>
#include <sysLib.h>
#include "drvIpac.h"


/* Characteristics of the card */

#define SLOTS 4
#define IO_SPACES 2	/* Address spaces in A16 */
#define IPAC_IRQS 2	/* Interrupts per module */


/* Offsets from base address in VME A16 */

#define REGS_A 0x0000
#define PROM_A 0x0080
#define REGS_B 0x0100
#define PROM_B 0x0180
#define REGS_C 0x0200
#define PROM_C 0x0280
#define REGS_D 0x0300
#define PROM_D 0x0380


/* VME Interrupt levels for -01 option */

#define IRQ_A0 4
#define IRQ_A1 5
#define IRQ_B0 2
#define IRQ_B1 1
#define IRQ_C0 4
#define IRQ_C1 5
#define IRQ_D0 2
#define IRQ_D1 1


/* Carrier Private structure type, one instance per board */

typedef void* private_t[IPAC_ADDR_SPACES][SLOTS];


/*******************************************************************************

Routine:
    initialise

Purpose:
    Creates new private table for VIPC616 at addresses given by cardParams

Description:
    Checks the parameter string for the address of the card I/O space and 
    optional address and size of the card memory space.  A private table 
    is created for this card which is a 2-D array of pointers to the base 
    addresses of the various accessible parts of each IP module.

Parameters:

    The parameter string should comprise a hex number (the 0x or 0X at
    the start is optional), optionally followed by a comma and another
    hex number, and possibly then another comma and a decimal integer.  
    The first number is the I/O base address of the card in the VME A16
    address space (the factory default is 0x6000).  If the second
    number is present without the third number, this is the base
    address of the module memory space in the VME A32 address space
    (the address can then only have non-zero bits in A25 to A31; each 
    IP module is allocated 8Mb for memory in the VME A32 space).  If a
    third number is present, the second number is the card memory
    address within the VME A24 address space and the third number is
    the size in Kbytes allocated to each IP module.  Legal memory size
    values are 0, 64?, 128, 256, 512, 1024 or 2048.  This memory size
    interacts with the memory base address such that it is possible to
    exclude memory from the lower slots while still providing access to
    memory in the later slots by adjusting the base address suitably.

    The factory default for the memory base address of the VIPC616 is 
    0xD0000000 (A32 space).  If the memory address parameters are omitted 
    then none of the IP modules on the carrier provide any memory space.  

Examples:
    "0x6000" 
	This indicates that the carrier board has its I/O base set to 
	0x6000, and none of the slots provide memory space.
    "1000,80000000"
	Here the I/O base is set to 0x1000, and the module memory areas
	are accessible in A32 space starting at 0x80000000, so IP module 
	A memory starts at 0x80000000, module B at 0x80800000, module C 
	at 0x81000000 and D at 0x81800000.
    "6000,700000,1024"
	The I/O base is at 0x6000, and the carrier memory base is in A24 
	space at 0x700000.  However because the memory size is set to 
	1024 Kbytes, modules A, B and C cannot be selected (1024 K = 
	0x100000, so they are decoded at 0x400000, 0x500000 and 0x600000 
	but can't be accessed because these are below the memory base 
	address).

Returns:
    0 = OK, 
    S_IPAC_badAddress = Parameter string error, or address not reachable

*/

LOCAL int initialise (
    char *cardParams,
    void **pprivate,
    ushort_t carrier
) {
    int params, ioStatus, memStatus = OK, mSize = 0;
    ulong_t ioBase, mOrig, mBase, addr;
    ushort_t space, slot;
    private_t *private;
    static const int offset[IO_SPACES][SLOTS] = {
	{
	    PROM_A, PROM_B, PROM_C, PROM_D
	}, {
	    REGS_A, REGS_B, REGS_C, REGS_D
	}
    };

    if (cardParams == NULL ||
	strlen(cardParams) == 0) {
	/* No params or empty string, use manufacturers default settings */
	ioBase = 0x6000;
	mBase = 0xd0000000;
	params = 2;	/* Pretend, mBase is in A32 space */
    } else {
	params = sscanf(cardParams, "%p,%p,%i", 
			(void **) &ioBase, (void **) &mBase, &mSize);
	if (params < 1 || params > 3 ||
	    ioBase > 0xfc00 || ioBase & 0x03ff ||
	    (params == 2 && mBase & 0x01ffffff) ||
	    (params == 3 && mBase & 0xff01ffff) ||
	    mSize < 0 || mSize > 2048 || mSize & 63) {
	    return S_IPAC_badAddress;
	}
    }

    ioStatus = sysBusToLocalAdrs(VME_AM_SUP_SHORT_IO, 
				(char *) ioBase, (char **) &ioBase);
    if (params == 1) {
    	/* No memory, just the A16 I/O space */
	mSize = 0;
	mOrig = 0;
    } else if (params == 2) {
	/* A32 space, 8Mb allocated per module */
	memStatus = sysBusToLocalAdrs(VME_AM_EXT_SUP_DATA, 
				    (char *) mBase, (char **) &mBase);
	mSize = 8 << 20;
	mOrig = mBase;
    } else {
	/* A24 space, variable size per module */
	memStatus = sysBusToLocalAdrs(VME_AM_STD_SUP_DATA, 
				    (char *) mBase, (char **) &mBase);
    	mSize = mSize << 10;	    /* Convert size from K to Bytes */
    	mOrig = mBase & ~(mSize * SLOTS - 1);

    }
    if (ioStatus || memStatus) {
	return S_IPAC_badAddress;
    }

    private = malloc(sizeof (private_t));
    for (space = 0; space < IO_SPACES; space++) {
	for (slot = 0; slot < SLOTS; slot++) {
	    (*private)[space][slot] = (void *) (ioBase + offset[space][slot]);
	}
    }

    for (slot = 0; slot < SLOTS; slot++) {
	(*private)[ipac_addrIO32][slot] = NULL;
	addr = mOrig + (mSize * slot);
	if ((mSize == 0) || (addr < mBase)) {
	    (*private)[ipac_addrMem][slot] = NULL;
	} else {
	    (*private)[ipac_addrMem][slot] = (void *) addr;
	}
    }

    *pprivate = private;

    return OK;
}


/*******************************************************************************

Routine:
    baseAddr

Purpose:
    Returns the base address for the requested slot & address space

Description:
    Because we did all that hard work in the initialise routine, this 
    routine only has to do a table lookup in the private array.
    Note that no parameter checking is required - the IPAC driver which 
    calls this routine handles that.

Returns:
    The requested address, or NULL if the module has no memory.

*/

LOCAL void *baseAddr (
    void *private,
    ushort_t slot,
    ipac_addr_t space
) {
    return (*(private_t *) private)[space][slot];
}


/*******************************************************************************

Routine:
    irqCmd

Purpose:
    Handles interrupter commands and status requests

Description:
    The GreenSpring board is limited to fixed interrupt levels, and has 
    no control over interrupts.  The only commands thus supported are
    a request of the interrupt level associated with a particular slot 
    and interrupt number, or to enable interrupts by making sure the
    VMEbus interrupter is listening on the necessary level.

Returns:
    ipac_irqGetLevel returns the interrupt level,
    ipac_irqEnable returns 0 = OK,
    other calls return S_IPAC_notImplemented.

*/

LOCAL int irqCmd (
    void *private,
    ushort_t slot,
    ushort_t irqNumber,
    ipac_irqCmd_t cmd
) {
    static const int irqLevel[SLOTS][IPAC_IRQS] = {
	{
	    IRQ_A0, IRQ_A1
	}, {
	    IRQ_B0, IRQ_B1
	}, {
	    IRQ_C0, IRQ_C1
	}, {
	    IRQ_D0, IRQ_D1
	}
    };

    switch (cmd) {
	case ipac_irqGetLevel:
	    return irqLevel[slot][irqNumber];

	case ipac_irqEnable:
	    sysIntEnable(irqLevel[slot][irqNumber]);
	    return OK;

	default:
	    return S_IPAC_notImplemented;
    }
}

/******************************************************************************/


/* IPAC Carrier Table */

ipac_carrier_t vipc616_01 = {
    "GreenSpring VIPC616-01",
    SLOTS,
    initialise,
    NULL,
    baseAddr,
    irqCmd,
    NULL
};

