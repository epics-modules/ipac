/*******************************************************************************

Project:
    CAN Bus Driver for EPICS

File:
    drvVipc310.c

Description:
    IPAC Carrier Driver for the GreenSpring VIPC310 Dual IndustryPack 
    Carrier VME board, provides the interface between IPAC driver and the 
    hardware.  This carrier is 3U high, and thus cannot support 32-bit 
    accesses to dual-slot IP modules.

Author:
    Andrew Johnson <anjohnson@iee.org>
Created:
    5 July 1995
Version:
    $Id: drvVipc310.c,v 1.4 2000-02-21 21:35:33 anj Exp $

Copyright (c) 1995-2000 Andrew Johnson

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

*******************************************************************************/

#include <vxWorks.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <vme.h>
#include <sysLib.h>
#include "drvIpac.h"


/* Characteristics of the card */

#define SLOTS 2
#define IO_SPACES 2	/* Address spaces in A16 */
#define IPAC_IRQS 2	/* Interrupts per module */


/* Offsets from base address in VME A16 */

#define REGS_A 0x0000
#define PROM_A 0x0080
#define REGS_B 0x0100
#define PROM_B 0x0180


/* VME Interrupt levels */

#define IRQ_A0 4
#define IRQ_A1 5
#define IRQ_B0 2
#define IRQ_B1 1


/* Carrier Private structure type, one instance per board */

typedef void* private_t[IPAC_ADDR_SPACES][SLOTS];


/*******************************************************************************

Routine:
    initialise

Purpose:
    Creates new private table for VIPC310 at addresses given by cardParams

Description:
    Checks the parameter string for the address of the card I/O space and 
    optional size of the memory space for the modules.  If both the I/O and
    memory base addresses can be reached from the CPU, a private table is 
    created for this board.  The private table is a 2-D array of pointers 
    to the base addresses of the various accessible parts of the IP module.

Parameters:
    The parameter string should comprise a hex number (the 0x or 0X at the 
    start is optional) optionally followed by a comma and a decimal integer.  
    The first number is the I/O base address of the card in the VME A16 
    address space (the factory default is 0x6000).  If present the second 
    number gives the memory space in Kbytes allocated to each IP module.
    The memory base address of the VIPC310 card is set using the same jumpers
    as the I/O base address and is always 256 times the I/O base address,
    but in the VME A24 address space.  The factory default fot the memory
    base address is thus 0x600000.  If the memory size parameter is omitted
    or set to zero then neither IP module provides any memory space.  Legal
    memory size values are 0, 64, 128, 256, 512, 1024 or 2048.  The memory 
    size interacts with the memory base address such that it is possible to
    set the existance of memory in either slot independently with suitable 
    adjustment of the base address.

Examples:
    "0x6000" 
	This indicates that the carrier has its I/O base set to 0x6000, and
	neither slot uses any memory space.
    "1000,512"
	Here the I/O base is set to 0x1000, and there is 512 Kbytes of 
	memory on each module, with the IP module A memory at 0x100000 
	and module B at 0x180000.
    "0xfe00, 128"
	The I/O base is at 0xfe00, and hence the carrier memory base is 
	0xfe0000. However because the memory size is set to give each module
	128 Kbytes of memory space, module A cannot be selected (128 K = 
	0x020000, so the module is decoded at 0xfc0000 but can't be accessed 
	because this is below the memory base).

Returns:
    0 = OK, 
    S_IPAC_badAddress = Parameter string error, or address not reachable

*/

LOCAL int initialise (
    char *cardParams,
    void **pprivate,
    ushort_t carrier
) {
    int params, status1 = OK, status2 = OK, mSize = 0;
    long ioBase, mOrig, mBase;
    ushort_t space, slot;
    private_t *private;
    static const int offset[IO_SPACES][SLOTS] = {
	{ PROM_A, PROM_B },
	{ REGS_A, REGS_B }
    };

    if (cardParams == NULL ||
	strlen(cardParams) == 0) {
	ioBase = 0x6000;
    } else {
	params = sscanf(cardParams, "%p,%i", (void **) &ioBase, &mSize);
	if (params < 1 || params > 2 ||
	    ioBase > 0xfe00 || ioBase & 0x01ff ||
	    mSize < 0 || mSize > 2048 || mSize & 63) {
	    return S_IPAC_badAddress;
	}
    }

    mBase = ioBase << 8;	/* Fixed by VIPC310 card */

    status1 = sysBusToLocalAdrs(VME_AM_SUP_SHORT_IO, 
				(char *) ioBase, (char **) &ioBase);
    if (mSize > 0) {
	status2 = sysBusToLocalAdrs(VME_AM_STD_SUP_DATA, 
			  (char *) mBase, (char **) &mBase);
    }
    if (status1 || status2) {
	return S_IPAC_badAddress;
    }

    mSize = mSize << 10;	/* Convert size from K to Bytes */
    mOrig = mBase & ~(mSize * SLOTS - 1);

    private = malloc(sizeof (private_t));
    for (space = 0; space < IO_SPACES; space++) {
	for (slot = 0; slot < SLOTS; slot++) {
	    (*private)[space][slot] = (void *) (ioBase + offset[space][slot]);
	}
    }

    (*private)[ipac_addrIO32][0] = NULL;
    (*private)[ipac_addrIO32][1] = NULL;

    if (mOrig == mBase) {
	(*private)[ipac_addrMem][0] = (void *) mBase;
	(*private)[ipac_addrMem][1] = (void *) (mBase + mSize);
    } else {
	(*private)[ipac_addrMem][0] = NULL;
	(*private)[ipac_addrMem][1] = (void *) mBase;
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
    ipac_irqGetLevel returns the interrupt level (1, 2, 4 or 5),
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
	{ IRQ_A0, IRQ_A1 } , 
	{ IRQ_B0, IRQ_B1 }
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

ipac_carrier_t vipc310 = {
    "GreenSpring VIPC310",
    SLOTS,
    initialise,
    NULL,
    baseAddr,
    irqCmd,
    NULL
};

