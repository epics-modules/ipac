/*******************************************************************************

Project:
    CAN Bus Driver for EPICS

File:
    drvVipc610.c

Description:
    IPAC Carrier Driver for the SBS/GreenSpring VIPC610 and VIPC610-01 Quad
    IndustryPack Carrier VME boards, it provides the interface between IPAC
    driver and the hardware.  This carrier is 6U high, but cannot support
    32-bit accesses to dual-slot IP modules, or Extended mode addresses.
    The VIPC610-01 fixes the IRQ levels to be equivalent to two VIPC310
    carriers, which is different to (and more useful than) the VIPC610.

Author:
    Andrew Johnson <anjohnson@iee.org>
Created:
    19 July 1995

Copyright (c) 1995-2003 Andrew Johnson

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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <epicsTypes.h>
#include <errMdef.h>
#include <devLib.h>
#include <iocsh.h>
#include <epicsExport.h>

#include "drvIpac.h"


/* Characteristics of the card */

#define SLOTS 4
#define IO_SPACES 2	/* Address spaces in A16 */
#define IPAC_IRQS 2	/* Interrupts per module */
#define EXTENT 0x400	/* Register size in A16 */


/* Offsets from base address in VME A16 */

#define REGS_A 0x0000
#define PROM_A 0x0080
#define REGS_B 0x0100
#define PROM_B 0x0180
#define REGS_C 0x0200
#define PROM_C 0x0280
#define REGS_D 0x0300
#define PROM_D 0x0380


/* VME Interrupt levels */

#define IRQ_A0 1
#define IRQ_A1 2
#define IRQ_B0 3
#define IRQ_B1 4
#define IRQ_C0 5
#define IRQ_C1 6
#define IRQ_D0 7
#define IRQ_D1 0

/* VME Interrupt levels for -01 option */

#define IRQ_A0_01 4
#define IRQ_A1_01 5
#define IRQ_B0_01 2
#define IRQ_B1_01 1
#define IRQ_C0_01 4
#define IRQ_C1_01 5
#define IRQ_D0_01 2
#define IRQ_D1_01 1


/* Carrier Private structure type, one instance per board */

typedef void* private_t[IPAC_ADDR_SPACES][SLOTS];


/*******************************************************************************

Routine:
    initialise

Purpose:
    Creates new private table for VIPC610 at addresses given by cardParams

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
    The memory base address of the VIPC610 card is set using the same jumpers 
    as the I/O base address and is always 256 times the I/O base address, 
    but in the VME A24 address space.  The factory default for the memory 
    base address is thus 0x600000.  If the memory size parameter is omitted 
    or set to zero then none of the IP modules on the carrier provide any 
    memory space.  Legal memory size values are 0, 64?, 128, 256, 512, 1024 
    or 2048.  The memory size interacts with the memory base address such
    that it is possible to exclude memory from the lower slots while still
    providing access to memory in the later slots by adjusting the base
    address suitably.

Examples:
    "0x6000" 
	This indicates that the carrier board has its I/O base set to 
	0x6000, and none of the slots provide memory space.
    "1000,128"
	Here the I/O base is set to 0x1000, and there is 128Kbytes of 
	memory on each module, with the IP module A memory at 0x100000,
	module B at 0x120000, module C at 0x140000 and D at 0x160000.
    "7000,1024"
	The I/O base is at 0x7000, and hence the carrier memory base is 
	0x700000.  However because the memory size is set to 1024 Kbytes, 
	modules A, B and C cannot be selected (1024 K = 0x100000, so they 
	are decoded at 0x400000, 0x500000 and 0x600000 but can't be accessed
	because these are below the base address).

Returns:
    0 = OK, 
    S_IPAC_badAddress = Parameter string error, or address not reachable

*/

LOCAL int initialise (
    const char *cardParams,
    void **pprivate,
    epicsUInt16 carrier
) {
    int params, mSize = 0;
    epicsUInt32 ioBase, mOrig, mBase, mEnd, addr;
    volatile void *ptr;
    char *ioPtr, *mPtr;
    int space, slot;
    private_t *private;
    static const int offset[IO_SPACES][SLOTS] = {
	{ PROM_A, PROM_B, PROM_C, PROM_D },
	{ REGS_A, REGS_B, REGS_C, REGS_D }
    };

    if (cardParams == NULL ||
	strlen(cardParams) == 0) {
	/* No params or empty string, use manufacturers default settings */
	ioBase = 0x6000;
    } else {
	params = sscanf(cardParams, "%i,%i", &ioBase, &mSize);
	if (params < 1 || params > 2 ||
	    ioBase > 0xfc00 || ioBase & 0x01ff ||
	    mSize < 0 || mSize > 2048 || mSize & 63) {
	    return S_IPAC_badAddress;
	}
    }

    mBase = ioBase << 8;	/* Fixed by the VIPC610 card */
    ioBase = ioBase & 0xfc00;	/* Clear A09 */

    if (devRegisterAddress("VIPC610", atVMEA16, ioBase, EXTENT, &ptr)) {
	return S_IPAC_badAddress;
    }
    ioPtr = (char *) ptr;       /* ioPtr points to ioBase in A16 space */

    mSize = mSize << 10;	/* Convert size from K to Bytes */
    mEnd = (mBase & ~(mSize * SLOTS - 1)) + mSize * SLOTS;

    if (mSize &&
	devRegisterAddress("VIPC610", atVMEA24, mBase, mEnd - mBase, &ptr)) {
	return S_IPAC_badAddress;
    }
    mPtr = (char *) ptr;        /* mPtr points to mBase in A24 space */
    mOrig = mBase & ~(mSize * SLOTS - 1);

    private = malloc(sizeof (private_t));
    if (!private)
	return S_IPAC_noMemory;

    for (space = 0; space < IO_SPACES; space++) {
	for (slot = 0; slot < SLOTS; slot++) {
	    (*private)[space][slot] = (void *) (ioPtr + offset[space][slot]);
	}
    }

    for (slot = 0; slot < SLOTS; slot++) {
	(*private)[ipac_addrIO32][slot] = NULL;
	addr = mOrig + (mSize * slot);
	if ((mSize == 0) || (addr < mBase)) {
	    (*private)[ipac_addrMem][slot] = NULL;
	} else {
	    (*private)[ipac_addrMem][slot] = (void *) (mPtr + (addr - mBase));
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
    epicsUInt16 slot,
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
    The carrier board is limited to fixed interrupt levels, and has 
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
    epicsUInt16 slot,
    epicsUInt16 irqNumber,
    ipac_irqCmd_t cmd,
    const int irqLevel[SLOTS][IPAC_IRQS]
) {
    switch (cmd) {
	case ipac_irqGetLevel:
	    return irqLevel[slot][irqNumber];

	case ipac_irqEnable:
	    devEnableInterruptLevel(intVME, irqLevel[slot][irqNumber]);
	    return OK;

	default:
	    return S_IPAC_notImplemented;
    }
}

LOCAL int irqCmd_610 (
    void *private,
    epicsUInt16 slot,
    epicsUInt16 irqNumber,
    ipac_irqCmd_t cmd
) {
    static const int irqLevel[SLOTS][IPAC_IRQS] = {
	{ IRQ_A0, IRQ_A1 }, 
	{ IRQ_B0, IRQ_B1 },
	{ IRQ_C0, IRQ_C1 },
	{ IRQ_D0, IRQ_D1 }
    };
    return irqCmd(private, slot, irqNumber, cmd, irqLevel);
}

LOCAL int irqCmd_610_01 (
    void *private,
    epicsUInt16 slot,
    epicsUInt16 irqNumber,
    ipac_irqCmd_t cmd
) {
    static const int irqLevel[SLOTS][IPAC_IRQS] = {
	{ IRQ_A0_01, IRQ_A1_01 }, 
	{ IRQ_B0_01, IRQ_B1_01 },
	{ IRQ_C0_01, IRQ_C1_01 },
	{ IRQ_D0_01, IRQ_D1_01 }
    };
    return irqCmd(private, slot, irqNumber, cmd, irqLevel);
}


/******************************************************************************/


/* IPAC Carrier Tables */

static ipac_carrier_t vipc610 = {
    "SBS/GreenSpring VIPC610",
    SLOTS,
    initialise,
    NULL,
    baseAddr,
    irqCmd_610,
    NULL,
    NULL
};

int ipacAddVIPC610(const char *cardParams) {
    return ipacAddCarrier(&vipc610, cardParams);
}

static ipac_carrier_t vipc610_01 = {
    "SBS/GreenSpring VIPC610-01",
    SLOTS,
    initialise,
    NULL,
    baseAddr,
    irqCmd_610_01,
    NULL,
    NULL
};

int ipacAddVIPC610_01(const char *cardParams) {
    return ipacAddCarrier(&vipc610_01, cardParams);
}


/* iocsh command table and registrar */

static const iocshArg vipcArg0 =
    {"cardParams", iocshArgString};
static const iocshArg * const vipcArgs[1] =
    {&vipcArg0};

static const iocshFuncDef vipc610FuncDef =
    {"ipacAddVIPC610", 1, vipcArgs};
static const iocshFuncDef vipc610_01FuncDef =
    {"ipacAddVIPC610_01", 1, vipcArgs};

static void vipc610CallFunc(const iocshArgBuf *args) {
    ipacAddVIPC610(args[0].sval);
}

static void vipc610_01CallFunc(const iocshArgBuf *args) {
    ipacAddVIPC610_01(args[0].sval);
}

static void epicsShareAPI vipc610Registrar(void) {
    iocshRegister(&vipc610FuncDef, vipc610CallFunc);
    iocshRegister(&vipc610_01FuncDef, vipc610_01CallFunc);
}

epicsExportRegistrar(vipc610Registrar);
