/*******************************************************************************

Project:
    CAN Bus Driver for EPICS

File:
    drvIpMv162.c

Description:
    IPAC Carrier Driver for the IndustryPack carriers on the Motorola 
    MVME162 CPU board, provides the interface between IPAC driver and the 
    hardware.

Author:
    Andrew Johnson <anjohnson@iee.org>
Created:
    6 July 1995

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
#include <vme.h>
#include <sysLib.h>
#include <vxLib.h>
#include <taskLib.h>

#include "epicsTypes.h"
#include "errMdef.h"
#include "iocsh.h"
#include "epicsExport.h"

#include "drvIpac.h"
#include "ipic.h"


/* Characteristics of the card */

#define SLOTS 4
#define IO_SPACES 2	/* Address spaces in A16 */
#define IPAC_IRQS 2	/* Interrupts per module */
#define IPIC_BASE 0xfffbc000


/* Base Addresses of IO and ID spaces */

#define REGS_A 0xfff58000
#define PROM_A 0xfff58080
#define REGS_B 0xfff58100
#define PROM_B 0xfff58180
#define REGS_C 0xfff58200
#define PROM_C 0xfff58280
#define REGS_D 0xfff58300
#define PROM_D 0xfff58380
#define REGS_AB 0xfff58400
#define REGS_CD 0xfff58500


/* IPIC chip */

LOCAL ipic_t *ipic = (ipic_t *) IPIC_BASE;


/* IP Recovery Timers */

LOCAL const uchar_t recoveryTime[] = {
    IPIC_GEN_RT_0, 
    IPIC_GEN_RT_2, 
    IPIC_GEN_RT_2, 
    IPIC_GEN_RT_4, 
    IPIC_GEN_RT_4, 
    IPIC_GEN_RT_8, 
    IPIC_GEN_RT_8, 
    IPIC_GEN_RT_8, 
    IPIC_GEN_RT_8 
};


/* Carrier Base Address structure, only one instance can exist! */

LOCAL long mBase[IPAC_ADDR_SPACES][SLOTS] = {
    { PROM_A,  PROM_B, PROM_C,  PROM_D },
    { REGS_A,  REGS_B, REGS_C,  REGS_D },
    { REGS_AB, 0   ,   REGS_CD, 0      },
    { 0   ,    0   ,   0   ,    0      }
};


/*******************************************************************************

Routine:
    initialise

Purpose:
    Initialises the MVME162 IPIC chip with settings given in cardParams

Description:
    

Parameters:
    

Examples:
    "A:m=0x80000000,1024 l=4;B:l=2,2;C:m=0x80100000,64"

Returns:
    0 = OK, 
    S_IPAC_tooMany = Carrier already registered
    S_IPAC_badDriver = IPIC chip not found
    S_IPAC_badAddress = Parameter string error, or address not reachable

*/

LOCAL int initialise (
    const char *cardParams,
    void **pprivate,
    ushort_t carrier
) {
    static int initialised = FALSE;
    ushort_t slot;
    int count, next;
    long p1, p2;
    char dummy, cmd;

    if (initialised) {
	return S_IPAC_tooMany;
    }

    if (vxMemProbe((void *)&ipic->chipId, READ, 1, &dummy) ||
	ipic->chipId != IPIC_CHIP_ID) {
	return S_IPAC_badDriver;
    }

    if (*cardParams == 'R') {
	/* Module reset requested */
	ipic->ipReset = IPIC_IP_RESET;
	taskDelay(sysClkRateGet() / 20 + 2);
	ipic->ipReset = 0;
	taskDelay(sysClkRateGet() / 10 + 2);
	cardParams++;
    }
    
    /* Initialise the IPIC chip */
    for (slot = 0; slot < SLOTS; slot++) {
	ipic->intCtrl[slot][0] = IPIC_INT_ICLR;
	ipic->intCtrl[slot][1] = IPIC_INT_ICLR;
	ipic->genCtrl[slot] = IPIC_GEN_WIDTH_16 | IPIC_GEN_RT_0;
    }

    /* Parse the parameter string */
    slot = 0;
    while ((cmd = *cardParams++) != '\0') {
	switch (cmd) {
	    case 'A':
	    case 'B':
	    case 'C':
	    case 'D':
		slot = cmd - 'A';
		break;

	    case 'm':
		p1 = p2 = 0;
		count = sscanf(cardParams, "= %p, %ld %n",
				(void **) &p1, &p2, &next);
		if (count != 2 ||
		    (char *) p1 < sysMemTop() ||
		    (p1 & 0xffff) != 0 ||
		    p2 < 64 || p2 > 16384 ||
		    (unsigned) (p1 + (p2*1024)) > 0xfff00000u) {
		    return S_IPAC_badAddress;
		}

		ipic->memBase[slot] = p1 >> 16;
		ipic->memSize[slot] = (p2 / 64) - 1;
		ipic->genCtrl[slot] |= IPIC_GEN_MEN;
		mBase[ipac_addrMem][slot] = p1;
		cardParams += next;
		break;

	    case 'l':
		p1 = p2 = 0;
		count = sscanf(cardParams, "= %ld %n , %ld %n", &p1, &next, &p2, &next);
		if (count < 1 || count > 2 ||
		    p1 < 0 || p1 > 7 ||
		    p2 < 0 || p2 > 7) {
		    return S_IPAC_badAddress;
		}

		ipic->intCtrl[slot][0] = (p1 & IPIC_INT_LEVEL) |
		    (ipic->intCtrl[slot][0] & ~IPIC_INT_LEVEL);
		ipic->intCtrl[slot][1] = (p2 & IPIC_INT_LEVEL) |
		    (ipic->intCtrl[slot][1] & ~IPIC_INT_LEVEL);
		cardParams += next;
		break;

	    case 'r':
		p1 = 0;
		count = sscanf(cardParams, "= %ld %n", &p1, &next);
		if (count != 1 ||
		    p1 < 0 || p1 > 8) {
		    return S_IPAC_badAddress;
		}

		ipic->genCtrl[slot] = (ipic->genCtrl[slot] & ~IPIC_GEN_RT) | 
				      recoveryTime[p1];
		cardParams += next;
		break;
	    
	    case 'w':
		p1 = 0;
		count = sscanf(cardParams, "= %ld %n", &p1, &next);
		if (count != 1) {
		    return S_IPAC_badAddress;
		}

		switch (p1) {
		    case 8:
			ipic->genCtrl[slot] = IPIC_GEN_WIDTH_8 |
				(ipic->genCtrl[slot] & ~IPIC_GEN_WIDTH);
			break;
		    case 16:
			ipic->genCtrl[slot] = IPIC_GEN_WIDTH_16 |
				(ipic->genCtrl[slot] & ~IPIC_GEN_WIDTH);
			break;
		    case 32:
			if (slot & 1) {
			    /* Illegal for odd-numbered slots */
			    return S_IPAC_badAddress;
			}
			ipic->genCtrl[slot] = IPIC_GEN_WIDTH_32 | 
				(ipic->genCtrl[slot] & ~IPIC_GEN_WIDTH);
			ipic->genCtrl[slot+1] &= ~(IPIC_GEN_WIDTH | 
						   IPIC_GEN_MEN);
			break;
		    default:
			return S_IPAC_badAddress;
		}
	}
    }

    initialised = TRUE;
    return OK;
}


/*******************************************************************************

Routine:
    baseAddr

Purpose:
    Returns the base address for the requested slot & address space

Description:
    This routine only has to do a table lookup in the mBase array.
    Note that no parameter checking is required - the IPAC driver which 
    calls this routine handles that.

Returns:
    The requested address, or NULL if the slot has no memory in the
    requested address space.

*/

LOCAL void *baseAddr (
    void *private,
    ushort_t slot,
    ipac_addr_t space
) {
    return (void *) mBase[space][slot];
}


/*******************************************************************************

Routine:
    irqCmd

Purpose:
    Handles interrupter commands and status requests

Description:
    The IPIC chip allows a lot of control over the IP interrupters, thus
    all commands perform the requested action.

Returns:
    ipac_irqGetLevel returns the current interrupt level,
    ipac_irqPoll returns >0 if interrupt line active else 0,
    other calls return 0 = OK.

*/

LOCAL int irqCmd (
    void *private,
    ushort_t slot,
    ushort_t irqNumber,
    ipac_irqCmd_t cmd
) {
    switch (cmd) {
	case ipac_irqLevel0:
	case ipac_irqLevel1:
	case ipac_irqLevel2:
	case ipac_irqLevel3:
	case ipac_irqLevel4:
	case ipac_irqLevel5:
	case ipac_irqLevel6:
	case ipac_irqLevel7:
	    ipic->intCtrl[slot][irqNumber] = (cmd & IPIC_INT_LEVEL) | 
		(ipic->intCtrl[slot][irqNumber] & ~IPIC_INT_LEVEL);
	    return OK;
	    
	case ipac_irqGetLevel:
	    return ipic->intCtrl[slot][irqNumber] & IPIC_INT_LEVEL;

	case ipac_irqEnable:
	    ipic->intCtrl[slot][irqNumber] |= IPIC_INT_ICLR;
	    ipic->intCtrl[slot][irqNumber] |= IPIC_INT_IEN;
	    return OK;

	case ipac_irqDisable:
	    ipic->intCtrl[slot][irqNumber] &= ~IPIC_INT_IEN;
	    return OK;

	case ipac_irqPoll:
	    return ipic->intCtrl[slot][irqNumber] & IPIC_INT_INT;

	case ipac_irqSetEdge:
	    ipic->intCtrl[slot][irqNumber] |= IPIC_INT_EDGE;
	    return OK;

	case ipac_irqSetLevel:
	    ipic->intCtrl[slot][irqNumber] &= ~IPIC_INT_EDGE;
	    return OK;

	case ipac_irqClear:
	    ipic->intCtrl[slot][irqNumber] |= IPIC_INT_ICLR;
	    return OK;

	default:
	    return S_IPAC_notImplemented;
    }
}

/******************************************************************************/


/* IPAC Carrier Table */

static ipac_carrier_t ipmv162 = {
    "Motorola MVME162/172",
    SLOTS,
    initialise,
    NULL,
    baseAddr,
    irqCmd,
    NULL,
    NULL
};

int ipacAddMVME162(const char *cardParams) {
    return ipacAddCarrier(&ipmv162, cardParams);
}


/* iocsh command table and registrar */

static const iocshArg mvipArg0 = { "cardParams",iocshArgString};
static const iocshArg * const mvipArgs[1] = {&mvipArg0};
static const iocshFuncDef mvipFuncDef = {"ipacAddMVME162", 1, mvipArgs};
static void mvipCallFunc(const iocshArgBuf *args) {
    ipacAddMVME162(args[0].sval);
}

static void epicsShareAPI mv162ipRegistrar(void) {
    iocshRegister(&mvipFuncDef, mvipCallFunc);
}

epicsExportRegistrar(mv162ipRegistrar);
