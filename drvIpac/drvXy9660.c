/*******************************************************************************

Project:
    IndustryPack Driver Interface for EPICS

File:
    drvXy9660.c

Description:

    IPAC Carrier Driver for the Acromag AVME-9660, 9668 and 9670 Quad IP carrier
    VME boards, which were once sold by Xycom as the XVME-9660 and XVME-9670.
    This file provides the interface between the IPAC driver and the hardware.

    These carriers are 6U high and support VME A16+A24 addressing modes only.
    The difference between the 9660 and 9670 is in the physical wiring used to
    connect external I/O signals to the IP modules; the 9670 requires a VME64x
    backplane with a P0 connector, and has no front panel wiring.

    The AVME-9668 is similar to the 9660 board, but can also operate individual
    IP modules at 32MHz.

Author:
    Andrew Johnson <anjohnson@iee.org>
Created:
    20th August 2007

Copyright (c) 2007 UChicago Argonne LLC.

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

/* ANSI headers */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

/* EPICS headers */
#define epicsAssertAuthor "Andrew Johnson <anj@aps.anl.gov>"
#include <epicsAssert.h>
#include <epicsTypes.h>
#include <errMdef.h>
#include <dbDefs.h>
#include <devLib.h>
#include <epicsThread.h>
#include <epicsExit.h>
#include <iocsh.h>
#include <epicsEndian.h>
#include <epicsExport.h>

/* Module headers */
#include "drvIpac.h"


/* Characteristics of the card */

#define SLOTS 4         /* Number of IP slots */
#define IO_SPACES 2     /* Address spaces in A16 */
#define EXTENT 0x400    /* Register size in A16 */


/* Offsets from base address in VME A16 */

#define REGS_A 0x0000
#define PROM_A 0x0080
#define CTLREG 0x00C0
#define REGS_B 0x0100
#define PROM_B 0x0180
#define REGS_C 0x0200
#define PROM_C 0x0280
#define REGS_D 0x0300
#define PROM_D 0x0380


/* Carrier Board Registers */

typedef struct {
    epicsInt16 ctlStatus;
    epicsInt16 irqLevel;
    epicsInt16 ipError;
    epicsInt16 memEnable;
    epicsInt16 clkControl;
    epicsInt16 carrierId;
    epicsInt16 pad_cc[2];
    epicsInt16 memCtl[SLOTS];
    epicsInt16 pad_d8[4];
    epicsInt16 irqEnable;
    epicsInt16 irqStatus;
    epicsInt16 irqClear;
    epicsInt16 pad_e6[13];
} ctrl_t;

/* Bits in the ctlStatus register */

#define CSR_GIP     0x04    /* Global Interrupt Pending (r) */
#define CSR_GIE     0x08    /* Global Interrupt Enable (rw) */
#define CSR_RESET   0x10    /* Soft Reset (rw) */
#define CSR_TOA     0x20    /* Time Out Access (r) */
#define CSR_AAD     0x40    /* Auto Acknowledge Disable (rw) */
#define CSR_ACE     0x80    /* Auto Clear Interrupt Enable (rw) */

/* The 9668 has a carrierId register with this identifier */
#define ID_32MHz    0x0b    /* 32MHz clocks supported */


/* Carrier Private structure, one instance per board */

typedef struct {
    volatile ctrl_t *regs;
    void *addr[IPAC_ADDR_SPACES][SLOTS];
} private_t;


static void avme96XXreboot(void *r) {
    volatile ctrl_t *regs = (ctrl_t *)r;

    regs->ctlStatus &= ~CSR_GIE;
}

/*******************************************************************************

Routine:
    initialise

Purpose:
    Registers a new carrier with settings given in cardParams.

Description:
    Parses the parameter string for the card settings, initializes the card,
    sets its interrupt level and any slot memory mappings and creates a new
    private table with the necessary address information.

Parameters:
    The parameter string starts with a hex number (a leading 0x is optional)
    which sets the I/O base address of the card in the VME A16 address space
    (the factory default is 0x0000). Next must come a comma, followed by the VME
    interrupt level (0 through 7, although 0 means all interrupts are disabled)
    to be used for this carrier (all module interrupts share the same interrupt
    level from this board).

    An 'R' may appear at this point, which causes a carrier soft reset to be
    generated before further configuration occurs.

    Finally if any module drivers need to access the memory space on their
    module, the relevent slots must have their memory size and base address
    configured, which is done like this:
        <slot letter A-D> = <mem size 1/2/4/8>, <A24 base address>

    The memory size is a single digit, expressed in MegaBytes.  The A24 base
    address is a 6-digit hexadecimal number (a leading 0x is optional) which
    must be compatible with the selected memory size.  Slots can be configured
    in any order, but cannot be programmed to overlap.

Examples:
    "0x6000,4 R"
        This indicates that the carrier board has its I/O base set to A16:6000
        and that it generates interrupts on VME IRQ4.  None of the slots are
        configured for memory, and modules are reset at initialization.
    "C000,3 A=2,800000 C=1,A00000"
        The carrier is at A16:C000 and generates level 3 interrupts. Slot A is
        configured for 2MB of memory space at A24:800000 and Slot C for 1MB of
        memory space at A24:A00000.

Returns:
    0 = OK,
    S_IPAC_badAddress = Parameter string error
    S_IPAC_noMemory = malloc() failed
    S_dev_ errors returned by devLib

*/

static int initialise (
    const char *cardParams,
    void **pprivate,
    epicsUInt16 carrier
) {
    epicsUInt32 baseAddr;
    char *basePtr;
    int irqLevel;
    int skip;
    long status;
    volatile void *ptr;
    volatile ctrl_t *regs;
    epicsUInt16 value;
    int space, slot;
    private_t *private;
    static const int offset[IO_SPACES][SLOTS] = {
        { PROM_A, PROM_B, PROM_C, PROM_D },
        { REGS_A, REGS_B, REGS_C, REGS_D }
    };
    char memSlot[2], memSize[2];
    epicsUInt32 memBase;
    static const int memBits[10] = {
        -1, 0, 1, -1, 2, -1, -1, -1, 3, -1
    };
    int memCtl;
    static const epicsUInt32 memMask[4] = {
        0x0fffff, 0x1fffff, 0x3fffff, 0x7fffff
    };

    if (cardParams == NULL ||
        strlen(cardParams) == 0) {
        return S_IPAC_badAddress;
    }

    if (2 != sscanf(cardParams, "%x, %i %n", &baseAddr, &irqLevel, &skip)) {
        printf("AVME-IP: Error parsing card configuration '%s'\n", cardParams);
        return S_IPAC_badAddress;
    }
    cardParams += skip;

    status = devRegisterAddress("AVME-IP", atVMEA16, baseAddr, EXTENT, &ptr);
    if (status) {
        printf("AVME-IP: Can't map VME address A16:%4.4x\n", baseAddr);
        return status;
    }
    basePtr = (char *) ptr;
    regs = (volatile ctrl_t *) (basePtr + CTLREG);

    if (irqLevel < 0 || irqLevel > 7) {
        printf("AVME-IP: Bad IRQ level '%d'\n", irqLevel);
        return S_IPAC_badAddress;
    }

    /* Safely disable everything on the carrier */
    value = 0;
    /* Only byte-wide writes work; address the low byte */
    #define B0 (EPICS_BYTE_ORDER == EPICS_ENDIAN_BIG)
    if ((status = devWriteProbe(1, B0 + (char *) &regs->ctlStatus, &value)) ||
        (status = devWriteProbe(1, B0 + (char *) &regs->memEnable, &value)) ||
        (status = devWriteProbe(1, B0 + (char *) &regs->irqEnable, &value))) {
        printf("AVME-IP: Write to %p (VME A16:%4.4x) failed, is it there?\n",
            ptr, baseAddr);
        return status;
    }

    regs->irqClear  = 0xff;
    if ((regs->carrierId & 0xff) == ID_32MHz)
        regs->clkControl = 0;

    private = malloc(sizeof (private_t));
    if (!private)
        return S_IPAC_noMemory;

    private->regs = regs;
    for (space = 0; space < IO_SPACES; space++) {
        for (slot = 0; slot < SLOTS; slot++) {
            private->addr[space][slot] =
                (void *) (basePtr + offset[space][slot]);
        }
    }
    for (slot = 0; slot < SLOTS; slot++) {
        private->addr[ipac_addrIO32][slot] = NULL;
        private->addr[ipac_addrMem ][slot] = NULL;
    }

    /* Check for a Reset in the parameter string */
    while (isspace((int) *cardParams))
        ++cardParams;
    if (toupper((int) *cardParams) == 'R') {
        ++cardParams;
        regs->ctlStatus = CSR_RESET;
        while (regs->ctlStatus & CSR_RESET)
            epicsThreadSleep(0.01);
    }

    /* Now configure the card */
    epicsAtExit(avme96XXreboot, (void *) regs);
    regs->irqLevel = irqLevel;
    regs->ctlStatus = CSR_ACE | /* Auto-clear interrupts */
                      CSR_AAD | /* Disable auto-DTACK */
                      CSR_GIE;  /* Enable interrupts */
    devEnableInterruptLevel(intVME, irqLevel);

    /* On the 9668, use 32MHz clocks where modules support them */
    if ((regs->carrierId & 0xff) == ID_32MHz) {
        epicsUInt16 clkControl = 0;

        for (slot = 0; slot < SLOTS; slot++) {
            epicsUInt16 word;
            ipac_idProm_t *id = (ipac_idProm_t *)
                private->addr[ipac_addrID][slot];

            if (!devReadProbe(sizeof(word), (void *)&id->asciiI, (char *)&word)
                && ipcCheckId(id) == OK) {
                if ((id->asciiP & 0xff) == 'P') {
                    /* ID Prom is Format 1 */
                    clkControl |= ((id->asciiC & 0xff) == 'H') << slot;
                } else {
                    /* ID Prom is Format 2 */
                    ipac_idProm2_t *id2 = (ipac_idProm2_t *) id;

                    word = id2->flags;
                    if (word & 1) {
                        printf("AVME-IP: IP module at (%d,%d) has flags = %x\n",
                            carrier, slot, word);
                        continue;
                    }
                    if (word & 4)
                        clkControl |= 1 << slot;
                }
            }
        }

        if (clkControl)
            regs->clkControl = clkControl;
    }

    /* Now finish parsing the parameter string */
    while (*cardParams) {
        if (3 != sscanf(cardParams, "%1[ABCDabcd] = %1[1248], %x %n",
                memSlot, memSize, &memBase, &skip)) {
            printf("AVME-IP: Error parsing slot configuration '%s'\n",
                cardParams);
            return S_IPAC_badAddress;
        }
        cardParams += skip;

        slot = tolower((int) *memSlot) - 'a';
        assert(slot < SLOTS);

        *memSize -= '0';
        memCtl = memBits[(int) *memSize];
        assert(memCtl >= 0);

        if (memBase & memMask[memCtl]) {
            printf("AVME-IP: Slot %c bad memory base address %x\n",
                *memSlot, memBase);
            return S_IPAC_badAddress;
        }
        memCtl |= (memBase >> 16) & 0xf0;

        /* This also checks for overlapping memory areas */
        status = devRegisterAddress("AVME-IP", atVMEA24, memBase,
            *memSize << 20, &ptr);
        if (status) {
            printf("AVME-IP: Can't map VME address A24:%6.6x\n", memBase);
            return status;
        }
        private->addr[ipac_addrMem ][slot] = (void *) ptr;

        regs->memCtl[slot] = memCtl;
        regs->memEnable |= 1 << slot;
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
    Because we did all the calculations in the initialise routine, this code
    only has to do a table lookup from an array in the private area. Note that
    no parameter checking is required since we have array entries for all
    combinations of slot and space - the IPAC driver which calls this routine
    handles that.

Returns:
    The requested address, or NULL if the module has no memory.

*/

static void *baseAddr (
    void *p,
    epicsUInt16 slot,
    ipac_addr_t space
) {
    private_t *private = (private_t *)p;
    return private->addr[space][slot];
}


/*******************************************************************************

Routine:
    irqCmd

Purpose:
    Handles interrupter commands and status requests

Description:
    The XVME-9660 only supports one interrupt level for all IP modules, but it
    does allow the interrupt signals to be individually enabled, disabled and
    polled. This driver also supports the irqClear command, although the board
    is configured so it should not be necessary to use it.

Returns:
    ipac_irqGetLevel returns the interrupt level,
    ipac_irqEnable, ipac_irqDisable and ipac_irqClear return 0 = OK,
    ipac_irqPoll returns non-zero if an interrupt is pending, else 0,
    other calls return S_IPAC_notImplemented.

*/

static int irqCmd (
    void *p,
    epicsUInt16 slot,
    epicsUInt16 irqNumber,
    ipac_irqCmd_t cmd
) {
    private_t *private = (private_t *)p;
    volatile ctrl_t *regs = private->regs;
    int irqBit = 1 << (irqNumber + 2*slot);

    switch (cmd) {
        case ipac_irqGetLevel:
            return regs->irqLevel & 7;

        case ipac_irqEnable:
            regs->irqClear = irqBit;
            regs->irqEnable |= irqBit;
            return OK;

        case ipac_irqDisable:
            regs->irqEnable &= ~ irqBit;
            return OK;

        case ipac_irqPoll:
            return regs->irqStatus & irqBit;

        case ipac_irqClear:
            regs->irqClear = irqBit;
            return OK;

        default:
            return S_IPAC_notImplemented;
    }
}


/******************************************************************************/

/* IPAC Carrier Table */

static ipac_carrier_t avme96XX = {
    "Acromag AVME-96xx",
    SLOTS,
    initialise,
    NULL,
    baseAddr,
    irqCmd,
    NULL,
    NULL
};


int ipacAddXy9660(const char *cardParams) {
    return ipacAddCarrier(&avme96XX, cardParams);
}

int ipacAddAvme96XX(const char *cardParams) {
    return ipacAddCarrier(&avme96XX, cardParams);
}


/* iocsh Command Table and Registrar */

static const iocshArg arg0 =
    {"cardParams", iocshArgString};
static const iocshArg * const avmeArgs[] =
    {&arg0};

static const iocshFuncDef xyFuncDef =
    {"ipacAddXy9660", NELEMENTS(avmeArgs), avmeArgs};
static const iocshFuncDef avmeFuncDef =
    {"ipacAddAvme96XX", NELEMENTS(avmeArgs), avmeArgs};

static void avmeCallFunc(const iocshArgBuf *args) {
    ipacAddAvme96XX(args[0].sval);
}

static void epicsShareAPI xy9660Registrar(void) {
    iocshRegister(&xyFuncDef, avmeCallFunc);
    iocshRegister(&avmeFuncDef, avmeCallFunc);
}

epicsExportRegistrar(xy9660Registrar);
