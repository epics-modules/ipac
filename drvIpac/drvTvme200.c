/*******************************************************************************

Project:
    CAN Bus Driver for EPICS

File:
    drvTvme200.c

Description:
    IPAC Carrier Driver for the Tews TVME200 (a.k.a. SBS VIPC626) Quad
    IndustryPack Carrier VME board, it provides the interface between IPAC
    driver and the hardware.  This carrier is 6U high and can support VME
    Extended mode addresses, but not 32-bit access to dual-slot IP modules.
    The board can be software compatible with the VIPC616 and VIPC610
    carriers, but has much better configurability, using hex switches
    for board configuration.  Each slot also has two control registers that
    can be used to read or set VME interrupt levels and to reset individual
    IP modules.

Author:
    Andrew Johnson <anjohnson@iee.org>
Created:
    10 December 2004

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

Modifications:
   24-Apr-2006  Wayne Lewis   Added devLib code for non-vxWorks systems

*******************************************************************************/

/* ANSI headers */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/* EPICS headers */
#include <epicsTypes.h>
#include <errMdef.h>
#include <dbDefs.h>
#include <devLib.h>
#include <epicsThread.h>
#include <iocsh.h>
#include <epicsExport.h>

/* Module headers */
#include "drvIpac.h"


/* Characteristics of the card */

#define SLOTS 4         /* Number of IP slots */
#define IO_SPACES 2     /* Address spaces in A16 */
#define SETTINGS 5      /* Defined S3 positions */
#define EXTENT 0x400    /* Register size in A16 */


/* Offsets from base address in VME A16 space */

#define REGS_A 0x0000
#define PROM_A 0x0080
#define CTRL_A 0x00c0
#define REGS_B 0x0100
#define PROM_B 0x0180
#define CTRL_B 0x01c0
#define REGS_C 0x0200
#define PROM_C 0x0280
#define CTRL_C 0x02c0
#define REGS_D 0x0300
#define PROM_D 0x0380
#define CTRL_D 0x03c0

static const int tvmeAddrs[IO_SPACES][SLOTS] = {
    { PROM_A, PROM_B, PROM_C, PROM_D },
    { REGS_A, REGS_B, REGS_C, REGS_D }
};

static const int tvmeCtrls[SLOTS] = {
    CTRL_A, CTRL_B, CTRL_C, CTRL_D
};

/* Default VME Interrupt levels */

/* These values are the settings for the IRQ config registers.
 * Note that the high nybble is the slot's IRQ1 level, the low
 * nybble that for IRQ0, so they may look reversed. */

static const int tvmeIrqs[SETTINGS][SLOTS] = {
    { 0x00, 0x00, 0x00, 0x00 },     /* S3=0 */
    { 0x21, 0x43, 0x65, 0x07 },     /* S3=1 */
    { 0x54, 0x12, 0x54, 0x12 },     /* S3=2 */
    { 0x46, 0x13, 0x46, 0x13 },     /* S3=3 */
    { 0x11, 0x22, 0x33, 0x44 }      /* S3=4 */
};


/* IRQ and Control Registers */

typedef struct {
    epicsInt16 irqLevel;  /* IRQ0 = bits 0-2, IRQ1 = bits 4-6 */
    epicsInt16 control;   /* 0 = IRQ0, 1 = IRQ1, 2 = Error, 7 = Reset */
} ctrl_t;


/* Carrier Private structure, one instance per board */

typedef struct {
    void *addr[IPAC_ADDR_SPACES][SLOTS];
    ctrl_t *ctrl[SLOTS];
} private_t;


/*******************************************************************************

Routine:
    initialise

Purpose:
    Registers a new TVME200 with addresses given by cardParams

Description:
    Reads the parameter string for settings of the 6 rotary switches on
    the card, checks these for legality, and creates a private table
    for the card which contains arrays of pointers to the base addresses
    of the various accessible parts of each IP module.

Parameters:
    The parameter string should comprise exactly 6 hexadecimal digits,
    one for each board switch setting, given in order S1 through S6.

Examples:
    "6010D0"
        This indicates that the carrier board has its I/O base at 0x6000,
        the memory space has been disabled.  This is the board's factory
        default setting.
    "602FB0"
        Here the I/O base is set to 0x6000, and the module memory areas
        are in the VMEbus A32 space at 0xB0000000, so module A's memory
        starts at 0xB0000000, module B's is at 0xB0800000, module C's at
        0xB1000000 and D's is at 0xB1800000.  This is a common setting
        for boards used at the APS.
    "102670"
        The I/O base is at 0x1000, and the carrier memory base is in A24
        space at 0x700000.  However the memory size is configured to be
        1MB per module but the A24 address is not a multiple of 4MB, so
        this configuration is illegal and will be rejected by the
        carrier initialization code.

Returns:
    0 = OK, 
    S_IPAC_badAddress = Parameter string error, or address not reachable

*/

static int initialise (
    const char *cardParams,
    void **pprivate,
    epicsUInt16 carrier
) {
    int s3, s4, mAM;
    epicsUInt32 switches, ioBase, mSize, mBase;
    volatile void *ptr;
    char *ioPtr, *mPtr;
    int space, slot;
    private_t *settings;

    if (cardParams == NULL ||
        strlen(cardParams) != 6)
        return S_IPAC_badAddress;

    switches = strtoul(cardParams, NULL, 16);
    ioBase = (switches >> 8) & 0xff00;
    s3 = (switches >> 12) & 0xf;
    s4 = (switches >> 8) & 0xf;
    mBase = (switches << 16) & 0xff0000;

    if ((ioBase & 0x0300) || s3 >= SETTINGS)
        return S_IPAC_badAddress;

    if (devRegisterAddress("TVME200", atVMEA16, ioBase, EXTENT, &ptr))
        return S_IPAC_badAddress;
    ioPtr = (char *) ptr;

    for (slot = 0; slot < SLOTS; slot++) {
        ctrl_t *ctrl = (ctrl_t *) (ioPtr + tvmeCtrls[slot]);
        int reg = ctrl->irqLevel & 0x77;
        int set = tvmeIrqs[s3][slot];
        /* Correct and warn if levels are wrong */
        if (reg != set) {
            ctrl->irqLevel = set;
            printf("TVME200: Card %d slot %d Int levels fixed %d+%d => %d+%d\n",
                   carrier, slot, reg & 7, reg >> 4 & 7, set & 7, set >> 4 & 7);
        }
    }

    switch (s4) {
    case 0:
        mSize = 0;
        mAM = 0;                /* prevent compiler warning */
        break;
        
    case 1: case 2: case 3: case 4: case 5: case 6: case 7:
        /* A24, variable size per module */
        mSize = 16384 << s4;    /* Calculate size: 1=32KB, 2=64KB, ... */
        mAM = atVMEA24;
        break;
        
    case 0xf:
        /* A32, 8MB allocated per module */
        mSize = 8 << 20;        /* 8MB */
        mBase <<= 8;
        mAM = atVMEA32;
        break;
        
    default:
        return S_IPAC_badAddress;
    }

    if (mSize &&
        (((mSize * SLOTS - 1) & mBase) ||       /* address must match size */
        devRegisterAddress("TVME200", mAM, mBase, mSize * SLOTS,  &ptr))) {
        return S_IPAC_badAddress;
    }
    mPtr = (char *) ptr;

    settings = (private_t *)malloc(sizeof (private_t));
    if (!settings)
        return S_IPAC_noMemory;

    for (space = 0; space < IO_SPACES; space++) {
        for (slot = 0; slot < SLOTS; slot++) {
            settings->addr[space][slot] = (void *)
                (ioPtr + tvmeAddrs[space][slot]);
        }
    }

    for (slot = 0; slot < SLOTS; slot++) {
        settings->addr[ipac_addrMem][slot] = (void *) (mPtr + mSize * slot);
        settings->ctrl[slot] = (ctrl_t *) (ioPtr + tvmeCtrls[slot]);
        settings->addr[ipac_addrIO32][slot] = NULL;
    }

    *pprivate = (void *)settings;
    return OK;
}

/*******************************************************************************

Routine:
    report

Purpose:
    Returns a status string for the requested slot

Description:
    The status indicates if the module is asserting its error signal, then
    for each interrupt line it shows the interrupt level and whether that
    interrupt signal is currently active.

Returns:
    A static string containing the slot's status.

*/

static char *report (
    void *private,
    epicsUInt16 slot
) {
    private_t *settings = (private_t *)private;
    volatile ctrl_t *ctrl = settings->ctrl[slot];
    int irq = ctrl->irqLevel;
    int ctl = ctrl->control;
    static char output[IPAC_REPORT_LEN];

    sprintf(output, "%sInt0: level %d%s    Int1: level %d%s", 
            (ctl & 4 ? "Error signal    " : ""),
            (irq & 7), (ctl & 1 ? ", active" : ""),
            (irq >> 4 & 7), (ctl & 2 ? ", active" : ""));
    return output;
}

/*******************************************************************************

Routine:
    baseAddr

Purpose:
    Returns the base address for the requested slot & address space

Description:
    Because we did all that hard work in the initialise routine, this 
    routine only has to do a table lookup in the private settings array.
    Note that no parameter checking is required - the IPAC driver which 
    calls this routine handles that.

Returns:
    The requested address, or NULL if the module has no memory.

*/

static void *baseAddr (
    void *private,
    epicsUInt16 slot,
    ipac_addr_t space
) {
    private_t *settings = (private_t *)private;
    return settings->addr[space][slot];
}


/*******************************************************************************

Routine:
    irqCmd

Purpose:
    Handles interrupter commands and status requests

Description:

    The carrier board provides a switch to select from 5 default interrupt
    level settings, and a control register to allow these to be overridden.
    The commands supported include setting and fetching the current interrupt
    level associated with a particular slot and interrupt number, enabling
    interrupts by making sure the VMEbus interrupter is listening on the
    relevent level, and the abilty to ask whether a particular slot interrupt
    is currently pending or not.

Returns:
    ipac_irqLevel0-7 return 0 = OK,
    ipac_irqGetLevel returns the current interrupt level,
    ipac_irqEnable returns 0 = OK,
    ipac_irqPoll returns 0 = no interrupt or 1 = interrupt pending,
    other calls return S_IPAC_notImplemented.

*/

static int irqCmd (
    void *private,
    epicsUInt16 slot,
    epicsUInt16 irqNumber,
    ipac_irqCmd_t cmd
) {
    private_t *settings = (private_t *)private;
    volatile ctrl_t *ctrl = settings->ctrl[slot];
    int reg, iShift = irqNumber * 4;

    switch (cmd) {
        case ipac_irqLevel0:
        case ipac_irqLevel1:
        case ipac_irqLevel2:
        case ipac_irqLevel3:
        case ipac_irqLevel4:
        case ipac_irqLevel5:
        case ipac_irqLevel6:
        case ipac_irqLevel7:
            reg = ctrl->irqLevel & ~(7 << iShift);
            ctrl->irqLevel = reg | cmd << iShift;
            return OK;

        case ipac_irqGetLevel:
            return (ctrl->irqLevel >> iShift) & 7;

        case ipac_irqEnable:
            devEnableInterruptLevel(intVME, (ctrl->irqLevel >> iShift) & 7);
            return OK;

        case ipac_irqPoll:
            return (ctrl->control >> irqNumber) & 1;

        case ipac_slotReset:
            ctrl->control = 0x80;
            while (ctrl->control & 0x80)
                epicsThreadSleep(0.05);
            return OK;

        default:
            return S_IPAC_notImplemented;
    }
}


/******************************************************************************/

/* IPAC Carrier Table */

static ipac_carrier_t tvme200 = {
    "TEWS TVME200",
    SLOTS,
    initialise,
    report,
    baseAddr,
    irqCmd,
    NULL,
    NULL
};

int ipacAddTVME200(const char *cardParams) {
    return ipacAddCarrier(&tvme200, cardParams);
}


/* iocsh Command Table and Registrar */

static const iocshArg vipcArg0 =
    {"cardParams",iocshArgString};
static const iocshArg * const Args[] =
    {&vipcArg0};

static const iocshFuncDef tvme200FuncDef =
    {"ipacAddTVME200", NELEMENTS(Args), Args};

static void tvme200CallFunc(const iocshArgBuf *args) {
    ipacAddTVME200(args[0].sval);
}

static void epicsShareAPI tvme200Registrar(void) {
    iocshRegister(&tvme200FuncDef, tvme200CallFunc);
}

epicsExportRegistrar(tvme200Registrar);
