/*******************************************************************************

Project:
    Hytec 8002 and 8004 Carrier Driver for EPICS

File:
    drvHy8002.c

Description:
    IPAC Carrier Driver for the Hytec VICB8002 and 8004 VME64x Quad IP Carrier
    boards. This file provides the board-specific interface between IPAC driver
    and the hardware. These carrier boards are 6U high and can support VME64x
    geographic addresses. The carrier has 4 IP card slots and can be configured
    to use any of the 7 interrupt levels (1 ~ 7).  The hotswap capability of the
    8002 board is not supported in this driver.

Authors:
    Andrew Johnson, Argonne National Laboratory
    Walter Scott (aka Scotty), HyTec Electronics Ltd
    Jim Chen, HyTec Electronics Ltd

Portions Copyright (c) 1995-1999 Andrew Johnson
Portions Copyright (c) 1999-2013 UChicago Argonne LLC,
    as Operator of Argonne National Laboratory
Portions Copyright (c) 2002-2010 Hytec Electronics Ltd, UK.

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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* EPICS headers */
#include <epicsTypes.h>
#include <errMdef.h>
#include <dbDefs.h>
#include <devLib.h>
#include <epicsTypes.h>
#include <epicsExit.h>
#include <iocsh.h>
#include <epicsExport.h>

/* Module headers */
#include "drvIpac.h"


/* Characteristics of the card */

#define SLOTS 4         /* Number of IP slots */
#define IO_SPACES 2     /* IP Address spaces in A16/A24 */
#define EXTENT 0x800    /* Total footprint in A16/A24 space */


/* Default configuration parameters */

#define DEFAULT_IPMEM   1
#define DEFAULT_IPCLOCK -1
#define DEFAULT_ROAK    0
#define DEFAULT_MEMBASE -1


/* Offsets from base address in VME A16 space */

#define IP_A_IO     0x0000
#define IP_A_ID     0x0080
#define IP_B_IO     0x0100
#define IP_B_ID     0x0180
#define IP_C_IO     0x0200
#define IP_C_ID     0x0280
#define IP_D_IO     0x0300
#define IP_D_ID     0x0380
#define REGISTERS   0x400
#define CONFIG_ROM  0x600

static const epicsUInt16 ipOffsets[IO_SPACES][SLOTS] = {
    { IP_A_ID, IP_B_ID, IP_C_ID, IP_D_ID },
    { IP_A_IO, IP_B_IO, IP_C_IO, IP_D_IO }
};


/* Carrier board registers */

typedef struct {
    epicsUInt16 ipstat;
    epicsUInt16 pad_02;
    epicsUInt16 memoff;
    epicsUInt16 pad_06;
    epicsUInt16 csr;
    epicsUInt16 pad_0a;
    epicsUInt16 intsel;
    epicsUInt16 pad_0e;
} ctrl_t;


/* Bit patterns for the CSR register */

#define CSR_RESET       0x0001
#define CSR_INTEN       0x0002
#define CSR_INTSEL0     0x0004
#define CSR_CLKSEL      0x0020
#define CSR_MEMMODE     0x0040
#define CSR_IPMS_1MB    0x0000
#define CSR_IPMS_2MB    0x0080
#define CSR_IPMS_4MB    0x0100
#define CSR_IPMS_8MB    0x0180
#define CSR_INTRELS     0x0200
#define CSR_IPACLK      0x0400  /* 8004 only */


/* Offsets to fields in the VME64x Config ROM */

#define VME64CR_VALIDC  0x01F
#define VME64CR_VALIDR  0x023

#define VME64CR_MAN1    0x027
#define VME64CR_MAN2    0x02B
#define VME64CR_MAN3    0x02F

#define VME64CR_MOD1    0x033
#define VME64CR_MOD2    0x037
#define VME64CR_MOD3    0x03B
#define VME64CR_MOD4    0x03F

#define VME64CR_REVN    0x043
#define VME64CR_XIL1    0x047
#define VME64CR_XIL2    0x04B
#define VME64CR_XIL3    0x04F

#define VME64CR_SER1    0x0CB
#define VME64CR_SER2    0x0CF
#define VME64CR_SER3    0x0D3
#define VME64CR_SER4    0x0D7
#define VME64CR_SER5    0x0DB
#define VME64CR_SER6    0x0DF


/* ID values used in the ROM */

#define IEEE_MANUFACTURER_HYTEC 0x008003
#define HYTEC_MODEL_8002        0x80020000u
#define HYTEC_MODEL_8004        0x80040000u


/* Carrier Private structure, one instance per board */

typedef struct private_t {
    struct private_t * next;
    int carrier;
    volatile ctrl_t * regs;
    volatile epicsUInt8 * prom;
    void * addr[IPAC_ADDR_SPACES][SLOTS];
} private_t;


/* Module Variables */

static private_t *list_head = NULL;
static const char * const drvname = "drvHy8002";


/*******************************************************************************

Routine: internal function
    scanparm

Purpose:
    Parses configuration parameters

Description:
    This function parses the configuration parameter string to extract the
    mandatory VME slot number and interrupt level. Following these the string
    may also include settings for IP memory size, clock frequency, interrupt
    release and/or the IP memory base address; default values are set for any
    of these optional parameters that are not included.

Parameters:

    The parameter string starts with two decimal integers separated by a comma,
    which give the board's VME slot number (0 through 21) and the VME interrupt
    level to be used (0 through 7, 0 means interrupts are disabled). If a VME64x
    backplane is not being used to provide geographical addressing information,
    the slot number parameter must match the value set in jumpers J6 through
    J10, with J6 being the LSB and an installed jumper giving a 1 bit.

    After the above mandatory parameters, any of the following optional
    parameters may appear in any order. Space characters cannot be used on
    either side of the equals sign:

    IPMEM=<size>
        Sets the extent of the memory space allocated for each IP slot. The size
        parameter is a single digit 1, 2, 4 or 8 which gives the memory size to
        be used, expressed in MegaBytes. If this parameter is not provided the
        default value of 1 will be used.

    MEMBASE=<address>
        Specifies the top 16 bits of the base address of the A32 memory area to
        be used for IP slot memory space, overriding the jumper-selected or
        VME64x geographical address that will be used if this parameter is not
        provided. The address string should be a decimal integer or a number hex
        with a leading 0x. The resulting base address must be a multiple of 4
        times the slot memory size; an error message will be printed if this
        restriction is not met.

    IPCLK=<frequency>
        Specifies the frequency in MHz of the IP clock passed to all slots. The
        frequency parameter must be either 8 or 32. If not specified for a 8002
        carrier the IP clock will default to 8MHz, while for a 8004 carrier the
        driver will then configure each slot individually based on information
        from the ID prom of the module installed; specifying this parameter for
        an 8004 board overrides the module's configuration for all slots.

    ROAK=<release>
        This parameter controls when an 8004 carrier board will releases the
        interrupt request to the VMEbus. Giving release as 1 will cause the
        request to be released by the VME Interrupt Acknowledge cycle (ROAK
        protocol). If release is given as 0 or the parameter is not specified at
        all, the interrupt request will be released by the module's interrupt
        service routine clearing the module's reason for the interrupt request
        (RORA protocol) or disabling the interrupt using ipmIrqCmd(carrier,
        slot, irqn, ipac_irqDisable).

Examples:
    ipacAddHy8002("3,2")
        The board is in slot 3 of a VME64x crate, or jumpers are installed in
        positions J6 and J7. Module interrupts use VMEbus level 2. IP slots get
        1MB of memory in the block starting at A32:0x00c00000. IP clocks use the
        default described above.

    ipacAddHy8002("3,2 IPMEM=2, MEMBASE=0x9000,IPCLCK=32 ROAK=1")
        This modifies the above configuration to give each slot 2MB of memory in
        the block starting at A32:0x90000000, drives all IP clocks at 32MHz, and
        uses the ROAK protocol for releasing interrupt requests.

Return:
    0 = OK if successful
    S_IPAC_badAddress = Error in string

*/

static int scanparm (
    const char *params,
    int *vmeslot,
    int *intlevel,
    int *ipmem,
    int *ipclock,
    int *roak,
    int *membase
) {
    int vme = 0, itr = 0;
    int skip = 0;
    const char *pstart;
    int count;

    if (params == NULL || *params == 0)
        return S_IPAC_badAddress;

    count = sscanf(params, "%d, %d %n", &vme, &itr, &skip);
    if (count != 2) {
        printf("%s: Error parsing card configuration '%s'\n", drvname, params);
        return S_IPAC_badAddress;
    }

    if (vme < 0 || vme > 21) {
        printf("%s: Bad VME slot number %d\n", drvname, vme);
        return S_IPAC_badAddress;
    }
    else
        *vmeslot = vme;

    if (itr < 0 || itr > 7) {
        printf("%s: Bad VME interrupt level %d\n", drvname, itr);
        return S_IPAC_badAddress;
    }
    else
        *intlevel = itr;

    params += skip;

    /* Parse IP memory size */
    if ((pstart = strstr(params, "IPMEM=")) != NULL) {
        int ipm;

        if ((1 != sscanf(pstart+6, "%d", &ipm)) ||
            (ipm != 1 && ipm != 2 && ipm != 4 && ipm != 8))
            return S_IPAC_badAddress;
        *ipmem = ipm;
    }

    /* Parse memory base address */
    if ((pstart = strstr(params, "MEMBASE=")) != NULL) {
        int mem;

        if ((1 != sscanf(pstart+8, "%i", &mem)) ||
            mem < 0 || mem + (*ipmem << 6) > 0xffff)
            return S_IPAC_badAddress;
        *membase = mem;
    }

    /* Parse IP clock frequency */
    if ((pstart = strstr(params, "IPCLCK=")) != NULL) {
        int ipc;

        if ((1 != sscanf(pstart+7, "%d", &ipc)) ||
            (ipc != 8 && ipc != 32))
            return S_IPAC_badAddress;
        *ipclock = ipc;
    }

    /* Parse ROAK setting */
    if ((pstart = strstr(params, "ROAK=")) != NULL) {
        int ro;

        if ((1 != sscanf(pstart+5, "%d", &ro)) ||
            (ro !=0 && ro !=1))
            return S_IPAC_badAddress;
        *roak = ro;
    }

    return OK;
}


/*******************************************************************************

Routine:
    checkVMEprom

Purpose:
    Ensure the card is an 8002 or 8004 carrier

Description:
    This function checks that addressed VME slot has a configuration ROM
    at the expected location, and makes sure it identifies the module as
    being a Hytec 8002 or 8004 card.

Returns:
    0 = OK
    S_IPAC_noModule = Bus Error accessing card ROM
    S_IPAC_badModule = ROM contents did not match

*/

static int checkVMEprom(volatile epicsUInt8 *prom)
{
    epicsUInt32 id;
    char valid;

    /* Make sure the Configuration ROM exists */
    if (devReadProbe(1, &prom[VME64CR_VALIDC], &valid)) {
        printf("%s: Bus Error accessing card, check configuration\n", drvname);
        return S_IPAC_noModule;
    }
    if (valid != 'C' || prom[VME64CR_VALIDR] != 'R') {
        printf("%s: Configuration ROM not found, check address\n", drvname);
        return S_IPAC_badModule;
    }

    /* Manufacturer ID */
    id = (prom[VME64CR_MAN1] << 16) +
        (prom[VME64CR_MAN2] << 8) + prom[VME64CR_MAN3];

    if (id != IEEE_MANUFACTURER_HYTEC) {
        printf("%s: Manufacturer ID is %x, expected %x\n",
            drvname, id, IEEE_MANUFACTURER_HYTEC);
        return S_IPAC_badModule;
    }

    /* Board ID */
    id = (prom[VME64CR_MOD1] << 24) + (prom[VME64CR_MOD2] << 16) +
        (prom[VME64CR_MOD3] << 8) + prom[VME64CR_MOD4];

    if (id != HYTEC_MODEL_8002 && id != HYTEC_MODEL_8004) {
        printf("%s: Board ID is %x, expected %x or %x\n",
            drvname, id, HYTEC_MODEL_8002, HYTEC_MODEL_8004);
        return S_IPAC_badModule;
    }

    return OK;
}


/*******************************************************************************

Routine:
    shutdown

Purpose:
    Disable interrupts on IOC reboot

Description:
    This function is registered as an epicsAtExit routine which disables
    interrupts from the carrier when the IOC is shuttind down.

Returns:
    N/A

*/

static void shutdown(void *r) {
    volatile ctrl_t *regs = (ctrl_t *)r;

    regs->csr &= ~CSR_INTEN;
}


/*******************************************************************************

Routine:
    initialise

Purpose:
    Registers a new Hy8002 or 8004 carrier board

Description:
    Parses the parameter string for the card settings, initializes the card,
    sets its interrupt level and memory base address and creates a new private
    table with the necessary address information.

Parameters:
    See the scanparm routine above for a description of the parameter string.

Returns:
    0 = OK,
    S_IPAC_badAddress = Parameter string error
    S_IPAC_noMemory = malloc() failed
    S_dev_ errors returned by devLib

*/

static int initialise (
    const char *params,
    void **pprivate,
    epicsUInt16 carrier
) {
    int status;
    int vmeslot, intlevel;
    int ipmem   = DEFAULT_IPMEM;
    int ipclock = DEFAULT_IPCLOCK;
    int roak    = DEFAULT_ROAK;
    int membase = DEFAULT_MEMBASE;
    epicsUInt32 baseaddr, memaddr;
    char *baseptr, *memptr;
    volatile void *ptr;
    volatile epicsUInt8 *prom;
    volatile ctrl_t *regs;
    size_t memsize;
    epicsUInt16 csr;
    private_t* private;
    int space, slot;

    status = scanparm(params, &vmeslot, &intlevel,
        &ipmem, &ipclock, &roak, &membase);
    if (status)
        return status;

    /* Refuse illegal configurations */
    if (ipmem == 2 && vmeslot > 15) {
        printf("%s: Can't use Geographical slot %d (> 15) with IPMEM=2\n",
            drvname, vmeslot);
        return S_IPAC_badAddress;
    }
    if (ipmem == 4 && membase < 0) {
        printf("%s: Can't use Geographical adressing with IPMEM=4\n", drvname);
        return S_IPAC_badAddress;
    }

    /* Reserve our A16 memory space */
    baseaddr = vmeslot << 11;
    status = devRegisterAddress(drvname, atVMEA16, baseaddr, EXTENT, &ptr);
    if (status) {
        printf("%s: Can't map VME address A16:0x%4.4x\n", drvname, baseaddr);
        return status;
    }
    baseptr = (char *) ptr;
    regs = (ctrl_t *) (baseptr + REGISTERS);
    prom = (epicsUInt8 *) (baseptr + CONFIG_ROM);

    /* Ensure this really is a Hytec 8002 or 8004 */
    status = checkVMEprom(prom);
    if (status) {
        devUnregisterAddress(atVMEA16, baseaddr, drvname);
        return status;
    }

    /* Reserve the A24 memory space the card occupies (not used) */
    baseaddr = vmeslot << 19;
    status = devRegisterAddress(drvname, atVMEA24, baseaddr, EXTENT, &ptr);
    if (status) {
        printf("%s: Can't map VME address A24:0x%6.6x\n", drvname, baseaddr);
        return status;
    }

    /* Reserve our A32 memory space */
    if (membase < 0) {
        switch (ipmem) {
        case 1:
            memaddr = (epicsUInt32) vmeslot << 22;
            break;
        case 2:
            memaddr = (epicsUInt32) vmeslot << 23;
            break;
        case 8:
            memaddr = (epicsUInt32) vmeslot << 27;
            break;
        default:
            printf("%s: Internal Error 1\n", drvname);
            return S_IPAC_badAddress;
        }
    }
    else {
        int mask = (ipmem << 6) - 1;

        if (membase & mask) {
            printf("%s: MEMBASE=0x%4.4x is incompatible with IPMEM=%d\n",
                drvname, membase, ipmem);
            membase &= ~mask;
            printf("\tNearest allowed settings are 0x%4.4x or 0x%4.4x\n",
                membase, membase + (ipmem << 6));
            return S_IPAC_badAddress;
        }
        memaddr = (epicsUInt32) membase << 16;
    }
    memsize = ipmem << 20;
    status = devRegisterAddress(drvname, atVMEA32, memaddr, memsize * 4, &ptr);
    if (status) {
        printf("%s: Can't map VME address A32:%8.8x\n", drvname, memaddr);
        return status;
    }
    memptr = (char *) ptr;

    /* Calculate the CSR value */
    csr = intlevel * CSR_INTSEL0 | CSR_INTEN;

    if (membase >= 0)
        csr |= CSR_MEMMODE;
    if (ipclock == 32)
        csr |= CSR_CLKSEL;
    if (roak)
        csr |= CSR_INTRELS;

    switch (ipmem) {
    case 1:
        csr |= CSR_IPMS_1MB;
        break;
    case 2:
        csr |= CSR_IPMS_2MB;
        break;
    case 4:
        csr |= CSR_IPMS_4MB;
        break;
    case 8:
        csr |= CSR_IPMS_8MB;
        break;
    default:
        printf("%s: Internal Error 2\n", drvname);
        return S_IPAC_badAddress;
    }

    /* Create a private structure */
    private = (private_t *) malloc(sizeof(private_t));
    if (private == NULL)
        return S_IPAC_noMemory;

    private->next = list_head;
    private->carrier = carrier;
    private->regs = regs;
    private->prom = prom;

    /* Fill in slot memory addresses */
    for (space = 0; space < IO_SPACES; space++) {
        for (slot = 0; slot < SLOTS; slot++) {
            private->addr[space][slot] = baseptr + ipOffsets[space][slot];
        }
    }
    for (slot = 0; slot < SLOTS; slot++) {
        private->addr[ipac_addrIO32][slot] = NULL;
        private->addr[ipac_addrMem ][slot] = memptr + memsize * slot;
    }

    /* On the 8004, use 32MHz clocks where modules support them */
    if (ipclock < 0 && prom[VME64CR_MOD2] == 0x04) {
        for (slot = 0; slot < SLOTS; slot++) {
            epicsUInt16 word;
            ipac_idProm_t *id = (ipac_idProm_t *)
                private->addr[ipac_addrID][slot];

            if (!devReadProbe(sizeof(word), (void *)&id->asciiI, (char *)&word)
                && ipcCheckId(id) == OK) {
                if ((id->asciiP & 0xff) == 'P') {
                    /* ID Prom is Format 1 */
                    if ((id->asciiC & 0xff) == 'H')
                        csr |= CSR_IPACLK << slot;
                } else {
                    /* ID Prom is Format 2 */
                    ipac_idProm2_t *id2 = (ipac_idProm2_t *) id;
                    word = id2->flags;

                    if (word & 1) {
                        printf("%s: IP module at (%d,%d) has flags = %x\n",
                            drvname, carrier, slot, word);
                        continue;
                    }
                    if (word & 4)
                        csr |= CSR_IPACLK << slot;
                }
            }
        }
    }

    /* Set card registers */
    if (membase >= 0)
        regs->memoff = membase;
    regs->intsel = 0;
    regs->csr    = csr;

    epicsAtExit(shutdown, (void *) regs);
    devEnableInterruptLevel(intVME, intlevel);

    /* Insert into list and pass back to drvIpac */
    list_head = private;
    *pprivate = private;
    return OK;
}


/*******************************************************************************

Routine:
    report

Purpose:
    Returns a status string for the requested slot

Description:
    The status indicates if the module is asserting its error signal, then
    for each interrupt line it shows whether the interrupt is enabled and
    if that interrupt signal is currently active.

Returns:
    A static string containing the slot's current status.

*/

static char* report (
    void *p,
    epicsUInt16 slot
) {
    private_t *private = (private_t *) p;
    volatile ctrl_t *regs = private->regs;
    volatile epicsUInt8 *prom = private->prom;
    int ipstat = regs->ipstat;
    int intsel = regs->intsel;
    static char output[IPAC_REPORT_LEN];

    switch (prom[VME64CR_MOD2]) {
    case 0x02:
        sprintf(output, "%sInt0: %sabled%s    Int1: %sabled%s",
            (ipstat & (0x100 << slot) ? "Slot Error    " : ""),
            (intsel & (0x001 << slot) ? "en" : "dis"),
            (ipstat & (0x001 << slot) ? ", active" : ""),
            (intsel & (0x010 << slot) ? "en" : "dis"),
            (ipstat & (0x010 << slot) ? ", active" : ""));
        break;
    case 0x04:
        sprintf(output, "%sInt0: %sabled%s    Int1: %sabled%s",
            (ipstat & 0x100 ? "IP Error    " : ""),
            (intsel & (0x001 << slot) ? "en" : "dis"),
            (ipstat & (0x001 << slot) ? ", active" : ""),
            (intsel & (0x010 << slot) ? "en" : "dis"),
            (ipstat & (0x010 << slot) ? ", active" : ""));
        break;
    }
    return output;
}


/*******************************************************************************

Routine:
    baseAddr

Purpose:
    Returns a pointer to the requested slot & address space

Description:
    Because we did all the calculations in the initialise routine, this code
    only has to do a table lookup from an array in the private area. Note that
    no parameter checking is required since we have array entries for all
    combinations of slot and space - the IPAC driver which calls this routine
    handles that.

Returns:
    The requested pointer, or NULL if the module has no memory.

*/

static void * baseAddr (
    void *p,
    epicsUInt16 slot,
    ipac_addr_t space
) {
    private_t *private = (private_t *) p;
    return private->addr[space][slot];
}

/*******************************************************************************

Routine:
    irqCmd

Purpose:
    Handles interrupter commands and status requests

Description:
    The board supports one interrupt level for all IP modules which cannot be
    changed after initialisation.  This routine returns the level for module
    drivers to use, and allows interrupt signals to be individually enabled,
    disabled and polled.

Returns:
    ipac_irqGetLevel returns the interrupt level,
    ipac_irqEnable and ipac_irqDisable return 0 = OK,
    ipac_irqPoll returns non-zero if an interrupt pending, else 0,
    other calls return S_IPAC_notImplemented.

*/

static int irqCmd (
    void *p,
    epicsUInt16 slot,
    epicsUInt16 irqNumber,
    ipac_irqCmd_t cmd
) {
    private_t* private = (private_t *) p;
    volatile ctrl_t *regs = private->regs;
    int irqBit = 1 << (4 * irqNumber + slot);

    switch(cmd) {
        case ipac_irqGetLevel:
            return (regs->csr / CSR_INTSEL0) & 7;

        case ipac_irqEnable:
            regs->intsel |= irqBit;
            return OK;

        case ipac_irqDisable:
            regs->intsel &= ~ irqBit;
            return OK;

        case ipac_irqPoll:
            return regs->ipstat & irqBit;

        default:
            return S_IPAC_notImplemented;
    }
}



/*******************************************************************************

Routine:
    ipacHy8002CarrierInfo

Purpose:
    Print model and serial numbers from carrier card ROM

Description:
    Display model and serial number information from the carrier PROM for a
    selected carrier, or for all registered carriers when the carrier number
    requested is > 20.

Return:
    0 = OK

*/

int ipacHy8002CarrierInfo (
    epicsUInt16 carrier
) {
    private_t *private = list_head;

    if (private == NULL) {
        printf("No Hy8002/8004 carriers registered.\n");
        return OK;
    }

    while (private != NULL) {
        volatile epicsUInt8 *prom = private->prom;

        if ((private->carrier == carrier) || (carrier > 20)) {
            printf("PROM manufacturer ID: 0x%06x.\n",
                (prom[VME64CR_MAN1] << 16) + (prom[VME64CR_MAN2] << 8) +
                 prom[VME64CR_MAN3]);

            printf("PROM model #: 0x%04x, board rev. 0x%02x\n",
                (prom[VME64CR_MOD1] << 8) + prom[VME64CR_MOD2],
                prom[VME64CR_REVN]);

            printf("PROM Xilinx rev.: 0x%02x, 0x%02x, 0x%02x\n",
                prom[VME64CR_XIL1], prom[VME64CR_XIL2], prom[VME64CR_XIL3]);

            printf("PROM Serial #: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
                prom[VME64CR_SER1], prom[VME64CR_SER2], prom[VME64CR_SER3],
                prom[VME64CR_SER4], prom[VME64CR_SER5], prom[VME64CR_SER6]);

            if (private->carrier == carrier)
                break;
        }
        private = private->next;
    }
    return OK;
}


/******************************************************************************/

/* IPAC Carrier Table */

static ipac_carrier_t Hy8002 = {
    "Hytec VICB8002/8004",
    SLOTS,
    initialise,
    report,
    baseAddr,
    irqCmd,
    NULL,
    NULL
};


int ipacAddHy8002(const char *cardParams) {
    return ipacAddCarrier(&Hy8002, cardParams);
}


/* iocsh Command Table and Registrar */

static const iocshArg argParams =
    {"cardParams", iocshArgString};
static const iocshArg argCarrier =
    {"carrier", iocshArgInt};

static const iocshArg * const addArgs[] =
    {&argParams};
static const iocshArg * const infoArgs[] =
    {&argCarrier};

static const iocshFuncDef Hy8002FuncDef =
    {"ipacAddHy8002", NELEMENTS(addArgs), addArgs};

static void Hy8002CallFunc(const iocshArgBuf *args) {
    ipacAddHy8002(args[0].sval);
}

static const iocshFuncDef Hy8002InfoFuncDef =
    {"ipacHy8002CarrierInfo", NELEMENTS(infoArgs), infoArgs};

static void Hy8002InfoCallFunc(const iocshArgBuf *args) {
    ipacHy8002CarrierInfo(args[0].ival);
}

static void epicsShareAPI Hy8002Registrar(void) {
    iocshRegister(&Hy8002FuncDef, Hy8002CallFunc);
    iocshRegister(&Hy8002InfoFuncDef, Hy8002InfoCallFunc);
}

epicsExportRegistrar(Hy8002Registrar);
