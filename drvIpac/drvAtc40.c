/*******************************************************************************

Project:
    LEDA Run Permit system

File:
    drvAtc40.c

Description:
    IPAC Carrier Driver for the SBS/GreenSpring ATC40 Quad IndustryPack
    Carrier ISA board, provides the interface between IPAC driver and the
    hardware.

Original Author:
    Peregrine McGehee (interrupt support by Jeff Hill)

Created:
    24 August 1998

TODO:
allow the user to configure different memory sizes

*******************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <stddef.h>

#include <vxWorks.h>
#include <sysLib.h>
#include <lstLib.h>
#include <rebootLib.h>
#include <intLib.h>
#include <logLib.h>
#include <iv.h>
#include <semLib.h>

#include "drvIpac.h"
#include "epicsExport.h"
#include "iocsh.h"


/*
 * Characteristics of the card
 */
#define SLOTS 4
#define IO_SPACES 4     /* Address spaces in ISA 16MByte */
#define IPAC_IRQS 2     /* Interrupts per module */

typedef volatile struct {
    uint16_t io[0x40];
    uint16_t id[0x40];
} atc40IpacIOID;

/*
 * It appears that the doc for the atc 40 has the vectors for int0 and int1
 * reversed
 */
typedef volatile struct {
    uint16_t vecInt0;
    uint16_t vecInt1;
    uint8_t reserved[0x7c];
} atc40IpacVecs;

typedef volatile struct {
    uint16_t config;
    uint8_t reserved[0xfe];
} atc40IpacMemConfig;

/*
 * overlay structure for the ATC 40 IPAC carrier default memory map
 */
typedef volatile struct {
    atc40IpacIOID ioid[4];
    atc40IpacVecs vec[4];
    uint8_t reserved0[0x200];
    uint8_t mem[4][0x800];
    uint8_t reserved1[0x1000];
    atc40IpacMemConfig memConfig[4];
    uint16_t intStatus;
    uint8_t reserved2[0xfe];
    uint16_t intEnable;
    uint8_t reserved3[0x2fe];
} atc40Device;

/*
 * Carrier Private structure type, one instance per board
 */
typedef struct {
    NODE node;
    void *baseAddr[IPAC_ADDR_SPACES][SLOTS];
    atc40Device *pDev;      /* ATC40 device address */
    ushort_t carrier;       /* IPAC carrier number */
    unsigned char irq;      /* interrupt request level */
} atc40Config_t;

/*
 * ISA Interrupt levels
 */
#define IRQ_DEFAULT_ATC40 11
#define ISA_N_IRQS 16

/*
 * IPAC vectors
 */
#define IPAC_N_VECTORS 0x100

/*
 * One entry for each of 256 possible interrupt vectors
 */
typedef struct {
    void (*pISR) (int parameter);
    atc40Config_t *pConfig;
    int parameter;
    unsigned useCount;
    unsigned vecNum;
    uchar_t slot;
} atc40IntDispatch_t;

LOCAL atc40IntDispatch_t intDispatchTable[IPAC_N_VECTORS];

typedef struct {
    LIST deviceList;
    unsigned char intConnected;
} configATC40IRQ_t;

LOCAL configATC40IRQ_t irqATC40Table[ISA_N_IRQS];

LOCAL int disableAtc40Ints(int startType);
LOCAL void atc40GlobalISR(int irq);
LOCAL void atc40ISR(atc40Device * pDev);
LOCAL void atc40UnexpectedVecISR(int arg);
LOCAL SEM_ID atc40Lock;
LOCAL int intVecConnectLocked(void *cPrivate, ushort_t slot, ushort_t vecNum,
            void (*routine) (int parameter), int parameter);

/*
 * this is a vxWorks BSP supplied glovbal variable
 */
extern UINT sysVectorIRQ0;

/*
 * can be set from the vxWorks shell
 */
unsigned char debugAtc40InterruptJam;
void (*pGlobalATC40ISR) (int irq) = atc40GlobalISR;

/*******************************************************************************

Routine:
    initialize

Purpose:
    Creates new private table for ATC40 at addresses given by cardParams

Description:
    Checks the parameter string for the address of the card I/O space and
    optional size of the memory space for the modules.  If both the I/O and
    memory base addresses can be reached from the CPU, a private table is
    created for this board.  The private table is a 2-D array of pointers
    to the base addresses of the various accessible parts of the IP module.

Parameters:
    The parameter string should comprise a integer number in scanf %p format
    (with GNU a leading 0x or 0X followed by a hexadecimal integer is required),
    optionally followed by white space and an integer in scanf %i format, optionally 
    followed by white space and another integer in scanf %i format. The first number 
    is the I/O base address of the card in the CPU's address space (the factory default 
    is 0xfc0000 but the software default is 0xa0000).  If present the second number 
    specifies the memory address space consumption in Kbytes allocated to each IP module. 
    If present the third number specifies the ISA interrupt request level that will be 
    used by the card.

    The ATC40 must be installed in a 16-bit ISA bus slot. This allows both
    byte and word access to IPs.

    The ATC40 configuration registers and its four IPs are mapped into
    16 Kbytes within the ISA 16 Mbytes memory space. The 16 Kbyte block is
    selected by a shunt block on the ATC40.

    IP I/O, ID, two Kbytes memory per IP, interrupt and configuration
    registers are mapped within this 16 Kbyte address block. Additional
    IP memory is supported, however the additional memory must be mapped
    above one Mbyte of ISA bus memory space in PC/AT. The additional memory
    sizes supported include 512, 1024, 2048, 4096, and 8192 KBytes.

Examples:
    "0x6000"
    This indicates that the carrier has its I/O base set to 0x6000, and
    the slots use 2 Kbytes memory space each.
    "0X1000,512"
    Here the I/O base is set to hex 1000, and there is 512 Kbytes of
    memory on each module.
    "0x1000,0x200,11"
    Here the I/O base is set to hex 1000, there is 512 Kbytes of
    memory on each module, and the ISA interrupt request level is 11.

Returns:
    0 = OK,
    S_IPAC_badAddress = Parameter string error, or address not reachable

*/

LOCAL int initialize(const char *cardParams, void **pprivate, ushort_t carrier)
{
    static char init = 0;
    int params;
    int status;
    unsigned mSize = 0;
    atc40Device *pDev = NULL;
    ushort_t slot;
    atc40Config_t *pConfig;
    ushort_t vecNum;
    unsigned irq = IRQ_DEFAULT_ATC40;
    int key;

#if 0
    /*
     * verify offsets in device overlay structure
     */
    printf("size %lx\n", sizeof(atc40Device));
    printf("ioid[0] at %x\n", offsetof(atc40Device, ioid[0]));
    printf("ioid[1] at %x\n", offsetof(atc40Device, ioid[1]));
    printf("ioid[2] at %x\n", offsetof(atc40Device, ioid[2]));
    printf("ioid[3] at %x\n", offsetof(atc40Device, ioid[3]));
    printf("vec[0] at %x\n", offsetof(atc40Device, vec[0]));
    printf("vec[1] at %x\n", offsetof(atc40Device, vec[1]));
    printf("vec[2] at %x\n", offsetof(atc40Device, vec[2]));
    printf("vec[3] at %x\n", offsetof(atc40Device, vec[3]));
    printf("intStatus at %x\n", offsetof(atc40Device, intStatus));
    printf("intEnable at %x\n", offsetof(atc40Device, intEnable));
#endif

    /*
     * shut off interrupts when its a soft reboot
     */
    if (!init) {
        /*
         * initialize the global mutex
         */
        atc40Lock = semMCreate(SEM_Q_PRIORITY | SEM_DELETE_SAFE | SEM_INVERSION_SAFE);
        if (atc40Lock == NULL) {
            return errno;
        }
        /*
         * install a reboot hook
         */
        status = rebootHookAdd(disableAtc40Ints);
        if (status < 0) {
            return errno;
        }
        /*
         * initialize the int dispatch table
         */
        for (vecNum = 0; vecNum < NELEMENTS(intDispatchTable); vecNum++) {
            intDispatchTable[vecNum].pISR = atc40UnexpectedVecISR;
            intDispatchTable[vecNum].parameter = vecNum;
            intDispatchTable[vecNum].pConfig = NULL;
            intDispatchTable[vecNum].useCount = 0u;
        }

        init = TRUE;
    }
    if (cardParams == NULL || strlen(cardParams) == 0) {
        /*
         * beginning of the "no memory" portion of the WRS pc486 address map
         */
        pDev = (atc40Device *) 0xa0000;
    } else {
        params = sscanf(cardParams, "%p %i %i", (void **) &pDev, &mSize, &irq);
        /*
         * verify reasonable parameters
         */
        if (params < 1 || params > 3) {
            return S_IPAC_badAddress;
        }
        /*
         * currently only the default memory map is supported
         */
        if (params >= 2 && mSize != sizeof(atc40Device)) {
            return S_IPAC_badAddress;
        }
        if (params >= 3 && (irq >= ISA_N_IRQS || irq < 0)) {
            return S_IPAC_badIntLevel;
        }
    }

    /*
     * Unfortunately, the WRS BSP is mapping all of the ISA bus as valid and
     * in this situation the Intel archictecture does not generate a page
     * fault exception when we access the card and it isnt there. Since this
     * module does not provide an identifier, then it is difficult to probe
     * for its existence.
     */

    /*
     * allocate space for the module private configuration structure
     */
    pConfig = calloc(1, sizeof(*pConfig));
    if (pConfig == NULL) {
        return errno;
    }
    pConfig->pDev = pDev;
    pConfig->carrier = carrier;
    pConfig->irq = (unsigned char) irq;

    /*
     * determine the base addresses of the various IPAC address spaces
     */
    for (slot = 0; slot < SLOTS; slot++) {
        pConfig->baseAddr[ipac_addrID][slot] = (void *) pDev->ioid[slot].id;
    }
    for (slot = 0; slot < SLOTS; slot++) {
        pConfig->baseAddr[ipac_addrIO][slot] = (void *) pDev->ioid[slot].io;
    }
    for (slot = 0; slot < SLOTS; slot++) {
        pConfig->baseAddr[ipac_addrIO32][slot] = NULL;
    }
    for (slot = 0; slot < SLOTS; slot++) {
        pConfig->baseAddr[ipac_addrMem][slot] = (void *) pDev->mem[slot];
    }

    /*
     * initially disable ints
     */
    pDev->intEnable = 0;

    /*
     * install into the reboot int shutdown list
     */
    status = semTake(atc40Lock, 5 * sysClkRateGet());
    if (status != OK) {
        free(pConfig);
        return errno;
    }
    /*
     * must also lock interrupts when adding to or deleteing from this list
     * because it is accessed at interrupt level
     */
    key = intLock();
    lstAdd(&irqATC40Table[irq].deviceList, &pConfig->node);
    intUnlock(key);

    status = semGive(atc40Lock);
    assert(status == OK);

    *pprivate = pConfig;
    return OK;
}

/*
 * disableAtc40Ints ()
 */
LOCAL int disableAtc40Ints(int startType)
{
    atc40Config_t *pConfig;
    int status;
    unsigned irq;

    if (atc40Lock == NULL) {
        return OK;
    }
    /*
     * install into the reboot int shutdown list
     */
    status = semTake(atc40Lock, 5 * sysClkRateGet());
    if (status != OK) {
    return errno;
    }
    for (irq = 0; irq < NELEMENTS(irqATC40Table); irq++) {
        for (pConfig = (atc40Config_t *) lstFirst(&irqATC40Table[irq].deviceList);
             pConfig; 
             pConfig = (atc40Config_t *) lstNext(&pConfig->node)) {
            /*
             * disable interrupts
             */
            pConfig->pDev->intEnable = 0;
            printf("Disabled interrupts on ATC 40 carrier %u\n", pConfig->carrier);
        }
    }

    status = semGive(atc40Lock);
    assert(status == OK);

    return OK;
}

/*******************************************************************************

Routine:
    baseAddr

Purpose:
    Returns the base address for the requested slot & address space

Description:
    Because we did all that hard work in the initialize routine, this
    routine only has to do a table lookup in the private array.
    Note that no parameter checking is required - the IPAC driver which
    calls this routine handles that.

Returns:
    The requested address, or NULL if the module has no memory.

*/

LOCAL void *baseAddr(void *private, ushort_t slot, ipac_addr_t space)
{
    atc40Config_t *pConfig = (atc40Config_t *) private;
    return pConfig->baseAddr[space][slot];
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
    ISAbus interrupter is listening on the necessary level.

Returns:
    ipac_irqGetLevel returns the interrupt level.
    ipac_irqEnable returns 0 = OK,
    other calls return S_IPAC_notImplemented.

*/

LOCAL int irqCmd(void *private, ushort_t slot, ushort_t irqNumber,
         ipac_irqCmd_t cmd)
{
    atc40Config_t *pConfig = (atc40Config_t *) private;

    switch (cmd) {
    case ipac_irqGetLevel:
        return pConfig->irq;

    case ipac_irqEnable:
        /*
         * enable the interrrupt level in the PIC chip
         */
        sysIntEnablePIC(pConfig->irq);

        /*
         * Allow the ATC-40 to pass interrupts to the ISA bus.
         */
        pConfig->pDev->intEnable = 1;

        return OK;

    default:
        return S_IPAC_notImplemented;
    }
}

/*******************************************************************************

Routine:
    ipmIntVecConnect

Function:
    Connect module driver to interrupt vector

Description:
    Since the interrupt vectoring mechanisms vary between bus types this
    routines allows a module driver to connect its routine to an interrupt
    vector from a particular IPAC module without knowing the specifics of the
    IP carrier or the bus type. In some situations the carrier driver will need
    to maintain a private interrupt dispatch table if the bus type (i.e. ISA)
    does not support interrupt vectoring.

Returns:
    0 = OK,
    S_IPAC_badAddress = illegal carrier, slot or vector

*/

LOCAL int intVecConnect(void *cPrivate, ushort_t slot, ushort_t vecNum,
            void (*routine) (int parameter), int parameter)
{
    int status;
    int vxStatus;

    /*
     * lock out other tasks
     */
    vxStatus = semTake(atc40Lock, 5 * sysClkRateGet());
    if (vxStatus != OK) {
        return errno;
    }
    status = intVecConnectLocked(cPrivate, slot, vecNum, routine, parameter);

    /*
     * unlock
     */
    vxStatus = semGive(atc40Lock);
    assert(vxStatus == OK);

    return status;
}

/*
 * intVecConnectLocked ()
 * 
 * global int disable is used instead of a device int disable because several
 * ATC40 devices might be using the same int dispatch table
 */
LOCAL int intVecConnectLocked(void *cPrivate, ushort_t slot, ushort_t vecNum,
                  void (*routine) (int parameter), int parameter)
{
    atc40Config_t *pConfig = (atc40Config_t *) cPrivate;
    int key;

    if (vecNum > NELEMENTS(intDispatchTable)) {
        return S_IPAC_badVector;
    }
    if (intDispatchTable[vecNum].pConfig == NULL) {
        if (routine == NULL) {
            /* disconnect request with nothing connected */
            return 0;
        }
    } else {
        /*
         * return an error if they attempt to install on top of some other
         * module's vector
         */
        if ((intDispatchTable[vecNum].pConfig != pConfig) ||
            (intDispatchTable[vecNum].slot != slot)) {
            logMsg("IPAC interrupt vector %#x in use by carrier %u slot %u,\n",
               vecNum, intDispatchTable[vecNum].pConfig->carrier, 
               intDispatchTable[vecNum].slot, 0, 0, 0);
            logMsg("can't reallocate vector to carrier %u slot %u.\n",
               pConfig->carrier, slot, 0, 0, 0, 0);
            return S_IPAC_vectorInUse;
        }
        if (routine == NULL) {
            /* disconnect request */
            key = intLock();
            intDispatchTable[vecNum].pISR = atc40UnexpectedVecISR;
            intDispatchTable[vecNum].parameter = vecNum;
            intDispatchTable[vecNum].pConfig = NULL;
            intDispatchTable[vecNum].useCount = 0u;
            intUnlock(key);
            return 0;
        }
    }

    /*
     * valid connect request
     */
    key = intLock();
    intDispatchTable[vecNum].pISR = routine;
    intDispatchTable[vecNum].parameter = parameter;
    intDispatchTable[vecNum].pConfig = pConfig;
    intDispatchTable[vecNum].slot = slot;
    intDispatchTable[vecNum].useCount = 0u;
    intUnlock(key);

    /*
     * if not connected, then connect to the interrupt
     */
    if (!irqATC40Table[pConfig->irq].intConnected) {
        int status;
        /*
         * attach to the IRQ level if its the first ATC40 device installed
         */
        status = intConnect(INUM_TO_IVEC(sysVectorIRQ0 + pConfig->irq), pGlobalATC40ISR, pConfig->irq);
        if (status == OK) {
            irqATC40Table[pConfig->irq].intConnected = TRUE;
        } else {
            logMsg("Unable to connect to ISA interrupt level %u\n", pConfig->irq,
               0, 0, 0, 0, 0);
        }
    }
    return 0;
}

/*
 * Routine: atc40GlobalISR
 * 
 * Function: global interrupt service routine for all ATC 40 carriers
 * 
 * Description: calls atc40ISR for each atc 40 carrier that has been installed
 * 
 * Returns: void
 */
LOCAL void atc40GlobalISR(int irq)
{
    atc40Config_t *pConfig;

    /*
     * call atc40 ISR for each carrier card that has been installed
     * 
     * interrupts are locked when adding to, or removing from, this list at task
     * level
     */
    for (pConfig = (atc40Config_t *) lstFirst(&irqATC40Table[irq].deviceList);
        pConfig; 
        pConfig = (atc40Config_t *) lstNext(&pConfig->node)) {
        /*
         * dispatch interrupts
         */
        atc40ISR(pConfig->pDev);
    }
}

/*
 * Routine: atc40ISR
 * 
 * Function: interrupt service routine for one ATC 40 carrier
 * 
 * Description: fetches vector and calls the module supplied routine if it
 * exists and prints an error message if not
 * 
 * Returns: void
 */
LOCAL void atc40ISR(atc40Device * pDev)
{
    atc40IntDispatch_t *pEntry;
    unsigned intStatus;
    unsigned slot;
    unsigned bit;

    /*
     * if interrupts are not enabled then NOOP
     */
    if (!pDev->intEnable) {
        return;
    }
    /*
     * disable ints
     */
    pDev->intEnable = 0;

    intStatus = pDev->intStatus;

#ifdef DEBUG
    logMsg("atc 40 int status=%x\n", intStatus, 0, 0, 0, 0, 0);
#endif

    /*
     * check to see which interrupts are active
     */
    for (slot = 0, bit = 1; slot < NELEMENTS(pDev->vec); slot++) {
        if ((intStatus & bit) == 0) {
            pEntry = &intDispatchTable[pDev->vec[slot].vecInt0 & 0xff];
            (*pEntry->pISR) (pEntry->parameter);
            pEntry->useCount++;
        }
        bit <<= 1;
        if ((intStatus & bit) == 0) {
            pEntry = &intDispatchTable[pDev->vec[slot].vecInt1 & 0xff];
            (*pEntry->pISR) (pEntry->parameter);
            pEntry->useCount++;
        }
        bit <<= 1;
    }

    /*
     * DEBUG aid --------- code here is only executed if
     * "debugAtc40InterruptJam" is set TRUE
     */
    if (debugAtc40InterruptJam) {
        /*
         * check for an interrupt that has not been cleared. note that the
         * interrupt may occasionally go active again after it was cleared
         * during an ISR in a normal situation
         */
        intStatus = pDev->intStatus;
        if (intStatus) {
            logMsg("atc40ISR carrier device at %p may have a jammed interrupt\n",
               (int) pDev, 0, 0, 0, 0, 0);
            /*
             * check to see which interrupts are active
             */
            for (slot = 0, bit = 1; slot < 2 * NELEMENTS(pDev->vec); slot++) {
                if ((intStatus & bit) == 0) {
                    logMsg("atc40ISR: interrupt jam on slot %u irq 0\n",
                       slot, 0, 0, 0, 0, 0);
                }
                bit <<= 1;
                if ((intStatus & bit) == 0) {
                    logMsg("atc40ISR: interrupt jam on slot %u irq 1\n",
                       slot, 0, 0, 0, 0, 0);
                }
                bit <<= 1;
            }
            logMsg("atc40ISR interrups were disabled on carrier with jammed interrupt\n",
               0, 0, 0, 0, 0, 0);
            return;
        }
    }
    /*
     * reenable ints
     * 
     * toggle edge trigger if ints are still pending
     */
    pDev->intEnable = 1;
}

/*
 * quiet unexpected interrupt vector handler
 */
LOCAL void atc40QuietUnexpectedVecISR(int vecNum)
{
}

/*
 * unexpected interrupt vector handler
 */
LOCAL void atc40UnexpectedVecISR(int vecNum)
{
    logMsg("Unexpect interrupt on vector = %#x from an ATC40? First and only warning.\n",
            vecNum, 0, 0, 0, 0, 0);
    intDispatchTable[vecNum].pISR = atc40QuietUnexpectedVecISR;
}

/*
 * reportInterruptStatus ()
 */
LOCAL char *reportInterruptStatus (char *pReport, unsigned vecNum, unsigned level, unsigned active)
{
    unsigned nChar;

    nChar = sprintf (pReport, " vec%u=%x ISR=%p(%d) use=%u", 
        level, vecNum, 
        intDispatchTable[vecNum].pISR, 
        intDispatchTable[vecNum].parameter, 
        intDispatchTable[vecNum].useCount);
    pReport = &pReport[nChar];
    if ( active ) {
        nChar = sprintf (pReport, "%c ", '!');
        pReport = &pReport[nChar];
    }
    return pReport;
}


/*
 * IO report routine
 */
char *report(void *cPrivate, ushort_t slot)
{
    atc40Config_t *pConfig = (atc40Config_t *) cPrivate;
    unsigned intStatus;
    unsigned bit;
    unsigned vecNum;
    static char report[256];
    char *pReport = report;

    intStatus = pConfig->pDev->intStatus;

    bit = 1 << (slot * 2);
    vecNum = pConfig->pDev->vec[slot].vecInt0 & 0xff;
    pReport = reportInterruptStatus (pReport, vecNum, 0u, (intStatus & bit) == 0);

    bit <<= 1;
    vecNum = pConfig->pDev->vec[slot].vecInt1 & 0xff;
    pReport = reportInterruptStatus (pReport, vecNum, 1u, (intStatus & bit) == 0);

    return report;
}

/******************************************************************************/


/* IPAC Carrier Table */

static ipac_carrier_t atc40 = {
    "SBS/GreenSpring ATC40",
    SLOTS,
    initialize,
    report,
    baseAddr,
    irqCmd,
    intVecConnect
};

int ipacAddATC40(const char *cardParams) {
    return ipacAddCarrier(&atc40, cardParams);
}


/* iocsh command table and registrar */

static const iocshArg atcArg0 = { "cardParams",iocshArgString};
static const iocshArg * const atcArgs[1] = {&atcArg0};
static const iocshFuncDef atcFuncDef = {"ipacAddATC40", 1, atcArgs};
static void atcCallFunc(const iocshArgBuf *args) {
    ipacAddATC40(args[0].sval);
}

static void epicsShareAPI atc40Registrar(void) {
    iocshRegister(&atcFuncDef, atcCallFunc);
}

epicsExportRegistrar(atc40Registrar);
