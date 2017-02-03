/**************************************************************************
 Header:        tyGSOctal.c

 Author:        Peregrine M. McGehee

 Description:   Sourcefile for SBS/GreenSpring Ip_Octal 232, 422, and 485
 serial I/O modules. This software was somewhat based on the HiDEOS
 device driver developed by Jim Kowalkowski of the Advanced Photon Source.
**************************************************************************

 USER-CALLABLE ROUTINES
 Most of the routines in this driver are accessible only through the I/O
 system.  Some routines, however, must be called directly: tyGSOctalDrv() to
 initialize the driver, tyGSOctalModuleInit() to register modules, and
 tyGSOctalDevCreate() or tyGSOctalDevCreateAll() to create devices.

 Before the driver can be used, it must be initialized by calling
 tyGSOctalDrv().
 This routine should be called exactly once, before any other routines.

 Each IP module must be registered with the driver before use by calling
 tyGSOctalModuleInit().

 Before a terminal can be used, it must be created using
 tyGSOctalDevCreate() or tyGSOctalDevCreateAll().
 Each port to be used must have exactly one device associated with it by
 calling either of the above routines.

 IOCTL FUNCTIONS
 This driver responds to the same ioctl() codes as a normal sio driver; for
 more information, see the manual entry for tyLib and the BSP documentation
 for sioLib.

 SEE ALSO
 tyLib, sioLib

 History:
 who  when      what
 ---  --------  ------------------------------------------------
 PMM  18/11/96  Original
 PMM  13/10/97  Recast as VxWorks device driver.
 ANJ  09/03/99  Merged into ipac <supporttop>, fixed warnings.
 BWK  29/08/00  Added rebootHook routine
 ANJ  11/11/03  Significant cleanup, added ioc shell stuff
**************************************************************************/

/*
 * vxWorks includes
 */
/* This is needed for vxWorks 6.x to prevent an obnoxious compiler warning */
#define _VSB_CONFIG_FILE <../lib/h/config/vsbConfig.h>

#include <vxWorks.h>
#include <iv.h>
#include <rebootLib.h>
#include <intLib.h>
#include <errnoLib.h>
#include <sysLib.h>
#include <tickLib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <logLib.h>
#include <taskLib.h>
#include <tyLib.h>
#include <sioLib.h>
#include <vxLib.h>
#include <epicsTypes.h>
#include <epicsString.h>

#include "ip_modules.h"     /* GreenSpring IP modules */
#include "scc2698.h"        /* SCC 2698 UART register map */
#include "tyGSOctal.h"      /* Device driver includes */
#include "drvIpac.h"        /* IP management (from drvIpac) */
#include "iocsh.h"
#include "epicsExport.h"

/* VxWorks 6.9 changed tyDevinit()'s txStartup argument type.
 * This avoids compiler warnings for that call.
 */
#if !defined(_WRS_VXWORKS_MAJOR) || \
    (_WRS_VXWORKS_MAJOR == 6) && (_WRS_VXWORKS_MINOR < 9)
#define TY_DEVSTART_PTR FUNCPTR
#endif

#if __STDC_VERSION__ < 199901L
#  define fn_nm __FUNCTION__
#else
#  define fn_nm __func__
#endif


LOCAL QUAD_TABLE *tyGSOctalModules;
LOCAL int tyGSOctalMaxModules;
int tyGSOctalLastModule;

LOCAL int tyGSOctalDrvNum;  /* driver number assigned to this driver */

/*
 * forward declarations
 */
void         tyGSOctalInt(int);
LOCAL void   tyGSOctalInitChannel(QUAD_TABLE *, int);
LOCAL int    tyGSOctalRebootHook(int);
LOCAL QUAD_TABLE * tyGSOctalFindQT(const char *);
LOCAL int    tyGSOctalOpen(TY_GSOCTAL_DEV *, const char *, int);
LOCAL int    tyGSOctalWrite(TY_GSOCTAL_DEV *, char *, long);
LOCAL STATUS tyGSOctalIoctl(TY_GSOCTAL_DEV *, int, int);
LOCAL void   tyGSOctalStartup(TY_GSOCTAL_DEV *);
LOCAL STATUS tyGSOctalBaudSet(TY_GSOCTAL_DEV *, int);
LOCAL void   tyGSOctalOptsSet(TY_GSOCTAL_DEV *, int);
LOCAL void   tyGSOctalSetmr(TY_GSOCTAL_DEV *, int, int);

/******************************************************************************
 *
 * tyGSOctalDrv - initialize the tty driver
 *
 * This routine initializes the serial driver, sets up interrupt vectors, and
 * performs hardware initialization of the serial ports.
 *
 * This routine should be called exactly once, before any reads, writes, or
 * calls to tyGSOctalDevCreate().
 *
 * This routine takes as an argument the maximum number of IP modules
 * to support.
 * For example:
 * .CS
 *    int status;
 *    status = tyGSOctalDrv(4);
 * .CE
 *
 * RETURNS: OK, or ERROR if the driver cannot be installed.
 *
 * SEE ALSO: tyGSOctalDevCreate()
*/
STATUS tyGSOctalDrv
    (
    int maxModules
    )
{
    /* check if driver already installed */
    if (tyGSOctalDrvNum > 0)
        return OK;

    tyGSOctalMaxModules = maxModules;
    tyGSOctalLastModule = 0;
    tyGSOctalModules = (QUAD_TABLE *)calloc(maxModules, sizeof(QUAD_TABLE));

    if (!tyGSOctalModules) {
        printf("%s: Memory allocation failed!", fn_nm);
        return ERROR;
    }
    rebootHookAdd(tyGSOctalRebootHook);

    tyGSOctalDrvNum = iosDrvInstall(tyGSOctalOpen, NULL, tyGSOctalOpen, NULL,
        tyRead, tyGSOctalWrite, tyGSOctalIoctl);

    return tyGSOctalDrvNum == ERROR ? ERROR : OK;
}

void tyGSOctalReport(void)
{
    int mod;

    for (mod = 0; mod < tyGSOctalLastModule; mod++) {
        QUAD_TABLE *qt = &tyGSOctalModules[mod];
        int port;

        printf("Module %d: carrier=%d slot=%d\n  %lu interrupts\n",
            mod, qt->carrier, qt->slot, qt->interruptCount);

        for (port=0; port < 8; port++) {
            TY_GSOCTAL_DEV *dev = &qt->dev[port];

            if (dev->created)
                printf("  Port %d: %lu chars in, %lu chars out, %lu errors\n",
                    port, dev->readCount, dev->writeCount, dev->errorCount);
        }
    }
}

LOCAL int tyGSOctalRebootHook(int type)
{
    int mod;
    int key = intLock();    /* disable interrupts */

    for (mod = 0; mod < tyGSOctalLastModule; mod++) {
        QUAD_TABLE *qt = &tyGSOctalModules[mod];
        int port;

        for (port=0; port < 8; port++) {
            TY_GSOCTAL_DEV *dev = &qt->dev[port];

            if (dev->created) {
                dev->irqEnable = 0; /* prevent re-enabling */
                dev->regs->u.w.imr = 0;
            }
            ipmIrqCmd(qt->carrier, qt->slot, 0, ipac_irqDisable);
            ipmIrqCmd(qt->carrier, qt->slot, 1, ipac_irqDisable);

            ipmIrqCmd(qt->carrier, qt->slot, 0, ipac_statUnused);
        }
    }
    intUnlock(key);
    return OK;
}

/******************************************************************************
 * tyGSOctalModuleInit - initialize an IP module
 *
 * The routine initializes the specified IP module. Each module is
 * characterized by its model name, interrupt vector, carrier board
 * number, and slot number on the board. No new setup is done if a
 * QUAD_TABLE entry already exists with the same carrier and slot
 * numbers.
 *
 * For example:
 * .CS
 *    int idx;
 *    idx = tyGSOctalModuleInit("SBS232-1", "232", 0x60, 0, 1);
 * .CE
 *
 *
 * RETURNS: Index into module table, or ERROR if the driver is not
 * installed, the channel is invalid, or the device already exists.
 *
 * SEE ALSO: tyGSOctalDrv()
*/
int tyGSOctalModuleInit
    (
    const char * moduleID,       /* IP module name */
    const char * type,           /* IP module type 232/422/485 */
    int          int_num,        /* Interrupt vector */
    int          carrier,        /* carrier number */
    int          slot            /* slot number */
    )
{
    int modelID;
    int status;
    int mod;
    QUAD_TABLE *qt;

    /*
     * Check for the driver being installed.
     */
    if (tyGSOctalDrvNum <= 0) {
        errnoSet(S_ioLib_NO_DRIVER);
        return ERROR;
    }

    if (!moduleID || !type) {
        errnoSet(EINVAL);
        return ERROR;
    }

    /*
     * Check the IP module type.
     */
    if (strstr(type, "232"))
        modelID = GSIP_OCTAL232;
    else if (strstr(type, "422"))
        modelID = GSIP_OCTAL422;
    else if (strstr(type, "485"))
        modelID = GSIP_OCTAL485;
    else {
        printf("%s: Unsupported module type: %s", fn_nm, type);
        errnoSet(EINVAL);
        return ERROR;
    }

    /*
     * Validate the IP module location and type.
     */
    status = ipmValidate(carrier, slot, GREEN_SPRING_ID, modelID);
    if (status) {
        printf("%s: IPAC Module validation failed\n"
            "    Carrier:%d slot:%d modelID:0x%x\n",
            fn_nm, carrier, slot, modelID);

        switch(status) {
        case S_IPAC_badAddress:
            printf("    Bad carrier or slot number\n");
            break;
        case S_IPAC_noModule:
            printf("    No module installed\n");
            break;
        case S_IPAC_noIpacId:
            printf("    IPAC identifier not found\n");
            break;
        case S_IPAC_badCRC:
            printf("    CRC Check failed\n");
            break;
        case S_IPAC_badModule:
            printf("    Manufacturer or model IDs wrong\n");
            break;
        default:
            printf("    Unknown status code: 0x%x\n", status);
            break;
        }
        errnoSet(status);
        return ERROR;
    }

    /* See if the associated IP module has already been set up */
    for (mod = 0; mod < tyGSOctalLastModule; mod++) {
        qt = &tyGSOctalModules[mod];
        if (qt->carrier == carrier &&
            qt->slot == slot)
            break;
    }

    /* Create a new quad table entry if not there */
    if (mod >= tyGSOctalLastModule) {
        void *addrIO;
        char *addrMem;
        char *ID = epicsStrDup(moduleID);
        uint16_t intNum = int_num;
        SCC2698 *r;
        SCC2698_CHAN *c;
        int block, port;

        if (tyGSOctalLastModule >= tyGSOctalMaxModules) {
            printf("%s: Maximum module count exceeded!", fn_nm);
            errnoSet(ENOSPC);
            return ERROR;
        }

        qt = &tyGSOctalModules[tyGSOctalLastModule];
        qt->modelID = modelID;
        qt->carrier = carrier;
        qt->slot = slot;
        qt->moduleID = ID;

        addrIO = ipmBaseAddr(carrier, slot, ipac_addrIO);
        r = (SCC2698 *) addrIO;
        c = (SCC2698_CHAN *) addrIO;

        for (port = 0; port < 8; port++) {
            block = port/2;
            qt->dev[port].created = 0;
            qt->dev[port].qt = qt;
            qt->dev[port].regs = &r[block];
            qt->dev[port].chan = &c[port];
        }

        for (block = 0; block < 4; block++)
            qt->imr[block] = 0;

        /* set up the single interrupt vector */
        addrMem = (char *) ipmBaseAddr(carrier, slot, ipac_addrMem);
        if (addrMem == NULL) {
            printf("%s: No IPAC memory allocated for carrier %d slot %d",
                fn_nm, carrier, slot);
            return ERROR;
        }
        if (vxMemProbe(addrMem, VX_WRITE, 2, (char *) &intNum) == ERROR) {
            printf("%s: Bus Error writing interrupt vector to address 0x%p",
                fn_nm, addrMem);
            return ERROR;
        }

        if (ipmIntConnect(carrier, slot, int_num,
                          tyGSOctalInt, tyGSOctalLastModule)) {
            printf("%s: Unable to connect ISR", fn_nm);
            return ERROR;
        }
        ipmIrqCmd(carrier, slot, 0, ipac_irqEnable);
        ipmIrqCmd(carrier, slot, 1, ipac_irqEnable);

        ipmIrqCmd(carrier, slot, 0, ipac_statActive);
    }

    return tyGSOctalLastModule++;
}

/******************************************************************************
 * tyGSOctalDevCreate - create a device for a serial port on an IP module
 *
 * This routine creates a device on a specified serial port.  Each port
 * to be used should have exactly one device associated with it by calling
 * this routine.
 *
 * For instance, to create the device "/SBS/0,1/3", with buffer sizes
 * of 512 bytes, the proper calls would be:
 * .CS
 *    if (tyGSOctalModuleInit("232-1", "232", 0x60, 0, 1) != ERROR) {
 *       char *nam = tyGSOctalDevCreate ("/SBS/0,1/3", "232-1", 3, 512, 512);
 * }
 * .CE
 *
 * RETURNS: Pointer to device name, or NULL if the driver is not
 * installed, the channel is invalid, or the device already exists.
 *
 * SEE ALSO: tyGSOctalDrv()
*/
const char * tyGSOctalDevCreate
    (
    char *       name,           /* name to use for this device          */
    const char * moduleID,       /* IP module name                       */
    int          port,           /* port on module for this device [0-7] */
    int          rdBufSize,      /* read buffer size, in bytes           */
    int          wrtBufSize      /* write buffer size, in bytes          */
    )
{
    TY_GSOCTAL_DEV *dev;
    QUAD_TABLE *qt = tyGSOctalFindQT(moduleID);

    if (!name || !qt)
        return NULL;

    /* if this doesn't represent a valid port, don't do it */
    if (port < 0 || port > 7)
        return NULL;

    dev = &qt->dev[port];

    /* if there is a device already on this channel, don't do it */
    if (dev->created)
        return NULL;

    /* initialize the ty descriptor */
    if (tyDevInit (&dev->tyDev, rdBufSize, wrtBufSize,
                   (TY_DEVSTART_PTR) tyGSOctalStartup) != OK)
        return NULL;

    /* initialize the channel hardware */
    tyGSOctalInitChannel(qt, port);

    /* mark the device as created, and add the device to the I/O system */
    dev->created = TRUE;

    if (iosDevAdd(&dev->tyDev.devHdr, name,
                  tyGSOctalDrvNum) != OK)
        return NULL;

    return name;
}

/******************************************************************************
 * tyGSOctalDevCreateAll - create devices for all ports on a module
 *
 * This routine creates up to 8 devices, one for each port that has not
 * already been created.  Use this after calling tyGSOctalDevCreate to
 * set up any ports that should not use the standard configuration.
 * The port names are constructed by appending the digits 0 through 7 to
 * the base name string given in the first argument.
 *
 * For instance, to create devices "/tyGS/0/0" through "/tyGS/0/7", with
 * buffer sizes of 512 bytes, the proper calls would be:
 * .CS
 *    if (tyGSOctalModuleInit("232-1", "232", 0x60, 0, 1) != ERROR) {
 *       tyGSOctalDevCreateAll ("/tyGS/0/", "232-1", 512, 512);
 * }
 * .CE
 *
 * RETURNS: OK, or ERROR if the driver is not installed, or any device
 * cannot be initialized.
 *
 * SEE ALSO: tyGSOctalDrv(), tyGSOctalDevCreate()
 */
STATUS tyGSOctalDevCreateAll
    (
    const char * base,           /* base name for these devices      */
    const char * moduleID,       /* module identifier from the
                                 * call to tyGSOctalModuleInit(). */
    int          rdBufSize,      /* read buffer size, in bytes       */
    int          wrtBufSize      /* write buffer size, in bytes      */
    )
{
    QUAD_TABLE *qt = tyGSOctalFindQT(moduleID);
    int port;

    if (!qt || !base) {
        errnoSet(EINVAL);
        return ERROR;
    }

    for (port=0; port < 8; port++) {
        TY_GSOCTAL_DEV *dev = &qt->dev[port];
        char name[256];

        /* if there is a device already on this channel, ignore it */
        if (dev->created)
            continue;

        /* initialize the ty descriptor */
        if (tyDevInit(&dev->tyDev, rdBufSize, wrtBufSize,
                (TY_DEVSTART_PTR) tyGSOctalStartup) != OK)
            return ERROR;

        /* initialize the channel hardware */
        tyGSOctalInitChannel(qt, port);

        /* mark the device as created, and give it to the I/O system */
        dev->created = TRUE;

        sprintf(name, "%s%d", base, port);

        if (iosDevAdd(&dev->tyDev.devHdr, name, tyGSOctalDrvNum) != OK)
            return ERROR;
    }
    return OK;
}


/******************************************************************************
 *
 * tyGSOctalFindQT - Find a named module quadtable
 *
 * NOMANUAL
 */
LOCAL QUAD_TABLE * tyGSOctalFindQT
    (
    const char *moduleID
    )
{
    int mod;

    if (!moduleID)
        return NULL;

    for (mod = 0; mod < tyGSOctalLastModule; mod++)
        if (strcmp(moduleID, tyGSOctalModules[mod].moduleID) == 0)
            return &tyGSOctalModules[mod];

    return NULL;
}

/******************************************************************************
 *
 * tyGSOctalInitChannel - initialize a single channel
 *
 * NOMANUAL
 */
LOCAL void tyGSOctalInitChannel
    (
        QUAD_TABLE *qt,
        int port
    )
{
    TY_GSOCTAL_DEV *dev = &qt->dev[port];
    int block = port/2;     /* 4 blocks per octal UART */
    int key;                /* current interrupt level mask */

    key = intLock ();       /* disable interrupts during init */

    dev->block = block;

    dev->irqEnable = ((port%2 == 0) ? SCC_ISR_TXRDY_A : SCC_ISR_TXRDY_B);

    /* choose set 2 BRG */
    dev->regs->u.w.acr = 0x80;

    dev->chan->u.w.cr = 0x1a; /* disable trans/recv, reset pointer */
    dev->chan->u.w.cr = 0x20; /* reset recv */
    dev->chan->u.w.cr = 0x30; /* reset trans */
    dev->chan->u.w.cr = 0x40; /* reset error status */

/*
 * Set up the default port configuration:
 * 9600 baud, no parity, 1 stop bit, 8 bits per char, no flow control
 */
    tyGSOctalBaudSet(dev, 9600);
    tyGSOctalOptsSet(dev, CS8 | CLOCAL);

/*
 * enable everything, really only Rx interrupts
*/
    qt->imr[block] |= ((port%2) == 0 ? SCC_ISR_RXRDY_A : SCC_ISR_RXRDY_B);

    dev->regs->u.w.imr = qt->imr[block]; /* enable RxRDY interrupt */
    dev->chan->u.w.cr = 0x05;            /* enable Tx,Rx */

    intUnlock (key);
}

/******************************************************************************
 *
 * tyGSOctalOpen - open file to UART
 *
 * NOMANUAL
 */
LOCAL int tyGSOctalOpen
    (
        TY_GSOCTAL_DEV *dev,
        const char * name,
        int          mode
    )
{
    return (int) dev;
}


/******************************************************************************
 * tyGSOctalWrite - Outputs a specified number of characters on a serial port
 *
 * NOMANUAL
 */
LOCAL int tyGSOctalWrite
    (
        TY_GSOCTAL_DEV *dev,   /* device descriptor block */
        char *write_bfr,                /* ptr to an output buffer */
        long write_size                 /* # bytes to write */
    )
{
    SCC2698_CHAN *chan = dev->chan;
    int nbytes;

    /*
     * verify that the device descriptor is valid
     */
    if (!dev) {
        logMsg("%s: NULL device descriptor from %s\n",
                (int)fn_nm, (int)taskName(taskIdSelf()), 3,4,5,6);
        return -1;
    }

    if (dev->mode == RS485)
        /* disable recv, 1000=assert RTSN (low) */
        chan->u.w.cr = 0x82;

    nbytes = tyWrite(&dev->tyDev, write_bfr, write_size);

    if (dev->mode == RS485) {
        /* make sure all data sent */
        while(!(chan->u.r.sr & 0x08))   /* Wait for TxEMT */
            ;
        /* enable recv, 1001=negate RTSN (high) */
        chan->u.w.cr = 0x91;
    }

    return nbytes;
}

/******************************************************************************
 *
 * tyGSOctalSetmr - set mode registers
 *
 * NOMANUAL
 */

LOCAL void tyGSOctalSetmr(TY_GSOCTAL_DEV *dev, int mr1, int mr2) {
    SCC2698_CHAN *chan = dev->chan;
    SCC2698 *regs = dev->regs;
    QUAD_TABLE *qt = dev->qt;

    if (qt->modelID == GSIP_OCTAL485) {
        dev->mode = RS485;

        /* MPOa/b are Tx output enables, must be controlled by driver */
        mr1 &= 0x7f; /* no auto RxRTS */
        mr2 &= 0xcf; /* no CTS enable Tx */
    }
    else {
        dev->mode = RS232;
        /* MPOa/b are RTS outputs, may be controlled by UART */
    }
    regs->u.w.opcr = 0x80; /* MPPn = output, MPOa/b = RTSN */
    chan->u.w.cr = 0x10; /* point MR to MR1 */
    chan->u.w.mr = mr1;
    chan->u.w.mr = mr2;

    if (mr1 & 0x80) { /* Hardware flow control */
        chan->u.w.cr = 0x80;    /* Assert RTSN */
    }
}

/******************************************************************************
 *
 * tyGSOctalOptsSet - set channel serial options
 *
 * NOMANUAL
 */

LOCAL void tyGSOctalOptsSet(TY_GSOCTAL_DEV *dev, int opts)
{
    epicsUInt8 mr1 = 0, mr2 = 0;

    switch (opts & CSIZE) {
    case CS5:
        break;
    case CS6:
        mr1 |= 0x01;
        break;
    case CS7:
        mr1 |= 0x02;
        break;
    case CS8:
    default:
        mr1 |= 0x03;
        break;
    }

    if (opts & STOPB)
        mr2 |= 0x0f;
    else
        mr2 |= 0x07;

    if (!(opts & PARENB))
        mr1 |= 0x10;

    if (opts & PARODD)
        mr1 |= 0x04;

    if (!(opts & CLOCAL)) {
        mr1 |= 0x80;      /* Control RTS from RxFIFO */
        mr2 |= 0x10;      /* Enable Tx using CTS */
    }

    tyGSOctalSetmr(dev, mr1, mr2);
    dev->opts = opts & (CSIZE | STOPB | PARENB | PARODD | CLOCAL);
}

/******************************************************************************
 *
 * tyGSOctalBaudSet - set channel baud rate
 *
 * NOMANUAL
 */

LOCAL STATUS tyGSOctalBaudSet(TY_GSOCTAL_DEV *dev, int baud)
{
    SCC2698_CHAN *chan = dev->chan;

    switch(baud) {  /* NB: ACR[7]=1 */
    case 1200:
        chan->u.w.csr=0x66;
        break;
    case 2400:
        chan->u.w.csr=0x88;
        break;
    case 4800:
        chan->u.w.csr=0x99;
        break;
    case 9600:
        chan->u.w.csr=0xbb;
        break;
    case 19200:
        chan->u.w.csr=0xcc;
        break;
    case 38400:
        chan->u.w.csr=0x22;
        break;
    default:
        errnoSet(EINVAL);
        return ERROR;
    }

    dev->baud = baud;
    return OK;
}

/******************************************************************************
 *
 * tyGSOctalIoctl - special device control
 *
 * This routine handles FIOBAUDRATE, SIO_BAUD_SET and SIO_HW_OPTS_SET
 * requests and passes all others to tyIoctl().
 *
 * RETURNS: OK, or ERROR if invalid input.
 */
LOCAL STATUS tyGSOctalIoctl
    (
    TY_GSOCTAL_DEV *dev,   /* device to control */
    int request,                    /* request code */
    int arg                         /* some argument */
    )
{
    STATUS status = 0;
    int key;

    switch (request)
    {
    case FIOBAUDRATE:
    case SIO_BAUD_SET:
        key = intLock ();
        status = tyGSOctalBaudSet(dev, arg);
        intUnlock (key);
        break;
    case SIO_BAUD_GET:
        *(int *)arg = dev->baud;
        break;
    case SIO_HW_OPTS_SET:
        key = intLock ();
        tyGSOctalOptsSet(dev, arg);
        intUnlock (key);
        break;
    case SIO_HW_OPTS_GET:
        *(int *)arg = dev->opts;
        break;
    default:
        status = tyIoctl (&dev->tyDev, request, arg);
        break;
    }

    return status;
}

/******************************************************************************
 *
 * tyGSOctalConfig - special device control (old version)
 *
 * This routine sets the baud rate, parity, stop bits, word size, and
 * flow control for the specified port.
 *
 */
STATUS tyGSOctalConfig (
    char *name,
    int baud,
    char parity,
    int stop,
    int bits,
    char flow
) {
    TY_GSOCTAL_DEV *dev = (TY_GSOCTAL_DEV *) iosDevFind(name, NULL);
    int opts = 0;
    int key;

    if (!dev || strcmp(dev->tyDev.devHdr.name, name)) {
        printf("%s: Device %s not found\n", fn_nm, name);
        return ERROR;
    }

    switch (bits) {
    case 5:
        opts |= CS5;
        break;
    case 6:
        opts |= CS6;
        break;
    case 7:
        opts |= CS7;
        break;
    case 8:
    default:
        opts |= CS8;
        break;
    }

    if (stop == 2)
        opts |= STOPB;
    if (tolower(flow) != 'h')
        opts |= CLOCAL;

    if (tolower(parity) == 'e')
        opts |= PARENB;
    else if (tolower(parity) == 'o')
        opts |= PARENB | PARODD;

    key = intLock ();
    tyGSOctalOptsSet(dev, opts);
    tyGSOctalBaudSet(dev, baud);
    intUnlock (key);
    return OK;
}

/*****************************************************************************
 * tyGSOctalInt - interrupt level processing
 *
 * NOMANUAL
 */
void tyGSOctalInt
    (
    int mod
    )
{
    epicsUInt8 sr, isr;
    QUAD_TABLE *qt = &tyGSOctalModules[mod];
    SCC2698 *regs;
    volatile epicsUInt8 *flush = NULL;
    int scan;

    qt->interruptCount++;


    /*
     * Check each port for work, stop when we find some.
     * Next time we're called for this module, continue
     * scanning with the next port (enforce fairness).
     */
    for (scan = 1; scan <= 8; scan++) {
        int port = (qt->scan + scan) & 7;
        TY_GSOCTAL_DEV *dev = &qt->dev[port];
        SCC2698_CHAN *chan;
        int block;
        int key;

        if (!dev->created)
            continue;

        block = dev->block;
        chan = dev->chan;
        regs = dev->regs;

        key = intLock();
        sr = chan->u.r.sr;

        /* Only examine the active interrupts */
        isr = regs->u.r.isr & qt->imr[block];

        /* Channel B interrupt data is on the upper nibble */
        if ((port % 2) == 1)
            isr >>= 4;

        if (isr & 0x02) /* a byte needs to be read */
        {
            char inChar = chan->u.r.rhr;

            tyIRd(&dev->tyDev, inChar);
            dev->readCount++;
        }

        if (isr & 0x01) /* a byte needs to be sent */
        {
            char outChar;

            if (tyITx(&dev->tyDev, &outChar) == OK) {
                chan->u.w.thr = outChar;
                dev->writeCount++;
                chan->u.w.cr = 0;   /* Null command */
                flush = &chan->u.w.cr;
            }
            else {
                /* deactivate Tx INT and disable Tx INT */
                qt->imr[block] &= ~dev->irqEnable;
                regs->u.w.imr = qt->imr[block];
                flush = &regs->u.w.imr;
            }
        }

        /* Reset errors */
        if (sr & 0xf0) {
            dev->errorCount++;
            chan->u.w.cr = 0x40;
            flush = &chan->u.w.cr;
        }

        intUnlock(key);

        /* Exit after processing one channel */
        if ((isr & 0x03) || (sr & 0xf0)) {
            qt->scan = port;
            break;
        }
    }

    if (flush)
        isr = *flush;    /* Flush last write cycle */
}

/******************************************************************************
 *
 * tyGSOctalStartup - transmitter startup routine
 *
 * Call interrupt level character output routine.
*/
LOCAL void tyGSOctalStartup
    (
    TY_GSOCTAL_DEV *dev    /* ty device to start up */
    )
{
    char outChar;
    QUAD_TABLE *qt = dev->qt;
    SCC2698 *regs = dev->regs;
    SCC2698_CHAN *chan = dev->chan;
    int block = dev->block;
    int key;

    key = intLock();
    if (tyITx (&dev->tyDev, &outChar) == OK) {
        if (chan->u.r.sr & 0x04)
            chan->u.w.thr = outChar;

        qt->imr[block] |= dev->irqEnable; /* activate Tx interrupt */
        regs->u.w.imr = qt->imr[block]; /* enable Tx interrupt */
        intUnlock(key);
    }
    else {
        qt->imr[block] &= ~dev->irqEnable;
        regs->u.w.imr = qt->imr[block];
        intUnlock(key);
    }
}


/******************************************************************************
 *
 * Command Registration with iocsh
 */

/* tyGSOctalDrv */
static const iocshArg tyGSOctalDrvArg0 = {"maxModules", iocshArgInt};
static const iocshArg * const tyGSOctalDrvArgs[1] = {&tyGSOctalDrvArg0};
static const iocshFuncDef tyGSOctalDrvFuncDef =
    {"tyGSOctalDrv",1,tyGSOctalDrvArgs};
static void tyGSOctalDrvCallFunc(const iocshArgBuf *args)
{
    tyGSOctalDrv(args[0].ival);
}

/* tyGSOctalReport */
static const iocshFuncDef tyGSOctalReportFuncDef = {"tyGSOctalReport",0,NULL};
static void tyGSOctalReportCallFunc(const iocshArgBuf *args)
{
    tyGSOctalReport();
}

/* tyGSOctalModuleInit */
static const iocshArg tyGSOctalModuleInitArg0 = {"moduleID",iocshArgString};
static const iocshArg tyGSOctalModuleInitArg1 = {"RS<nnn>",iocshArgString};
static const iocshArg tyGSOctalModuleInitArg2 = {"intVector", iocshArgInt};
static const iocshArg tyGSOctalModuleInitArg3 = {"carrier#", iocshArgInt};
static const iocshArg tyGSOctalModuleInitArg4 = {"slot", iocshArgInt};
static const iocshArg * const tyGSOctalModuleInitArgs[5] = {
    &tyGSOctalModuleInitArg0, &tyGSOctalModuleInitArg1,
    &tyGSOctalModuleInitArg2, &tyGSOctalModuleInitArg3, &tyGSOctalModuleInitArg4};
static const iocshFuncDef tyGSOctalModuleInitFuncDef =
    {"tyGSOctalModuleInit",5,tyGSOctalModuleInitArgs};
static void tyGSOctalModuleInitCallFunc(const iocshArgBuf *args)
{
    tyGSOctalModuleInit(args[0].sval,args[1].sval,args[2].ival,args[3].ival,
        args[4].ival);
}

/* tyGSOctalDevCreate */
static const iocshArg tyGSOctalDevCreateArg0 = {"devName",iocshArgString};
static const iocshArg tyGSOctalDevCreateArg1 = {"moduleID", iocshArgString};
static const iocshArg tyGSOctalDevCreateArg2 = {"port", iocshArgInt};
static const iocshArg tyGSOctalDevCreateArg3 = {"rdBufSize", iocshArgInt};
static const iocshArg tyGSOctalDevCreateArg4 = {"wrBufSize", iocshArgInt};
static const iocshArg * const tyGSOctalDevCreateArgs[5] = {
    &tyGSOctalDevCreateArg0, &tyGSOctalDevCreateArg1,
    &tyGSOctalDevCreateArg2, &tyGSOctalDevCreateArg3,
    &tyGSOctalDevCreateArg4};
static const iocshFuncDef tyGSOctalDevCreateFuncDef =
    {"tyGSOctalDevCreate",5,tyGSOctalDevCreateArgs};
static void tyGSOctalDevCreateCallFunc(const iocshArgBuf *arg)
{
    tyGSOctalDevCreate(arg[0].sval, arg[1].sval, arg[2].ival,
       arg[3].ival, arg[4].ival);
}

/* tyGSOctalDevCreateAll */
static const iocshArg tyGSOctalDevCreateAllArg0 = {"devName",iocshArgString};
static const iocshArg tyGSOctalDevCreateAllArg1 = {"moduleID", iocshArgString};
static const iocshArg tyGSOctalDevCreateAllArg2 = {"rdBufSize", iocshArgInt};
static const iocshArg tyGSOctalDevCreateAllArg3 = {"wrBufSize", iocshArgInt};
static const iocshArg * const tyGSOctalDevCreateAllArgs[4] = {
    &tyGSOctalDevCreateAllArg0, &tyGSOctalDevCreateAllArg1,
    &tyGSOctalDevCreateAllArg2, &tyGSOctalDevCreateAllArg3 };
static const iocshFuncDef tyGSOctalDevCreateAllFuncDef =
    {"tyGSOctalDevCreateAll",4,tyGSOctalDevCreateAllArgs};
static void tyGSOctalDevCreateAllCallFunc(const iocshArgBuf *arg)
{
    tyGSOctalDevCreateAll(arg[0].sval, arg[1].sval, arg[2].ival, arg[3].ival);
}


/* tyGSOctalConfig */
static const iocshArg tyGSOctalConfigArg0 = {"devName",iocshArgString};
static const iocshArg tyGSOctalConfigArg1 = {"baud", iocshArgInt};
static const iocshArg tyGSOctalConfigArg2 = {"parity", iocshArgString};
static const iocshArg tyGSOctalConfigArg3 = {"stop", iocshArgInt};
static const iocshArg tyGSOctalConfigArg4 = {"bits", iocshArgInt};
static const iocshArg tyGSOctalConfigArg5 = {"flow", iocshArgString};
static const iocshArg * const tyGSOctalConfigArgs[6] = {
    &tyGSOctalConfigArg0, &tyGSOctalConfigArg1,
    &tyGSOctalConfigArg2, &tyGSOctalConfigArg3,
    &tyGSOctalConfigArg4, &tyGSOctalConfigArg5};
static const iocshFuncDef tyGSOctalConfigFuncDef =
    {"tyGSOctalConfig",6,tyGSOctalConfigArgs};
static void tyGSOctalConfigCallFunc(const iocshArgBuf *arg)
{
    tyGSOctalConfig(arg[0].sval, arg[1].ival, arg[2].sval[0],
        arg[3].ival, arg[4].ival, arg[5].sval[0]);
}

static void tyGSOctalRegistrar(void) {
    iocshRegister(&tyGSOctalDrvFuncDef,tyGSOctalDrvCallFunc);
    iocshRegister(&tyGSOctalReportFuncDef,tyGSOctalReportCallFunc);
    iocshRegister(&tyGSOctalModuleInitFuncDef,tyGSOctalModuleInitCallFunc);
    iocshRegister(&tyGSOctalDevCreateFuncDef,tyGSOctalDevCreateCallFunc);
    iocshRegister(&tyGSOctalDevCreateAllFuncDef, tyGSOctalDevCreateAllCallFunc);
    iocshRegister(&tyGSOctalConfigFuncDef,tyGSOctalConfigCallFunc);
}
epicsExportRegistrar(tyGSOctalRegistrar);
