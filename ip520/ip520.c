/*
FILENAME...     ip520.c
USAGE...        Acromag IP520/521 VxWorks driver.

Version:        $Revision$
Modified By:    $Author$
Last Modified:  $Date$
HeadURL:        $URL$
*/

/**************************************************************************
 History:
 who  when       what
 ---  ---------- ------------------------------------------------
 RLS  2014/05/05 Original. Based on Andrew Johnson's "tyGSOctal".
 RLS  2016/12/14 Added IP521 support
**************************************************************************/

/*
Implentation Notes:

- The three Rx Error Flags (Overrun, Parity, Framing) in the LSR register are
  cleared whenever the CPU reads the LSR register. Therefore, Rx error
  processing must/should be done whenever the LSR register is read. The
  exception to this rule is IP520Report(). It is assumed that IP520Report() will
  only be called to identify a known problem.

- see README file for more info.

*/

/* This is needed for vxWorks 6.x to prevent an obnoxious compiler warning */
#define _VSB_CONFIG_FILE <../lib/h/config/vsbConfig.h>

#include <rebootLib.h>
#include <intLib.h>
#include <errnoLib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <logLib.h>
#include <taskLib.h>
#include <vxLib.h>
#include <sioLib.h>

#include "epicsString.h"
#include "epicsInterrupt.h"
#include "drvIpac.h"
#include "iocsh.h"
#include "epicsExport.h"

#include "Acromag_ip_modules.h"
#include "IP520Ext.h"
#include "IP520Int.h"

/*
 * Macros
 */

/* VxWorks 6.9 changed tyDevinit()'s txStartup argument type.
 * This avoids compiler warnings for that call.
 */
#if !defined(_WRS_VXWORKS_MAJOR) || \
    (_WRS_VXWORKS_MAJOR == 6) && (_WRS_VXWORKS_MINOR < 9)
#define TY_DEVSTART_PTR FUNCPTR
#endif

#define isPower2(x) ((x) && !((x) & ((x) - 1)))

#if __STDC_VERSION__ < 199901L
#  define fn_nm __FUNCTION__
#else
#  define fn_nm __func__
#endif


/*
 * Module variables
 */

LOCAL MOD_TABLE *IP520Modules;
LOCAL int IP520MaxModules;
int IP520LastModule;

LOCAL int IP520DrvNum;      /* driver number assigned to this driver */
LOCAL epicsUInt8 savedlcr;   /* Saved LCR value for EFROn & EFROff functions. */

/*
 * Forward declarations
 */
void         IP520Int(int);
LOCAL void   IP520InitChannel(MOD_TABLE *, int);
LOCAL int    IP520RebootHook(int);
LOCAL MOD_TABLE * IP520OctalFindQT(const char *);
LOCAL int    IP520Open(TY_IP520_DEV *, const char *, int);
LOCAL int    IP520Write(TY_IP520_DEV *, char *, long);
LOCAL STATUS IP520Ioctl(TY_IP520_DEV *, int, int);
LOCAL void   IP520TxStartup(TY_IP520_DEV *);
LOCAL STATUS IP520BaudSet(TY_IP520_DEV *, int);
LOCAL void   IP520OptsSet(TY_IP520_DEV *, int);
LOCAL void   EFROn(REGMAP *);
LOCAL void   EFROff(REGMAP *);
LOCAL void   IsrErrMsg(epicsUInt8, TY_IP520_DEV *);


/******************************************************************************
 *
 * IP520Drv - initialize the IP520 tty driver
 *
 * This routine initializes the serial driver, sets up interrupt vectors, and
 * performs hardware initialization of the serial ports.
 *
 * This routine should be called exactly once, before any reads, writes, or
 * calls to IP520DevCreate().
 *
 * This routine takes as an argument the maximum number of IP modules
 * to support.
 * For example:
 * .CS
 *    int status;
 *    status = IP520Drv(4);
 * .CE
 *
 * RETURNS: OK, or ERROR if the driver cannot be installed.
 *
 * SEE ALSO: IP520DevCreate()
*/

STATUS IP520Drv(int maxModules)
{
    /* check if driver already installed */
    if (IP520DrvNum > 0)
        return OK;

    IP520MaxModules = maxModules;
    IP520LastModule = 0;
    IP520Modules = (MOD_TABLE *) calloc(maxModules, sizeof(MOD_TABLE));

    if (!IP520Modules)
    {
        printf("%s: Memory allocation failed!\n", fn_nm);
        return ERROR;
    }

    rebootHookAdd(IP520RebootHook);
    IP520DrvNum = iosDrvInstall(IP520Open, NULL, IP520Open, NULL, tyRead, IP520Write, IP520Ioctl);

    return(IP520DrvNum == ERROR ? ERROR : OK);
}

void IP520Report(void)
{
    int mod;

    for (mod = 0; mod < IP520LastModule; mod++)
    {
        MOD_TABLE *pmod = &IP520Modules[mod];
        int port;

        printf("Module %d: carrier=%d slot=%d irqCnt=%u\n", mod, pmod->carrier, pmod->slot, pmod->irqCount);

        for (port = 0; port < 8; port++)
        {
            TY_IP520_DEV *dev = &pmod->dev[port];
            REGMAP *regs      = dev->regs;

            if (dev->created)
            {
                printf("  Port %d: %lu chars in, %lu chars out, %u overrun, %u parity, %u framing\n", port,
                       dev->readCount, dev->writeCount, dev->overCount, dev->parityCount, dev->frameCount);
                printf("  Port %d: IER = 0x%2.2hhX, LSR = 0x%2.2hhX, MCR = 0x%2.2hhX, LCR = 0x%2.2hhX\n", port,
                       regs->u.read.ier, regs->u.read.lsr, regs->u.read.mcr, regs->u.read.lcr);
            }
        }
    }
}

LOCAL int IP520RebootHook(int type)
{
    int mod;
    int key = intLock();    /* disable interrupts */

    for (mod = 0; mod < IP520LastModule; mod++)
    {
        MOD_TABLE *pmod = &IP520Modules[mod];
        int port;

        for (port=0; port < 8; port++)
        {
            TY_IP520_DEV *dev = &pmod->dev[port];

            if (dev->created)
            {
                dev->regs->u.write.ier = 0;
                dev->regs->u.write.mcr &= ~(0x08); /* Port interrupt disable. */
                if (dev->mode != RS232)
                    dev->regs->u.write.mcr &= ~(0x03); /* disable Tx & Rx transceivers. */
            }
            ipmIrqCmd(pmod->carrier, pmod->slot, 0, ipac_irqDisable);
            ipmIrqCmd(pmod->carrier, pmod->slot, 1, ipac_irqDisable);

            ipmIrqCmd(pmod->carrier, pmod->slot, 0, ipac_statUnused);
        }
    }
    free(IP520Modules);

    intUnlock(key);
    return OK;
}

/******************************************************************************
 * IP520ModuleInit - initialize an IP module
 *
 * The routine initializes the specified IP module. Each module is
 * characterized by its model name, interrupt vector, carrier board
 * number, and slot number on the board. No new setup is done if a
 * MOD_TABLE entry already exists with the same carrier and slot
 * numbers.
 *
 * For example:
 * .CS
 *    int idx;
 *    idx = IP520ModuleInit("SBS232-1", "232", 0x60, 0, 1);
 * .CE
 *
 *
 * RETURNS: Index into module table, or ERROR if the driver is not
 * installed, the channel is invalid, or the device already exists.
 *
 * SEE ALSO: IP520Drv()
*/
int IP520ModuleInit
    (
    const char * moduleID,       /* IP module name */
    const char * type,           /* IP module type 232/422/485 */
    int          int_num,        /* Interrupt vector */
    int          carrier,        /* carrier number */
    int          slot            /* slot number */
    )
{
    int modelID, status, mod, RSmode;
    MOD_TABLE *pmod;

    /*
     * Check for the driver being installed.
     */
    if (IP520DrvNum <= 0)
    {
        errnoSet(S_ioLib_NO_DRIVER);
        return ERROR;
    }

    if (!moduleID || !type)
    {
        errnoSet(EINVAL);
        return ERROR;
    }

    /*
     * Check the IP module type.
     */
    if (strstr(type, "232"))
    {
        modelID = IP520;
        RSmode = RS232;
    }
    else if (strstr(type, "422"))
    {
        modelID = IP521;
        RSmode = RS422;
    }
    else if (strstr(type, "485"))
    {
        modelID = IP521;
        RSmode = RS485;
    }
    else
    {
        printf("*Error* %s: Unsupported module type: %s\n", fn_nm, type);
        errnoSet(EINVAL);
        return ERROR;
    }

    /*
     * Validate the IP module location and type.
     */
    status = ipmValidate(carrier, slot, ACROMAG_ID, modelID);
    if (status)
    {
        printf("%s: IPAC Module validation failed\n"
            "    Carrier:%d slot:%d modelID:0x%x\n",
            fn_nm, carrier, slot, modelID);

        switch (status)
        {
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
    for (mod = 0; mod < IP520LastModule; mod++)
    {
        pmod = &IP520Modules[mod];
        if (pmod->carrier == carrier && pmod->slot == slot)
            break;
    }

    /* Create a new quad table entry if not there */
    if (mod >= IP520LastModule)
    {
        void *addrIO;
        char *ID = epicsStrDup(moduleID);
        REGMAP *preg;
        int port;

        if (IP520LastModule >= IP520MaxModules)
        {
            printf("%s: Maximum module count exceeded!\n", fn_nm);
            errnoSet(ENOSPC);
            return ERROR;
        }

        pmod = &IP520Modules[IP520LastModule];
        pmod->modelID = modelID;
        pmod->carrier = carrier;
        pmod->slot = slot;
        pmod->moduleID = ID;

        addrIO = ipmBaseAddr(carrier, slot, ipac_addrIO);
        preg = (REGMAP *) addrIO;

        for (port = 0; port < 8; port++)
        {
            pmod->dev[port].created = 0;
            pmod->dev[port].regs = &preg[port];
            pmod->dev[port].pmod= pmod;
            pmod->dev[port].regs->u.write.ier = 0;
            pmod->dev[port].regs->u.write.scr = int_num;
            pmod->dev[port].mode = RSmode;
        }

        if (ipmIntConnect(carrier, slot, int_num, IP520Int, IP520LastModule))
        {
            printf("%s: Unable to connect ISR\n", fn_nm);
            return ERROR;
        }

        ipmIrqCmd(carrier, slot, 0, ipac_irqEnable);
        ipmIrqCmd(carrier, slot, 1, ipac_irqEnable);
        ipmIrqCmd(carrier, slot, 0, ipac_statActive);
    }

    return IP520LastModule++;
}

/******************************************************************************
 * IP520DevCreate - create a device for a serial port on an IP module
 *
 * This routine creates a device on a specified serial port.  Each port
 * to be used should have exactly one device associated with it by calling
 * this routine.
 *
 * For instance, to create the device "/SBS/0,1/3", with buffer sizes
 * of 512 bytes, the proper calls would be:
 * .CS
 *    if (IP520ModuleInit("232-1", "232", 0x60, 0, 1) != ERROR) {
 *       char *nam = IP520DevCreate ("/SBS/0,1/3", "232-1", 3, 512, 512);
 * }
 * .CE
 *
 * RETURNS: Pointer to device name, or NULL if the driver is not
 * installed, the channel is invalid, or the device already exists.
 *
 * SEE ALSO: IP520Drv()
*/
const char * IP520DevCreate
    (
    char *       name,           /* name to use for this device          */
    const char * moduleID,       /* IP module name                       */
    int          port,           /* port on module for this device [0-7] */
    int          rdBufSize,      /* read buffer size, in bytes           */
    int          wrtBufSize      /* write buffer size, in bytes          */
    )
{
    TY_IP520_DEV *dev;
    MOD_TABLE *pmod = IP520OctalFindQT(moduleID);

    if (!name || !pmod)
        return NULL;

    /* if this doesn't represent a valid port, don't do it */
    if (port < 0 || port > 7)
        return NULL;

    dev = &pmod->dev[port];

    /* if there is a device already on this channel, don't do it */
    if (dev->created)
        return NULL;

    /* initialize the ty descriptor */
    if (tyDevInit (&dev->tyDev, rdBufSize, wrtBufSize, (TY_DEVSTART_PTR) IP520TxStartup) != OK)
        return NULL;

    /* initialize the channel hardware */
    IP520InitChannel(pmod, port);

    /* mark the device as created, and add the device to the I/O system */
    dev->created = TRUE;

    if (iosDevAdd(&dev->tyDev.devHdr, name, IP520DrvNum) != OK)
        return NULL;

    return name;
}

/******************************************************************************
 * IP520DevCreateAll - create devices for all ports on a module
 *
 * This routine creates up to 8 devices, one for each port that has not
 * already been created.  Use this after calling IP520DevCreate to
 * set up any ports that should not use the standard configuration.
 * The port names are constructed by appending the digits 0 through 7 to
 * the base name string given in the first argument.
 *
 * For instance, to create devices "/tyGS/0/0" through "/tyGS/0/7", with
 * buffer sizes of 512 bytes, the proper calls would be:
 * .CS
 *    if (IP520ModuleInit("232-1", "232", 0x60, 0, 1) != ERROR) {
 *       IP520DevCreateAll ("/tyGS/0/", "232-1", 512, 512);
 * }
 * .CE
 *
 * RETURNS: OK, or ERROR if the driver is not installed, or any device
 * cannot be initialized.
 *
 * SEE ALSO: IP520Drv(), IP520DevCreate()
 */
STATUS IP520DevCreateAll
    (
    const char * base,           /* base name for these devices      */
    const char * moduleID,       /* module identifier from the
                                 * call to IP520ModuleInit(). */
    int          rdBufSize,      /* read buffer size, in bytes       */
    int          wrtBufSize      /* write buffer size, in bytes      */
    )
{
    MOD_TABLE *pmod = IP520OctalFindQT(moduleID);
    int port;

    if (!pmod || !base)
    {
        errnoSet(EINVAL);
        return ERROR;
    }

    for (port=0; port < 8; port++)
    {
        TY_IP520_DEV *dev = &pmod->dev[port];
        char name[256];

        /* if there is a device already on this channel, ignore it */
        if (dev->created)
            continue;

        /* initialize the ty descriptor */
        if (tyDevInit(&dev->tyDev, rdBufSize, wrtBufSize, (TY_DEVSTART_PTR) IP520TxStartup) != OK)
            return ERROR;

        /* initialize the channel hardware */
        IP520InitChannel(pmod, port);

        /* mark the device as created, and give it to the I/O system */
        dev->created = TRUE;

        sprintf(name, "%s%d", base, port);

        if (iosDevAdd(&dev->tyDev.devHdr, name, IP520DrvNum) != OK)
            return ERROR;
    }
    return OK;
}


/******************************************************************************
 *
 * IP520OctalFindQT - Find a named module quadtable
 *
 * NOMANUAL
 */
LOCAL MOD_TABLE * IP520OctalFindQT(const char *moduleID)
{
    int mod;

    if (!moduleID)
        return NULL;

    for (mod = 0; mod < IP520LastModule; mod++)
        if (strcmp(moduleID, IP520Modules[mod].moduleID) == 0)
            return &IP520Modules[mod];

    return NULL;
}

/******************************************************************************
 *
 * IP520InitChannel - initialize a single channel
 *
 * NOMANUAL
 */
LOCAL void IP520InitChannel(MOD_TABLE *pmod, int port)
{
    TY_IP520_DEV *dev = &pmod->dev[port];
    REGMAP *regs      = dev->regs;
    int key;
    epicsUInt8 status;

    key = intLock();    /* disable interrupts during init */

    regs->u.write.ier = 0x0;   /* disable interrupts */
    status = regs->u.read.isr; /* clear interrupt status bits */

/*
 * Set up the default port configuration:
 * 9600 baud, no parity, 1 stop bit, 8 bits per char, no flow control
 */
    IP520BaudSet(dev, 9600);
    IP520OptsSet(dev, CS8 | CLOCAL);

    regs->u.write.ier |= 0x05;      /* enable FIFO and Rx interrupts */
    if (dev->mode != RS232)
        regs->u.write.mcr |= 0x01;  /* enable Rx transceiver */
    regs->u.write.mcr |= 0x08;      /* enable port interrupts */

    intUnlock(key);
}

/******************************************************************************
 *
 * IP520Open - open file to UART
 *
 * NOMANUAL
 */
LOCAL int IP520Open(TY_IP520_DEV *dev, const char * name, int mode)
{
    return (int) dev;
}


/******************************************************************************
 * IP520Write - Outputs a specified number of characters on a serial port
 *
 * NOMANUAL
 */
LOCAL int IP520Write
    (
    TY_IP520_DEV *dev,  /* device descriptor block */
    char *write_bfr,    /* ptr to an output buffer */
    long write_size     /* # bytes to write */
    )
{
    REGMAP *regs = dev->regs;
    int nbytes;

    /*
     * verify that the device descriptor is valid
     */
    if (!dev)
    {
        logMsg("%s: NULL device descriptor from %s\n", (int)fn_nm, (int)taskName(taskIdSelf()), 3,4,5,6);
        return -1;
    }

    if (dev->mode != RS232)
        regs->u.write.mcr &= ~(0x01);   /* Disable Rx transceiver */
        regs->u.write.mcr |= 0x02;      /* Enable  Tx transceiver */

    nbytes = tyWrite(&dev->tyDev, write_bfr, write_size);

    return nbytes;
}

/******************************************************************************
 *
 * IP520OptsSet - set channel serial options
 *
 * LOGIC
 *
 *  The Tx FIFO interrupt trigger level is configured to minimize interrupts
 *  without concern for Tx underrun. Hence, the level is always set to 8.
 *  Unless the application is sending more than 64 characters per line, the Tx
 *  interrupt is never enabled and hence the Tx FIFO interrupt trigger level is
 *  irrelevant in many cases.
 *
 *  The Rx FIFO interrupt trigger level is configured to minimize interrupts
 *  and to allow worst-case interrupt latency of 5ms without Rx overrun. The Rx
 *  Rx FIFO interrupt trigger level is based on the baudrate and if hardware
 *  handshaking is enabled or disabled.
 *
 *  IF baudrate = 1200, 2400, 4800 or 9600
 *      Set Rx level = 60 and Tx level = 8
 *  ELSE IF baudrate == 19200
 *      Set Rx level = 56 and Tx level = 8
 *  ELSE IF baudrate = 115200 or 230400
 *      IF hardware flow control is disabled
 *          Set Rx level = 8 and Tx level = 8
 *      ELSE
 *          Set Rx level = 56 and Tx level = 8
 *      ENDIF
 *  ELSE baudrate = 38400 or 57600
 *      IF hardware flow control is disabled
 *          Set Rx level = 16 and Tx level = 8
 *      ELSE
 *          Set Rx level = 56 and Tx level = 8
 *      ENDIF
 *  ENDIF
 *
 */

LOCAL void IP520OptsSet(TY_IP520_DEV * dev, int opts)
{
    REGMAP *regs    = dev->regs;
    epicsUInt8 llcr, lefr, lmcr, lisr, lfcr;
    int mask = (CSIZE | STOPB | PARENB | PARODD | CLOCAL);
    int baud, hardwareflowcontrol = 0;

    switch (opts & CSIZE)
    {
        case CS5:
            llcr = 0;
            break;
        case CS6:
            llcr = 1;
            break;
        case CS7:
            llcr = 2;
            break;
        case CS8:
        default:
            llcr = 3;
            break;
    }

    if (opts & STOPB)
        llcr |= 0x04; /* 1.5 or 2 Stop bits (default is 1 stop bit). */

    if (opts & PARENB)  /* Parity Enabled? */
    {
        llcr |= 0x08;
        if (!(opts & PARODD))
            llcr |= 0x10;  /* Even Parity. */
    }

    regs->u.write.lcr = llcr;
    llcr = regs->u.read.lcr;    /* Read to flush posted writes. */

    if (dev->mode == RS232)
    {
        if (!(opts & CLOCAL))   /* Hardware flow control */
            hardwareflowcontrol = 1;
    }
    else if ((dev->mode == RS422) && (!(opts & CLOCAL)))
        printf("*Warning* device %s configured for RTS/CTS handshaking is not supported for RS-422\n",
               dev->tyDev.devHdr.name);

    dev->opts = opts & mask;

    baud = dev->baud;
    if (baud <= 9600)
        lfcr = 0xC1;            /* Set Rx FIFO trigger level = 60. */
    else if (baud == 19200)
        lfcr = 0x81;            /* Set Rx FIFO trigger level = 56. */
    else if (baud >= 115200)
    {
        if (hardwareflowcontrol == 0)
            lfcr = 0x01;        /* Set Rx FIFO trigger level = 8. */
        else
            lfcr = 0x81;        /* Set Rx FIFO trigger level = 56. */
    }
    else                        /* For 38,400 and 57,600 baud. */
    {
        if (hardwareflowcontrol == 0)
            lfcr = 0x41;        /* Set Rx FIFO trigger level = 16. */
        else
            lfcr = 0x81;        /* Set Rx FIFO trigger level = 56. */
    }

    regs->u.write.fcr  = 0x00;      /* Clear FIFO's. */
    regs->u.write.fcr  = lfcr;      /* Set Rx FIFO trigger level based on baudrate,
                                     * Set Tx FIFO trigger level to 8 charaters. */
    if (dev->mode == RS232)
    {
        EFROn(regs);
        lefr = regs->u.read.isr;        /* Read EFR.*/
        if (hardwareflowcontrol == 0)
            lefr &= ~(0xC0);            /* Disable RTS/CTS flow control. */
        else
            lefr |=   0xC0;             /* Enable  RTS/CTS flow control. */
        regs->u.write.fcr = lefr;       /* Write to EFR. */
        lisr = regs->u.read.isr;        /* Read ISR to flush FCR posted writes. */
        EFROff(regs);

        lmcr = regs->u.read.mcr;
        if (hardwareflowcontrol == 0)
            lmcr &= ~(0x02);            /* Set RTS off. */
        else
            lmcr |=   0x02;             /* Set RTS on.  */
        regs->u.write.mcr = lmcr;
        lmcr = regs->u.read.mcr;        /* Read to flush posted writes. */
    }
}

/******************************************************************************
 *
 * IP520BaudSet - set channel baud rate
 *
 * NOMANUAL
 */

LOCAL STATUS IP520BaudSet(TY_IP520_DEV *dev, int baud)
{
    int rtnstat = 0;
    REGMAP *regs = dev->regs;
    epicsUInt8 llcr, lmcr, dlm, dll;

    if (dev->baud == baud)              /* Any changes? */
        return(rtnstat);                /* No. Exit.    */

    EFROn(regs);
    regs->u.write.lcr = savedlcr;       /* Restore LCR to saved value for following MCR write, but
                                           don't disable writes to enhanced functions (EF's). */
    if (baud == 57600)
        regs->u.write.mcr |=   0x80;    /* Only 57600 requires MCR bit#7 = 1; crystal freq. divide by 4.*/
    else
        regs->u.write.mcr &= ~(0x80);   /* MCR bit#7 = 0; crystal freq. divide by 1. */
    lmcr = regs->u.read.mcr;            /* Read MCR to flush posted writes. */
    EFROff(regs);

    regs->u.write.lcr |= 0x80;          /* Expose DLL/DLM; hide RBR/THR/IER. */
    llcr = regs->u.read.lcr;            /* Read LCR to flush posted writes. */

    switch (baud)
    {
        case 1200:
            dlm = 0x03; /* DLM */
            dll = 0x00; /* DLL */
            break;
        case 2400:
            dlm = 0x01; /* DLM */
            dll = 0x80; /* DLL */
            break;
        case 4800:
            dlm = 0x00; /* DLM */
            dll = 0xC0; /* DLL */
            break;
        case 9600:
            dlm = 0x00; /* DLM */
            dll = 0x60; /* DLL */
            break;
        case 19200:
            dlm = 0x00; /* DLM */
            dll = 0x30; /* DLL */
            break;
        case 38400:
            dlm = 0x00; /* DLM */
            dll = 0x18; /* DLL */
            break;
        case 57600:
            dlm = 0x00; /* DLM */
            dll = 0x04; /* DLL */
            break;
        case 115200:
            dlm = 0x00; /* DLM */
            dll = 0x08; /* DLL */
            break;
        case 230400:
            dlm = 0x00; /* DLM */
            dll = 0x04; /* DLL */
            break;
        default:
            errnoSet(EINVAL);
            rtnstat = -1;
    }

    if (rtnstat != -1)
    {
        regs->u.write.ier = dlm; /* DLM */
        regs->u.write.thr = dll; /* DLL */
        dev->baud = baud;
    }

    regs->u.write.lcr &= ~(0x80); /* Hide DLL/DLM; expose RBR/THR. */
    llcr = regs->u.read.lcr;      /* Read to flush posted writes. */

    return rtnstat;
}

/******************************************************************************
 *
 * IP520Ioctl - special device control
 *
 * This routine handles FIOBAUDRATE, SIO_BAUD_SET and SIO_HW_OPTS_SET
 * requests and passes all others to tyIoctl().
 *
 * RETURNS: OK, or ERROR if invalid input.
 */
LOCAL STATUS IP520Ioctl
    (
    TY_IP520_DEV *dev,  /* device to control */
    int request,        /* request code */
    int arg             /* some argument */
    )
{
    STATUS status = 0;
    int key;

    switch (request)
    {
        case FIOBAUDRATE:
        case SIO_BAUD_SET:
            if (dev->baud != arg)                   /* Any change? */
            {
                key = intLock();
                status = IP520BaudSet(dev, arg);
                IP520OptsSet(dev, dev->opts);   /* Always call after IP520BaudSet. */
                intUnlock(key);
            }
            break;
        case SIO_BAUD_GET:
            *(int *)arg = dev->baud;
            break;
        case SIO_HW_OPTS_SET:
            key = intLock();
            IP520OptsSet(dev, arg);
            intUnlock(key);
            break;
        case SIO_HW_OPTS_GET:
            *(int *)arg = dev->opts;
            break;
        default:
            status = tyIoctl(&dev->tyDev, request, arg);
            break;
    }

    return status;
}

/******************************************************************************
 *
 * IP520Config - special device control (old version)
 *
 * This routine sets the baud rate, parity, stop bits, word size, and
 * flow control for the specified port.
 *
 */
STATUS IP520Config(char *name, int baud, char parity, int stop, int bits, char flow)
{
    TY_IP520_DEV *dev = (TY_IP520_DEV *) iosDevFind(name, NULL);
    int opts = 0;
    int key;

    if (!dev || strcmp(dev->tyDev.devHdr.name, name) != 0)
    {
        printf("%s: Device %s not found\n", fn_nm, name);
        return(ERROR);
    }

    switch (bits)
    {
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

    key = intLock();
    IP520BaudSet(dev, baud);
    IP520OptsSet(dev, opts);    /* Always call after IP520BaudSet. */
    intUnlock(key);
    return(OK);
}

/*****************************************************************************
 * IP520Int - interrupt level processing
 *
 * LOGIC
 * Loop through each of the 8 serial ports, until no Rx or Tx processing required.
 *
 */
void IP520Int(int mod)
{
    MOD_TABLE *pmod = &IP520Modules[mod];
    REGMAP *regs;
    volatile epicsUInt8 dummy, *flush = NULL;
    int scan = 0;

    pmod->irqCount++;

    while (scan <= 7)
    {
        epicsUInt8 isr, lsr, ier;
        TY_IP520_DEV *dev = &pmod->dev[scan];
        int key, work = 0;

        if (!dev->created)
        {
            scan++;
            continue;
        }

        regs = dev->regs;
        key = intLock(); /* Is this required? */
        isr = regs->u.read.isr;
        ier = regs->u.read.ier;
        lsr = regs->u.read.lsr;

        if (lsr & 0x0E)         /* Check for overrun, parity or framing error. */
            IsrErrMsg(lsr, dev);

        while (lsr & 0x01)     /* RBR has a character to read. */
        {
            char inChar = regs->u.read.rbr;

            tyIRd(&dev->tyDev, inChar);
            dev->readCount++;
            work = 1;
            lsr = regs->u.read.lsr;
            if (lsr & 0x0E)         /* Check for overrun, parity or framing error. */
                IsrErrMsg(lsr, dev);
        }

        if ((ier & 0x02) && (lsr & 0x40)) /* If Tx interrupts are enabled, AND, Tx is empty (TEMT). */
        {
            STATUS status = OK;
            char outChar;
            int TxCtr = 64;

            while((TxCtr > 0) && ((status = tyITx(&dev->tyDev, &outChar)) == OK))
            {
                regs->u.write.thr = outChar;
                dev->writeCount++;
                TxCtr--;
            }

            if (status == ERROR)
            {
                if (dev->mode != RS232)
                {
                    regs->u.write.mcr &= ~(0x02);   /* Disable Tx transceiver */
                    regs->u.write.mcr |= 0x01;      /* Enable  Rx transceiver */
                }
                /* deactivate Tx INT and disable Tx INT */
                regs->u.write.ier &= ~(0x02);
                ier = regs->u.read.ier;
                flush = &regs->u.write.ier;
            }
            work = 1;
        }

        if (work == 0)
            scan++;

        intUnlock(key); /* Is this required? */
    }

    if (flush)
        dummy = *flush;    /* Flush last write cycle */
}


LOCAL void IsrErrMsg(epicsUInt8 lsr, TY_IP520_DEV *dev)
{
    int cnt;
    char *errmsg;
    static char overrunErrMsg[] = "      : Rx overrun ctr = xxx\n";
    static char parityErrMsg[]  = "      : Rx parity  ctr = xxx\n";
    static char framingErrMsg[] = "      : Rx framing ctr = xxx\n";

    if (lsr & 0x02)         /* Check for Rx overrun. */
    {
        cnt = ++dev->overCount;
        errmsg = overrunErrMsg;
    }
    else if (lsr & 0x04)    /* Check for parity error. */
    {
        cnt = ++dev->parityCount;
        errmsg = parityErrMsg;
    }
    else if (lsr & 0x08)    /* Check for framing error. */
    {
        cnt = ++dev->frameCount;
        errmsg = framingErrMsg;
    }
    else
        return;

    if (cnt <= 10 || isPower2(cnt))
    {
        int size;
        size = strlen(dev->pmod->moduleID);
        size = (size >= 6) ? 6 : (size - 1);
        strncpy(errmsg, dev->pmod->moduleID, size);
        errmsg[25] = '0' + (cnt / 100) % 10;
        errmsg[26] = '0' + (cnt / 10) % 10;
        errmsg[27] = '0' + cnt % 10;
        epicsInterruptContextMessage(errmsg);
    }
}

/******************************************************************************
 *
 * IP520TxStartup - transmitter startup routine
 *
 * LOGIC
 *  Initialize status = OK.
 *  Disable interrupts for processing LSR and filling the Tx FIFO.
 *  Read line status register (LSR).
 *  IF LSR shows Rx Overrun error.
 *      Call IsrErrMsg().
 *  ENDIF
 *
 *  IF Transmitter Hold Register is Empty (then FIFO is also empty).
 *      Set Tx counter to 64.
 *  ELSE
 *      Set Tx counter to 0 and let ISR fill the Tx FIFO.
 *  ENDIF
 *
 *  WHILE (TxCtr > 0, AND, another Tx character is available)
 *      Write character to Tx holding register (THR).
 *      Decrement TxCtr.
 *  ENDWHILE
 *
 *  IF another Tx character was NOT available (the tty ringbuffer is empty)
 *      Disable Tx interrupts.
 *  ELSE
 *      Enable Tx interrupts.
 *  ENDIF
 *
 *  Enable interrupts.
 */
LOCAL void IP520TxStartup(TY_IP520_DEV *dev)
{
    char outChar;
    REGMAP *regs = dev->regs;
    int key, TxCtr;
    STATUS status = OK;
    epicsUInt8 lsr;

    key = intLock();
    lsr = regs->u.read.lsr;
    if (lsr & 0x0E)         /* Check for overrun, parity or framing error. */
        IsrErrMsg(lsr, dev);

    if (lsr & 0x20)
        TxCtr = 64;
    else
        TxCtr = 0;

    while((TxCtr > 0) && ((status = tyITx(&dev->tyDev, &outChar)) == OK))
    {
        regs->u.write.thr = outChar;
        dev->writeCount++;
        TxCtr--;
    }

    if ((status == ERROR) && (dev->mode == RS232))
        regs->u.write.ier &= ~(0x02);   /* Disable Tx interrupt */
    else
        regs->u.write.ier |= 0x02;      /*  Enable Tx interrupt */

    intUnlock(key);
}

/* EFROn - Enable Enhanced Functions */
LOCAL void EFROn(REGMAP *regs)
{
    epicsUInt8 llcr, lefr;

    savedlcr = regs->u.read.lcr;        /* Save LCR. */
    regs->u.write.lcr = 0xBF;           /* Expose EFR/Xon-1/Xon-2/Xoff-1/Xoff-2; hide ISR/FCR/MCR/LSR/MSR/SCR. */
    llcr = regs->u.read.lcr;            /* Read LCR to flush posted writes. */
    regs->u.write.fcr |= 0x10;          /* Write to EFR; enable writes to enhanced functions. */
    lefr = regs->u.read.isr;            /* Read EFR to flush posted writes. */
}

/* EFROff - Disable Enhanced Functions */
LOCAL void EFROff(REGMAP * regs)
{
    epicsUInt8 llcr, lefr;

    regs->u.write.fcr &= ~(0x10);       /* Write to EFR:4; disable writes to enhanced functions.
                                         * Expose RBR/THR/IER; hide DLL/DLM, AND,
                                         * Expose ISR/FCR/MCR/LSR/MSR/SCR; hide EFR/Xon-1/Xon-2/Xoff-1/Xoff-2. */
    lefr = regs->u.read.isr;            /* Read EFR to flush posted writes. */
    regs->u.write.lcr = savedlcr;       /* Restore LCR to save value. */
    llcr = regs->u.read.lcr;            /* Read LCR to flush posted writes. */
}

/******************************************************************************
 *
 * Command Registration with iocsh
 */

/* IP520Drv */
static const iocshArg IP520DrvArg0 = {"maxModules", iocshArgInt};
static const iocshArg * const IP520DrvArgs[1] = {&IP520DrvArg0};
static const iocshFuncDef IP520DrvFuncDef = {"IP520Drv", 1, IP520DrvArgs};
static void IP520DrvCallFunc(const iocshArgBuf *args)
{
    IP520Drv(args[0].ival);
}

/* IP520Report */
static const iocshFuncDef IP520ReportFuncDef = {"IP520Report", 0, NULL};
static void IP520ReportCallFunc(const iocshArgBuf *args)
{
    IP520Report();
}

/* IP520ModuleInit */
static const iocshArg IP520ModuleInitArg0 = {"moduleID",  iocshArgString};
static const iocshArg IP520ModuleInitArg1 = {"RS<nnn>",   iocshArgString};
static const iocshArg IP520ModuleInitArg2 = {"intVector", iocshArgInt};
static const iocshArg IP520ModuleInitArg3 = {"carrier#",  iocshArgInt};
static const iocshArg IP520ModuleInitArg4 = {"slot",      iocshArgInt};

static const iocshArg * const IP520ModuleInitArgs[5] = {&IP520ModuleInitArg0, &IP520ModuleInitArg1, &IP520ModuleInitArg2,
                                                        &IP520ModuleInitArg3, &IP520ModuleInitArg4};
static const iocshFuncDef IP520ModuleInitFuncDef = {"IP520ModuleInit", 5, IP520ModuleInitArgs};
static void IP520ModuleInitCallFunc(const iocshArgBuf *args)
{
    IP520ModuleInit(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

/* IP520DevCreate */
static const iocshArg IP520DevCreateArg0 = {"devName",   iocshArgString};
static const iocshArg IP520DevCreateArg1 = {"moduleID",  iocshArgString};
static const iocshArg IP520DevCreateArg2 = {"port",      iocshArgInt};
static const iocshArg IP520DevCreateArg3 = {"rdBufSize", iocshArgInt};
static const iocshArg IP520DevCreateArg4 = {"wrBufSize", iocshArgInt};

static const iocshArg * const IP520DevCreateArgs[5] = {&IP520DevCreateArg0, &IP520DevCreateArg1, &IP520DevCreateArg2,
                                                       &IP520DevCreateArg3, &IP520DevCreateArg4};
static const iocshFuncDef IP520DevCreateFuncDef = {"IP520DevCreate", 5, IP520DevCreateArgs};
static void IP520DevCreateCallFunc(const iocshArgBuf *arg)
{
    IP520DevCreate(arg[0].sval, arg[1].sval, arg[2].ival, arg[3].ival, arg[4].ival);
}

/* IP520DevCreateAll */
static const iocshArg IP520DevCreateAllArg0 = {"devName",iocshArgString};
static const iocshArg IP520DevCreateAllArg1 = {"moduleID", iocshArgString};
static const iocshArg IP520DevCreateAllArg2 = {"rdBufSize", iocshArgInt};
static const iocshArg IP520DevCreateAllArg3 = {"wrBufSize", iocshArgInt};
static const iocshArg * const IP520DevCreateAllArgs[4] = {
    &IP520DevCreateAllArg0, &IP520DevCreateAllArg1,
    &IP520DevCreateAllArg2, &IP520DevCreateAllArg3 };
static const iocshFuncDef IP520DevCreateAllFuncDef =
    {"IP520DevCreateAll",4,IP520DevCreateAllArgs};
static void IP520DevCreateAllCallFunc(const iocshArgBuf *arg)
{
    IP520DevCreateAll(arg[0].sval, arg[1].sval, arg[2].ival, arg[3].ival);
}

/* IP520Config */
static const iocshArg IP520ConfigArg0 = {"devName",  iocshArgString};
static const iocshArg IP520ConfigArg1 = {"baud",     iocshArgInt};
static const iocshArg IP520ConfigArg2 = {"parity",   iocshArgString};
static const iocshArg IP520ConfigArg3 = {"stopbits", iocshArgInt};
static const iocshArg IP520ConfigArg4 = {"databits", iocshArgInt};
static const iocshArg IP520ConfigArg5 = {"flow",     iocshArgString};
static const iocshArg * const IP520ConfigArgs[6] = {&IP520ConfigArg0, &IP520ConfigArg1, &IP520ConfigArg2,
                                                    &IP520ConfigArg3, &IP520ConfigArg4, &IP520ConfigArg5};
static const iocshFuncDef IP520ConfigFuncDef = {"IP520Config",6,IP520ConfigArgs};
static void IP520ConfigCallFunc(const iocshArgBuf *arg)
{
    IP520Config(arg[0].sval, arg[1].ival, arg[2].sval[0], arg[3].ival, arg[4].ival, arg[5].sval[0]);
}

static void IP520Registrar(void) {
    iocshRegister(&IP520DrvFuncDef,IP520DrvCallFunc);
    iocshRegister(&IP520ReportFuncDef,IP520ReportCallFunc);
    iocshRegister(&IP520ModuleInitFuncDef,IP520ModuleInitCallFunc);
    iocshRegister(&IP520DevCreateFuncDef,IP520DevCreateCallFunc);
    iocshRegister(&IP520DevCreateAllFuncDef, IP520DevCreateAllCallFunc);
    iocshRegister(&IP520ConfigFuncDef,IP520ConfigCallFunc);
}
epicsExportRegistrar(IP520Registrar);
