/*
 * RTEMS/EPICS driver for Greensprings/SBS Octal UART
 *
 * Based on tyGSOctal.c by Peregrine M. McGehee as updated by Andrew Johnson.
 */
 
#include <epicsStdio.h>
#include <epicsStdlib.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <epicsExit.h>
#include <epicsInterrupt.h>
#include <epicsTypes.h>
#include <epicsExport.h>
#include <errlog.h>
#include <devLib.h>
#include <string.h>
#include <ctype.h>
#include <termios.h>
#include <errno.h>
#include <iocsh.h>
#include <rtems/system.h>
#include <rtems/io.h>
#include <rtems/error.h>
#include <rtems/termiostypes.h>

typedef void *TY_DEV;

#include "ip_modules.h"     /* GreenSpring IP modules */
#include "scc2698.h"        /* SCC 2698 UART register map */
#include "tyGSOctal.h"      /* Device driver includes */
#include "drvIpac.h"        /* IP management (from drvIpac) */

/*
 * 'Global' variables
 */
static QUAD_TABLE *tyGSOctalModules;
static int tyGSOctalMaxModules;
static int tyGSOctalLastModule;
rtems_device_major_number tyGsOctalMajor;

/*
 * Interrupt handler
 */
static void
tyGSOctalInt(int idx)
{
    epicsUInt8 sr, isr;
    unsigned int spin;
    int i;
    int block;
    QUAD_TABLE *pQt = &tyGSOctalModules[idx];
    SCC2698_CHAN *chan;
    SCC2698 *regs = NULL;

    pQt->interruptCount++;
    for (spin=0; spin < MAX_SPIN_TIME; spin++) {
        for (i = 0; i < 8; i++) {
            TY_GSOCTAL_DEV *pTyGSOctalDv = &(pQt->port[i]);
            if (!pTyGSOctalDv->created)
                continue;

            block = pTyGSOctalDv->block;
            chan = pTyGSOctalDv->chan;
            regs = pTyGSOctalDv->regs;

            sr = chan->u.r.sr;

            /* Only examine the active interrupts */
            isr = regs->u.r.isr & pQt->imr[block];
            
            /* Channel B interrupt status is in the upper nibble */
            if ((i%2) == 1)
                isr >>= 4;
            
            /*
             * If receiver is ready, read the character and push it up
             */
            if (isr & 0x02) {
                char inChar = chan->u.r.rhr;
                pTyGSOctalDv->readCharCount++;
                if (pTyGSOctalDv->tyDev)
                    rtems_termios_enqueue_raw_characters(pTyGSOctalDv->tyDev, &inChar, 1);
            }

            /*
             * If transmiter is ready tell termios that character has been sent.
             */
            if (isr & 0x1) {
                pQt->imr[block] &= ~pTyGSOctalDv->imr;
                regs->u.w.imr = pQt->imr[block];
                pTyGSOctalDv->writeCharCount++;
                if (pTyGSOctalDv->tyDev)
                    rtems_termios_dequeue_characters(pTyGSOctalDv->tyDev, 1);
            }

            /*
             * Reset errors
             */
            if (sr & 0xf0)
                chan->u.w.cr = 0x40;
        }
    }
    if (regs)
        isr = regs->u.r.isr;    /* Flush last write cycle (for PowerPC) */
}

/*
 * Map termios rate to clock-select code
 */
static int
csrForTermiosSpeedCode(int speedCode)
{
    switch(speedCode) {
    case B110:   return 0x1;
    case B150:   return 0x3;
    case B300:   return 0x4;
    case B600:   return 0x5;
    case B1200:  return 0x6;
    case B2400:  return 0x8;
    case B4800:  return 0x9;
    case B9600:  return 0xB;
    case B19200: return 0xC;
    case B38400: return 0x2;
    }
    return -1;
}

/*
 * TERMIOS callback routines
 */
static int
tyGsOctalCallbackWrite(int minor, const char *buf, int len)
{
    QUAD_TABLE *qt = &tyGSOctalModules[minor/8];
    TY_GSOCTAL_DEV *pTyGSOctalDv = &qt->port[minor%8];
    SCC2698_CHAN *chan = pTyGSOctalDv->chan;
    SCC2698 *regs = pTyGSOctalDv->regs;
    int block = pTyGSOctalDv->block;
	int key;

	key = epicsInterruptLock();
    chan->u.w.thr = *buf;
    qt->imr[block] |= pTyGSOctalDv->imr; /* activate Tx interrupt */
    regs->u.w.imr = qt->imr[block];      /* enable Tx interrupt */
	epicsInterruptUnlock(key);
	return 0;
}

static int
tyGsOctalCallbackSetAttributes(int minor, const struct termios *termios)
{
    QUAD_TABLE *qt = &tyGSOctalModules[minor/8];
    TY_GSOCTAL_DEV *pTyGSOctalDv = &qt->port[minor%8];
    SCC2698_CHAN *chan = pTyGSOctalDv->chan;
    SCC2698 *regs = pTyGSOctalDv->regs;
    int rxcsr, txcsr;
    int mr1, mr2;

    if (((rxcsr = csrForTermiosSpeedCode(cfgetispeed(termios))) < 0)
     || ((txcsr = csrForTermiosSpeedCode(cfgetospeed(termios))) < 0))
        return RTEMS_INVALID_NUMBER;
    chan->u.w.csr = (rxcsr << 4) | txcsr;
    
    switch (termios->c_cflag & CSIZE) {
    case CS5:     mr1 = 0x0; break;
    case CS6:     mr1 = 0x1; break;
    case CS7:     mr1 = 0x2; break;
    default:
    case CS8:     mr1 = 0x3; break;
    }
    if (termios->c_cflag & PARENB) {
        if (termios->c_cflag & PARODD)
            mr1 |= 0x04;
    }
    else {
        mr1 |= 0x10;
    }
    mr2 = (termios->c_cflag & CSTOPB) ? 0xF : 0x7;
    if (termios->c_cflag & CRTSCTS) {
        mr1 |=0x80;      /* Control RTS from RxFIFO */
        mr2 |=0x10;      /* Enable Tx using CTS */
    }
    regs->u.w.opcr = 0x80; /* MPPn = output, MPOa/b = RTSN */
    chan->u.w.cr = 0x10; /* point MR to MR1 */
    chan->u.w.mr = mr1;
    chan->u.w.mr = mr2;
    if (mr1 & 0x80) /* Hardware flow control */
        chan->u.w.cr = 0x80;    /* Assert RTSN */
    return RTEMS_SUCCESSFUL;
}

/*
 * RTEMS driver points
 */
static rtems_device_driver tyGsOctalOpen(rtems_device_major_number major,
                                         rtems_device_minor_number minor,
                                         void *arg) 
{
    rtems_libio_open_close_args_t *args = (rtems_libio_open_close_args_t *)arg;
    rtems_status_code sc;
    static const rtems_termios_callbacks callbacks = {
        NULL, /* firstOpen */
        NULL, /* lastClose */
        NULL, /* pollRead */
        tyGsOctalCallbackWrite,
        tyGsOctalCallbackSetAttributes,
        NULL, /* stopRemoteTx */
        NULL, /* startRemoteTx */
        TERMIOS_IRQ_DRIVEN
    };
    sc = rtems_termios_open(major, minor, arg, &callbacks);
    (tyGSOctalModules+(minor/8))->port[minor%8].tyDev = args->iop->data1;
    return sc;
}

static rtems_device_driver tyGsOctalClose(rtems_device_major_number major,
                                          rtems_device_minor_number minor,
                                          void *arg) 
{
    return rtems_termios_close(arg);
}

static rtems_device_driver tyGsOctalRead(rtems_device_major_number major,
                                         rtems_device_minor_number minor,
                                         void *arg) 
{
    return rtems_termios_read(arg);
}

static rtems_device_driver tyGsOctalWrite(rtems_device_major_number major,
                                          rtems_device_minor_number minor,
                                          void *arg) 
{
    return rtems_termios_write(arg);
}

static rtems_device_driver tyGsOctalControl(rtems_device_major_number major,
                                            rtems_device_minor_number minor,
                                            void *arg) 
{
    return rtems_termios_ioctl(arg);
}

/*
 * Shut things down on exit
 */
static void
tyGSOctalRebootHook(void *arg)
{
    QUAD_TABLE *qt;
    int i, n;
    TY_GSOCTAL_DEV *pty;
    int key;
        
    key = epicsInterruptLock();
    for (n = 0; n < tyGSOctalLastModule; n++) {
        qt = &tyGSOctalModules[n];
        for (i=0; i < 8; i++) {
            pty = &qt->port[i];
            if (pty->created) {
                pty->imr = 0;
                pty->regs->u.w.imr = 0;
            }
        ipmIrqCmd(qt->carrier, qt->module, 0, ipac_irqDisable);
        ipmIrqCmd(qt->carrier, qt->module, 1, ipac_irqDisable);
        
        ipmIrqCmd(qt->carrier, qt->module, 0, ipac_statUnused);
        }
    }
    epicsInterruptUnlock(key);
}

/******************************************************************************
 *
 * tyGSOctalDrv - initialize the driver
 *
 * This routine connects the driver with RTEMS
 *
 * This routine should be called exactly once, before any reads, writes, or
 * calls to tyGSOctalDevCreate().
 *
 * This routine takes as an argument the maximum number of IP modules
 * to support.
*/
int
tyGSOctalDrv(int maxModules)
{
    rtems_status_code sc;
    static rtems_driver_address_table tyGsOctalDriverTable = {
        NULL,  /* initialization */
        tyGsOctalOpen,
        tyGsOctalClose,
        tyGsOctalRead,
        tyGsOctalWrite,
        tyGsOctalControl
    };
    if (tyGsOctalMajor > 0)
        return 0;
    tyGSOctalMaxModules = maxModules;
    tyGSOctalLastModule = 0;
    tyGSOctalModules = (QUAD_TABLE *)calloc(maxModules, sizeof(QUAD_TABLE));
    if (!tyGSOctalModules) {
        printf("Memory allocation failed!");
        return -1;
    }
    epicsAtExit(tyGSOctalRebootHook, NULL);    

    sc = rtems_io_register_driver(0, &tyGsOctalDriverTable, &tyGsOctalMajor);
    if (sc != RTEMS_SUCCESSFUL) {
        printf("Can't register driver: %s\n", rtems_status_text(sc));
        return -1;
    }
    return 0;
}

void tyGSOctalReport()
{
    int i, n, header;

    header = 1;
    for (n = 0; n < tyGSOctalLastModule; n++) {
        QUAD_TABLE *qt = &tyGSOctalModules[n];
        printf("qt=%p carrier=%d module=%d interrupts:%lu\n",
                            qt, qt->carrier, qt->module, qt->interruptCount);
        for (i = 0; i < 8; i++) {
            TY_GSOCTAL_DEV *pTyGSOctalDv = &qt->port[i];
            if (pTyGSOctalDv->created) {
                if (header) {
                    printf("UART      IN       OUT\n");
                    header = 0;
                }
                printf("%3d %9lu %9lu\n", i, pTyGSOctalDv->readCharCount,
                                               pTyGSOctalDv->writeCharCount);
            }
        }
    }
}

/******************************************************************************
 * tyGSOctalModuleInit - initialize an IP module
 *
 * The routine initializes the specified IP module. Each module is
 * characterized by its model name, interrupt vector, carrier board
 * number, and module number on the board. No new setup is done if a
 * QUAD_TABLE entry already exists with the same carrier and module
 * numbers.
 *
 * For example:
 * .CS
 *    int idx;
 *    idx = tyGSOctalModuleInit("SBS232-1", "232", 0x60, 0, 1);
 * .CE
 *
 *
 * RETURNS: Index into module table, or -1 if the driver is not
 * installed, the channel is invalid, or the device already exists.
 *
 * SEE ALSO: tyGSOctalDrv()
*/
int tyGSOctalModuleInit
    (
    const char * moduleID,       /* IP module name           */
    const char * type,           /* IP module type           */
    int          int_num,        /* Interrupt number         */
    int          carrier,        /* which carrier board [0-n]         */
    int          module          /* module number on carrier [0-m]   */
    )
{
    static  char    *fn_nm = "tyGSOctalModuleInit";
    int modelID;
    int status;
    int i;
    QUAD_TABLE *qt;
    
    /*
     * Check for the driver being installed.
     */    
    if (tyGsOctalMajor == 0) {
        errno = ENODEV;
        return -1;
    }

    if (!moduleID || !type) {
        errno = EINVAL;
        return -1;
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
        errno = EINVAL;
        return -1;
    }

    /*
     * Validate the IP module location and type.
     */
    if ((status = ipmValidate(carrier, module, GREEN_SPRING_ID, modelID)) != 0) {
        printf("%s: Unable to validate IP module\n", fn_nm);
        printf("%s: carrier:%d module:%d modelID:%d\n", fn_nm, carrier, module, modelID);
        
        switch(status) {
            case S_IPAC_badAddress:
                printf("%s: Bad carrier or module number\n", fn_nm);
                break;
            case S_IPAC_noModule:
                printf("%s: No module installed\n", fn_nm);
                break;
            case S_IPAC_noIpacId:
                printf("%s: IPAC identifier not found\n", fn_nm);
                break;
            case S_IPAC_badCRC:
                printf("%s: CRC Check failed\n", fn_nm);
                break;
            case S_IPAC_badModule:
                printf("%s: Manufacturer or model IDs wrong\n", fn_nm);
                break;
            default:
                printf("%s: Bad error code: 0x%x\n", fn_nm, status);
                break;
        }
        errno = status;
        return -1;
    }

    /* See if the associated IP module has already been set up */
    for (i = 0; i < tyGSOctalLastModule; i++) {
        qt = &tyGSOctalModules[i];
        if (qt->carrier == carrier && qt->module == module) break;
    }
    
    /* Create a new quad table entry if not there */
    if (i >= tyGSOctalLastModule) {
        void *addrIO;
        char *addrMem;
        char *ID = malloc(strlen(moduleID) + 1);
        epicsUInt16 intNum = int_num;
        SCC2698 *r;
        SCC2698_CHAN *c;
        int block;
        
        if (tyGSOctalLastModule >= tyGSOctalMaxModules) {
            printf("%s: Maximum module count exceeded!", fn_nm);
            errno = ENOSPC;
            return -1;
        }
        qt = &tyGSOctalModules[tyGSOctalLastModule];
        qt->modelID = modelID;
        qt->carrier = carrier;
        qt->module = module;
        strcpy(ID, moduleID);
        qt->moduleID = ID;
        
        addrIO = ipmBaseAddr(carrier, module, ipac_addrIO);
        r = (SCC2698 *) addrIO;
        c = (SCC2698_CHAN *) addrIO;

        for (i = 0; i < 8; i++) {
            block = i/2;
            qt->port[i].created = 0;
            qt->port[i].qt = qt;
            qt->port[i].regs = &r[block];
            qt->port[i].chan = &c[i];
        }

        for (i = 0; i < 4; i++) qt->imr[i] = 0;
        
        /* set up the single interrupt vector */
        addrMem = (char *) ipmBaseAddr(carrier, module, ipac_addrMem);
        if (addrMem == NULL) {
            printf("%s: No IPAC memory allocated for carrier %d slot %d", fn_nm, carrier, module);
            return -1;
        }
        if (devWriteProbe(2, addrMem, &intNum) != 0) {
            printf("%s: Bus Error writing interrupt vector to address %#x",
                               fn_nm, (int) addrMem);
            return -1;
        }
        if (ipmIntConnect(carrier, module, int_num, 
                                      tyGSOctalInt, tyGSOctalLastModule)) {
            printf("%s: Unable to connect ISR", fn_nm);
            return -1;
        }
        ipmIrqCmd(carrier, module, 0, ipac_irqEnable);
        ipmIrqCmd(carrier, module, 1, ipac_irqEnable);
        
        ipmIrqCmd(carrier, module, 0, ipac_statActive);
    }
  
    return (tyGSOctalLastModule++);
}

/*
 * Find a named module quadtable
 */
static QUAD_TABLE *
tyGSOctalFindQT(const char *moduleID)
{
    int i;

    if (!moduleID)
        return NULL;
    for (i = 0; i < tyGSOctalLastModule; i++)
        if (strcmp(moduleID, tyGSOctalModules[i].moduleID) == 0)
            return &tyGSOctalModules[i];
    return NULL;
}

/*
 * Create a device for a serial port on an IP module
 *
 * This routine creates a device on a specified serial port.  Each port
 * to be used should have exactly one device associated with it by calling
 * this routine.
 *
 * Calling this routine with a negative port number creates all eight devices.
*/
const char *
tyGSOctalDevCreate (
    char *       name,           /* name or prefix for this device       */
    const char * moduleID,       /* IP module name                       */
    int          port,           /* port on module for this device [0-7] */
    int          rdBufSize,      /* read buffer size, in bytes           */
    int          wrtBufSize      /* write buffer size, in bytes          */
    )
{
    TY_GSOCTAL_DEV *pTyGSOctalDv;
    QUAD_TABLE *qt = tyGSOctalFindQT(moduleID);
    int block;
    int minor;
    rtems_status_code sc;
    int key;
    struct termios termios;

    if (port < 0) {
        char nameBuf[256];
        for (port = 0 ; port < 8 ; port++) {
            epicsSnprintf(nameBuf, sizeof nameBuf, "%s%d", name, port);
            tyGSOctalDevCreate(nameBuf, moduleID, port, rdBufSize, wrtBufSize);
        }
        return name;
    }

    /* if this doesn't represent a valid port, don't do it */
    if (!name || !qt || port < 0 || port > 7)
        return NULL;
    minor = ((qt - tyGSOctalModules) * 8) + port;
    block = port / 2;
    pTyGSOctalDv = &qt->port[port];

    /* if there is a device already on this channel, don't do it */
    if (pTyGSOctalDv->created)
        return NULL;
    
    /* initialize the channel hardware (9600-8N1) */
    termios.c_iflag = 0;
    termios.c_oflag = 0;
    termios.c_lflag = 0;
    termios.c_cflag = CS8;
    cfsetispeed(&termios, B9600);
    cfsetospeed(&termios, B9600);
    tyGsOctalCallbackSetAttributes(minor, &termios);

    key = epicsInterruptLock(); /* disable interrupts during init */
    pTyGSOctalDv->block = block;
    pTyGSOctalDv->imr = ((port%2 == 0) ? SCC_ISR_TXRDY_A : SCC_ISR_TXRDY_B);
    pTyGSOctalDv->regs->u.w.acr = 0x80; /* choose set 2 BRG */
    pTyGSOctalDv->chan->u.w.cr = 0x1a; /* disable trans/recv, reset pointer */
    pTyGSOctalDv->chan->u.w.cr = 0x20; /* reset recv */
    pTyGSOctalDv->chan->u.w.cr = 0x30; /* reset trans */
    pTyGSOctalDv->chan->u.w.cr = 0x40 ; /* reset error status */
    qt->imr[block] |= ((port%2) == 0 ? SCC_ISR_RXRDY_A : SCC_ISR_RXRDY_B); 
    pTyGSOctalDv->regs->u.w.imr = qt->imr[block]; /* enable RxRDY interrupt */
    pTyGSOctalDv->chan->u.w.cr = 0x05;            /* enable Tx,Rx */
    epicsInterruptUnlock(key);

    /* mark the device as created, and add it to the I/O system */
    pTyGSOctalDv->created = 1;
    sc = rtems_io_register_name(name, tyGsOctalMajor, minor);
    if (sc != RTEMS_SUCCESSFUL) {
        printf("rtems_io_register_name(\"%s\", %d, %d) failed: %s\n",
                    name, (int)tyGsOctalMajor, minor, rtems_status_text(sc));
        return NULL;
    }
    return name;
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

static void tyGSOctalRegistrar(void) {
    iocshRegister(&tyGSOctalDrvFuncDef,tyGSOctalDrvCallFunc);
    iocshRegister(&tyGSOctalReportFuncDef,tyGSOctalReportCallFunc);
    iocshRegister(&tyGSOctalModuleInitFuncDef,tyGSOctalModuleInitCallFunc);
    iocshRegister(&tyGSOctalDevCreateFuncDef,tyGSOctalDevCreateCallFunc);
}
epicsExportRegistrar(tyGSOctalRegistrar);
