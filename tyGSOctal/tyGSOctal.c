/**************************************************************************
 Header:        tyGSOctal.c

 Author:        Peregrine M. McGehee

 Description:   Sourcefile for GreenSpring Ip_Octal 232, 422, and 485
 serial I/O modules. This software is somewhat based on the HiDEOS
 device driver developed by Jim Kowalkowski of the Advanced Photon Source.
**************************************************************************
 
 USER-CALLABLE ROUTINES
 Most of the routines in this driver are accessible only through the I/O
 system.  Two routines, however, must be called directly: tyGSOctalDrv() to
 initialize the driver, and tyGSOctalDevCreate() to create devices.

 Before the driver can be used, it must be initialized by calling
 tyGSOctalDrv().
 This routine should be called exactly once, before any reads, writes, or
 calls to tyGSOctalModuleInit()/tyGSOctalDevCreate().
 
 Before a terminal can be used, it must be created using
 tyGSOctalModuleInit()/tyGSOctalDevCreate().
 Each port to be used should have exactly one device associated with it by
 calling this routine.

 IOCTL FUNCTIONS
 This driver responds to the same ioctl() codes as a normal tty driver; for
 more information, see the manual entry for tyLib.
 
 SEE ALSO
 tyLib
 
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

#include "ip_modules.h"     /* GreenSpring IP modules */
#include "scc2698.h"        /* SCC 2698 UART register map */
#include "tyGSOctal.h"      /* Device driver includes */
#include "drvIpac.h"        /* IP management (from drvIpac) */
#include "iocsh.h"
#include "epicsExport.h"

QUAD_TABLE *tyGSOctalModules;
int tyGSOctalMaxModules;
int tyGSOctalLastModule;

int tyGSOctalDebug = 0;
               
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
LOCAL int    tyGSOctalStartup(TY_GSOCTAL_DEV *);
LOCAL void   tyGSOctalBaudSet(TY_GSOCTAL_DEV *, int);
LOCAL void   tyGSOctalOptsSet(TY_GSOCTAL_DEV *pTyGSOctalDv, int opts);
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
    static  char    *fn_nm = "tyGSOctalDrv";
    
    /* check if driver already installed */

    if (tyGSOctalDrvNum > 0)
	return (OK);
    
    tyGSOctalMaxModules = maxModules;
    tyGSOctalLastModule = 0;
    tyGSOctalModules = (QUAD_TABLE *)calloc(maxModules, sizeof(QUAD_TABLE));

    if (!tyGSOctalModules) {
        logMsg("%s: Memory allocation failed!",
               (int)fn_nm,
               NULL,NULL,NULL,NULL,NULL);
        return (ERROR);
    }
    rebootHookAdd(tyGSOctalRebootHook);    


    tyGSOctalDrvNum = iosDrvInstall (tyGSOctalOpen,
                                (FUNCPTR) NULL,
                                tyGSOctalOpen,
				(FUNCPTR) NULL,
                                tyRead,
                                tyGSOctalWrite,
                                tyGSOctalIoctl);

    return (tyGSOctalDrvNum == ERROR ? ERROR : OK);
}

void tyGSOctalReport()
{
    QUAD_TABLE *qt;
    int i, n;
    TY_GSOCTAL_DEV *pty;

    for (n = 0; n < tyGSOctalLastModule; n++) {
        qt = &tyGSOctalModules[n];
        printf("qt=%p carrier=%d module=%d\n",
               qt, qt->carrier, qt->module);
        for (i=0; i < 8; i++) {
            pty = &qt->port[i];
            if (pty->created) {
                printf("port %d(%p)\t", i, pty);
                printf("qt:%p\t", pty->qt);
                printf("regs:%p chan:%p\n", pty->regs, pty->chan);
                printf("drvNum:%d\t", pty->tyDev.devHdr.drvNum);
                printf("%s\n",  pty->tyDev.devHdr.name);
            }
        }
    }
}

LOCAL int tyGSOctalRebootHook(int type)
{
    QUAD_TABLE *qt;
    int i, n;
    TY_GSOCTAL_DEV *pty;
    FAST int oldlevel;
	
    oldlevel = intLock();	/* disable all interrupts */

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
    intUnlock (oldlevel);
    return OK;
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
 *    idx = tyGSOctalModuleInit("232", 0x60, 0, 1);
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
    if (tyGSOctalDrvNum <= 0) {
	errnoSet (S_ioLib_NO_DRIVER);
	return (ERROR);
    }

    if (!moduleID || !type)
	return (ERROR);

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
        logMsg("%s: Unsupported module type: %s",
               (int)fn_nm, (int)type,
               NULL,NULL,NULL,NULL);
        return (ERROR);
    }

    /*
     * Validate the IP module location and type.
     */
    if ((status = ipmValidate(carrier, module, GREEN_SPRING_ID, modelID))
        != 0) {
        logMsg("%s: Unable to validate IP module\n",
               (int)fn_nm,
               NULL,NULL,NULL,NULL,NULL);
        logMsg("%s: carrier:%d module:%d modelID:%d\n",
                (int)fn_nm, carrier, module, modelID,
                NULL,NULL);
        
        switch(status) {
            case S_IPAC_badAddress:
                logMsg("%s: Bad carrier or module number\n",
                       (int)fn_nm,
                       NULL,NULL,NULL,NULL,NULL);
                break;
            case S_IPAC_noModule:
                logMsg("%s: No module installed\n",
                       (int)fn_nm,NULL,NULL,NULL,NULL,NULL);
                break;
            case S_IPAC_noIpacId:
                logMsg("%s: IPAC identifier not found\n",
                       (int)fn_nm,NULL,NULL,NULL,NULL,NULL);
                break;
            case S_IPAC_badCRC:
                logMsg("%s: CRC Check failed\n",
                       (int)fn_nm,NULL,NULL,NULL,NULL,NULL);
                break;
            case S_IPAC_badModule:
                logMsg("%s: Manufacturer or model IDs wrong\n",
                      (int)fn_nm,NULL,NULL,NULL,NULL,NULL);
                break;
            default:
                logMsg("%s: Bad error code: 0x%x\n",
                       (int)fn_nm, status,
                       NULL,NULL,NULL,NULL);
                break;
        }
        return (ERROR);
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
	uint16_t intNum = int_num;
	SCC2698 *r;
	SCC2698_CHAN *c;
	int block;
	
        if (tyGSOctalLastModule >= tyGSOctalMaxModules) {
            logMsg("%s: Maximum module count exceeded!",
                   (int)fn_nm,
                   NULL,NULL,NULL,NULL,NULL);
            return (ERROR);
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
	    logMsg("%s: No memory allocated for carrier %d slot %d",
		   (int)fn_nm, carrier, module,
		   NULL,NULL,NULL);
            return(ERROR);
	}
	if (vxMemProbe(addrMem, VX_WRITE, 2, (char *) &intNum) == ERROR) {
	    logMsg("%s: Bus Error writing interrupt vector to address %#x",
		   (int)fn_nm, (int) addrMem,
		   NULL,NULL,NULL,NULL);
            return(ERROR);
	}
	
        if (ipmIntConnect(carrier, module, int_num, 
	    		  tyGSOctalInt, tyGSOctalLastModule)) {
            logMsg("%s: Unable to connect ISR",
                   (int)fn_nm,
                   NULL,NULL,NULL,NULL,NULL);
            return(ERROR);
        }
        ipmIrqCmd(carrier, module, 0, ipac_irqEnable);
        ipmIrqCmd(carrier, module, 1, ipac_irqEnable);
	
	ipmIrqCmd(carrier, module, 0, ipac_statActive);
    }
  
    return (tyGSOctalLastModule++);
}

/******************************************************************************
 * tyGSOctalDevCreate - create a device for a serial port on an IP module
 *
 * This routine creates a device on a specified serial port.  Each port
 * to be used should have exactly one device associated with it by calling
 * this routine.
 *
 * For instance, to create the device "/tyGSOctal/0/1/3", with buffer 
 * sizes of 512 bytes, the proper calls would be:
 * .CS
 *    int idx, dev;
 *    idx = tyGSOctalModuleInit("232", 0x60, 0, 1);
 *    dev = tyGSOctalDevCreate ("/tyGSOctal/0/1/3", idx, 3, 512, 512);
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
    TY_GSOCTAL_DEV *pTyGSOctalDv;
    QUAD_TABLE *qt = tyGSOctalFindQT(moduleID);

    if (!name || !qt)
        return NULL;

    /* if this doesn't represent a valid port, don't do it */
    if (port < 0 || port > 7)
	return NULL;

    pTyGSOctalDv = &qt->port[port];

    /* if there is a device already on this channel, don't do it */
    if (pTyGSOctalDv->created)
	return NULL;
    
    /* initialize the ty descriptor */
    if (tyDevInit (&pTyGSOctalDv->tyDev, rdBufSize, wrtBufSize,
		   (FUNCPTR) tyGSOctalStartup) != OK)
	return NULL;
    
    /* initialize the channel hardware */
    tyGSOctalInitChannel(qt, port);

    /* mark the device as created, and add the device to the I/O system */
    pTyGSOctalDv->created = TRUE;

    if (iosDevAdd(&pTyGSOctalDv->tyDev.devHdr, name,
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
 *    int idx;
 *    idx = tyGSOctalModuleInit("232", 0x60, 0, 1);
 *    tyGSOctalDevCreateAll ("/tyGS/0/", idx, 512, 512);
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
    char name[256];

    if (!qt || !base)
        return ERROR;

    for (port=0; port < 8; port++) {
	TY_GSOCTAL_DEV *pTyGSOctalDv = &qt->port[port];
	
	/* if there is a device already on this channel, ignore it */
	if (pTyGSOctalDv->created) continue;

	/* initialize the ty descriptor */
	if (tyDevInit (&pTyGSOctalDv->tyDev, rdBufSize, wrtBufSize,
		       (FUNCPTR) tyGSOctalStartup) != OK)
	    return ERROR;

	/* initialize the channel hardware */
	tyGSOctalInitChannel(qt, port);

	/* mark the device as created, and give it to the I/O system */
	pTyGSOctalDv->created = TRUE;

	sprintf(name, "%s%d", base, port);

	if (iosDevAdd(&pTyGSOctalDv->tyDev.devHdr, name, tyGSOctalDrvNum) != OK)
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
    const char *      moduleID
    )
{
    int i;

    if (!moduleID)
	return NULL;

    for (i = 0; i < tyGSOctalLastModule; i++)
	if (strcmp(moduleID, tyGSOctalModules[i].moduleID) == 0)
	    return &tyGSOctalModules[i];

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
    TY_GSOCTAL_DEV *pTyGSOctalDv = &qt->port[port];
    int block = port/2;     /* 4 blocks per octal UART */
    FAST int oldlevel;	    /* current interrupt level mask */

    oldlevel = intLock ();	/* disable interrupts during init */

    pTyGSOctalDv->block = block;

    pTyGSOctalDv->imr = ((port%2 == 0) ? SCC_ISR_TXRDY_A : SCC_ISR_TXRDY_B);
  
    /* choose set 2 BRG */
    pTyGSOctalDv->regs->u.w.acr = 0x80;

    pTyGSOctalDv->chan->u.w.cr = 0x1a; /* disable trans/recv, reset pointer */
    pTyGSOctalDv->chan->u.w.cr = 0x20; /* reset recv */
    pTyGSOctalDv->chan->u.w.cr = 0x30; /* reset trans */
    pTyGSOctalDv->chan->u.w.cr = 0x40 ; /* reset error status */
    
/*
 * Set up the default port configuration:
 * 9600 baud, no parity, 1 stop bit, 8 bits per char, no flow control
 */
    tyGSOctalBaudSet(pTyGSOctalDv, 9600);
    tyGSOctalOptsSet(pTyGSOctalDv, CS8);

/*
 * enable everything, really only Rx interrupts
*/
    qt->imr[block] |= ((port%2) == 0 ? SCC_ISR_RXRDY_A : SCC_ISR_RXRDY_B); 
   
    pTyGSOctalDv->regs->u.w.imr = qt->imr[block]; /* enable RxRDY interrupt */
    pTyGSOctalDv->chan->u.w.cr = 0x05;            /* enable Tx,Rx */
    
    intUnlock (oldlevel);
}

/******************************************************************************
 *
 * tyGSOctalOpen - open file to UART
 *
 * NOMANUAL
 */
LOCAL int tyGSOctalOpen
    (
	TY_GSOCTAL_DEV *pTyGSOctalDv,
	const char * name,
	int          mode
    )
{
    return ((int) pTyGSOctalDv);
}


/******************************************************************************
 * tyGSOctalWrite - Outputs a specified number of characters on a serial port
 *
 * NOMANUAL
 */
LOCAL int tyGSOctalWrite
    (
	TY_GSOCTAL_DEV * pTyGSOctalDv,	/* device descriptor block */
	char *     write_bfr,           /* ptr to an output buffer */
	long             write_size	/* # bytes to write */
    )
{
    static char  *fn_nm = "tyGSOctalWrite";
    SCC2698_CHAN *chan = pTyGSOctalDv->chan;
    int nbytes;
    
    /*
     * verify that the device descriptor is valid
     */
    if ( !pTyGSOctalDv ) {
	logMsg( "%s: (%s) DEVICE DESCRIPTOR INVALID\n",
	        (int)fn_nm, (int)taskName( taskIdSelf() ),
		NULL,NULL,NULL,NULL );
        return (-1);
    } else {
        if (pTyGSOctalDv->mode == RS485)
            /* disable recv, 1000=assert RTSN (low) */
            chan->u.w.cr = 0x82;
        
        nbytes = tyWrite(&pTyGSOctalDv->tyDev, write_bfr, write_size);
        
        if (pTyGSOctalDv->mode == RS485)
        {
            /* make sure all data sent */
            while(!(chan->u.r.sr & 0x08)); /* TxEMT */
            /* enable recv, 1001=negate RTSN (high) */
            chan->u.w.cr = 0x91;
        }
        
        return nbytes;
    }
}

/******************************************************************************
 *
 * tyGSOctalSetmr - set mode registers
 *
 * NOMANUAL
 */

LOCAL void tyGSOctalSetmr(TY_GSOCTAL_DEV *pTyGSOctalDv, int mr1, int mr2) {
    SCC2698_CHAN *chan = pTyGSOctalDv->chan;
    SCC2698 *regs = pTyGSOctalDv->regs;
    QUAD_TABLE *qt = pTyGSOctalDv->qt;
    
    if (qt->modelID == GSIP_OCTAL485) {
	pTyGSOctalDv->mode = RS485;

	/* MPOa/b are Tx output enables, must be controlled by driver */
	mr1 &= 0x7f; /* no auto RxRTS */
	mr2 &= 0xcf; /* no CTS enable Tx */
    } else {
	pTyGSOctalDv->mode = RS232;
	/* MPOa/b are RTS outputs, may be controlled by UART */
    }
    regs->u.w.opcr = 0x80; /* MPPn = output, MPOa/b = RTSN */
    chan->u.w.cr = 0x10; /* point MR to MR1 */
    chan->u.w.mr = mr1;
    chan->u.w.mr = mr2;
    if (mr1 & 0x80) { /* Hardware flow control */
	chan->u.w.cr = 0x80;	/* Assert RTSN */
    }
}

/******************************************************************************
 *
 * tyGSOctalOptsSet - set channel serial options
 *
 * NOMANUAL
 */

LOCAL void tyGSOctalOptsSet(TY_GSOCTAL_DEV *pTyGSOctalDv, int opts)
{
    UCHAR mr1 = 0, mr2 = 0;
    
    switch (opts & CSIZE) {
	case CS5: break;
	case CS6: mr1|=0x01; break;
	case CS7: mr1|=0x02; break;
	default:
	case CS8: mr1|=0x03; break;
    }
    if (opts & STOPB) {
	mr2|=0x0f;
    } else {
	mr2|=0x07;
    }
    if (!(opts & PARENB)) {
	mr1|=0x10;
    }
    if (opts & PARODD) {
	mr1|=0x04;
    }
    if (!(opts & CLOCAL)) {
	mr1|=0x80;	/* Control RTS from RxFIFO */
	mr2|=0x10;	/* Enable Tx using CTS */
    }

    tyGSOctalSetmr(pTyGSOctalDv, mr1, mr2);
    pTyGSOctalDv->opts = opts & (CSIZE|STOPB|PARENB|PARODD|CLOCAL);
}

/******************************************************************************
 *
 * tyGSOctalBaudSet - set channel baud rate
 *
 * NOMANUAL
 */

LOCAL void tyGSOctalBaudSet(TY_GSOCTAL_DEV *pTyGSOctalDv, int baud)
{
    SCC2698_CHAN *chan = pTyGSOctalDv->chan;
    switch(baud)
    {	/* NB: ACR[7]=1 */
	case 1200:  chan->u.w.csr=0x66; break; 
	case 2400:  chan->u.w.csr=0x88; break; 
	case 4800:  chan->u.w.csr=0x99; break; 
	default:    baud=9600;
	case 9600:  chan->u.w.csr=0xbb; break; 
	case 19200: chan->u.w.csr=0xcc; break; 
	case 38400: chan->u.w.csr=0x22; break; 
    }
    pTyGSOctalDv->baud = baud;
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
    TY_GSOCTAL_DEV *pTyGSOctalDv,	/* device to control */
    int        request,		/* request code */
    int        arg		/* some argument */
    )
{
    STATUS status = 0;
    int oldlevel;

    switch (request)
    {
	case FIOBAUDRATE:
	case SIO_BAUD_SET:
	    oldlevel = intLock ();
	    tyGSOctalBaudSet(pTyGSOctalDv, arg);
	    intUnlock (oldlevel);
	    break;
	case SIO_BAUD_GET:
	    *(int *)arg = pTyGSOctalDv->baud;
	    break;
	case SIO_HW_OPTS_SET:
	    oldlevel = intLock ();
	    tyGSOctalOptsSet(pTyGSOctalDv, arg);
	    intUnlock (oldlevel);
	    break;
	case SIO_HW_OPTS_GET:
	    *(int *)arg = pTyGSOctalDv->opts;
	    break;
	default:
	    status = tyIoctl (&pTyGSOctalDv->tyDev, request, arg);
	    break;
    }

    return (status);
}

/******************************************************************************
 *
 * tyGSOctalConfig - special device control (old version)
 *
 * This routine sets the baud rate, parity, stop bits, word size, and
 * flow control for the specified port.
 *
 */
void tyGSOctalConfig (
    char *name,
    int baud,
    char parity,
    int stop,
    int bits,
    char flow
) {
    static  char    *fn_nm = "tyGSOctalConfig";
    
    int opts = 0;
    int oldlevel;
    TY_GSOCTAL_DEV *pTyGSOctalDv = (TY_GSOCTAL_DEV *) iosDevFind(name, NULL);

    if (!pTyGSOctalDv) {
	logMsg("%s: Device %s not found\n",
	       (int)fn_nm, (int)name, NULL,NULL,NULL,NULL);
	return;
    }

    switch (bits) {
	case 5: opts |= CS5; break;
	case 6: opts |= CS6; break;
	case 7: opts |= CS7; break;
	default:
	case 8: opts |= CS8; break;
    }

    if (stop == 2)                   opts |= STOPB;
    if (tolower(flow) != 'h')        opts |= CLOCAL;
    if (tolower(parity) == 'e')      opts |= PARENB;
    else if (tolower(parity) == 'o') opts |= PARENB | PARODD;

    oldlevel = intLock ();
    tyGSOctalOptsSet(pTyGSOctalDv, opts);
    tyGSOctalBaudSet(pTyGSOctalDv, baud);
    intUnlock (oldlevel);
}

/*****************************************************************************
 * tyGSOctalInt - interrupt level processing
 *
 * NOMANUAL
 */
void tyGSOctalInt
    (
    int idx
    )
{
    volatile unsigned char sr, isr;
    unsigned int spin;
    int i;
    int level;
    int vector;
    int block;
    char outChar;
    char inChar;
    QUAD_TABLE *pQt;
    TY_GSOCTAL_DEV *pTyGSOctalDv;
    SCC2698_CHAN *chan;
    SCC2698 *regs = NULL;

    pQt = &(tyGSOctalModules[idx]);
    
    level = ipmIrqCmd(pQt->carrier, pQt->module, 0, ipac_irqGetLevel);
    vector = sysBusIntAck(level);
  
    for (spin=0; spin < MAX_SPIN_TIME; spin++)
    {
    /*
     * check each port for work
     */ 
        for (i = 0; i < 8; i++)
        {
            pTyGSOctalDv = &(pQt->port[i]);
            if (!pTyGSOctalDv->created) continue;

            block = i/2;
            chan = pTyGSOctalDv->chan;
            regs = pTyGSOctalDv->regs;

            sr = chan->u.r.sr;

            /* Only examine the active interrupts */
            isr = regs->u.r.isr & pQt->imr[block];
            
            /* Channel B interrupt data is on the upper nibble */
            if ((i%2) == 1) isr >>= 4;
            
            if (isr & 0x02) /* a byte needs to be read */
            {
                inChar = chan->u.r.rhr;
                if (tyGSOctalDebug)
                    logMsg("%d/%dR%02x %02x\n", idx, i, inChar,
                           isr, NULL, NULL);

                if (tyIRd(&(pTyGSOctalDv->tyDev), inChar) != OK)
                    if (tyGSOctalDebug)
                        logMsg("tyIRd failed!\n",
                               NULL,NULL,NULL,NULL,NULL, NULL);
            }

            if (isr & 0x01) /* a byte needs to be sent */
            {
                if (tyITx(&(pTyGSOctalDv->tyDev), &(outChar)) == OK) {
                    if (tyGSOctalDebug)
                        logMsg("%d/%dT%02x %02x %lx = %d\n",
                               idx, i, outChar, isr,
                               (int)&(pTyGSOctalDv->tyDev.wrtState.busy),
                               pTyGSOctalDv->tyDev.wrtState.busy);
                    
                    chan->u.w.thr = outChar;
                }
                else {
                    /* deactivate Tx INT and disable Tx INT */
                    pQt->imr[pTyGSOctalDv->block] &=
                        ~pTyGSOctalDv->imr;
                    regs->u.w.imr = pQt->imr[pTyGSOctalDv->block];
                    
                    if (tyGSOctalDebug)
                        logMsg("TxInt disabled: %d/%d isr=%02x\n",
                               idx, i, isr,
                               NULL, NULL, NULL);
                    
                }
            }
            
            if (sr & 0xf0) /* error condition present */
            {
                if (tyGSOctalDebug)
                    logMsg("%d/%dE% 02x\n",
                       idx, i,
                       sr, NULL, NULL, NULL);
                       
                /* reset error status */
                chan->u.w.cr = 0x40;
            }
        }
    }
    if (regs)
        isr = regs->u.r.isr;    /* Flush last write cycle (PowerPC) */
}

/******************************************************************************
 *
 * tyGSOctalStartup - transmitter startup routine
 *
 * Call interrupt level character output routine.
*/
LOCAL int tyGSOctalStartup
    (
    TY_GSOCTAL_DEV *pTyGSOctalDv 		/* ty device to start up */
    )
{
    static  char    *fn_nm = "tyGSOctalStartup";
    char outChar;
    QUAD_TABLE *qt = pTyGSOctalDv->qt;
    SCC2698 *regs = pTyGSOctalDv->regs;
    SCC2698_CHAN *chan = pTyGSOctalDv->chan;
    int block = pTyGSOctalDv->block;

    if (tyITx (&pTyGSOctalDv->tyDev, &outChar) == OK) {
        if (chan->u.r.sr & 0x04)
            chan->u.w.thr = outChar;
        
        qt->imr[block] |= pTyGSOctalDv->imr; /* activate Tx interrupt */
        regs->u.w.imr = qt->imr[block]; /* enable Tx interrupt */
    }
    else
        logMsg("%s: tyITX ERROR, sr=%02x",
               (int)fn_nm, chan->u.r.sr,
               NULL, NULL, NULL, NULL);

    return (0);
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
    iocshRegister(&tyGSOctalConfigFuncDef,tyGSOctalConfigCallFunc);
}
epicsExportRegistrar(tyGSOctalRegistrar);
