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
 
 Functions: 
 name           description        
 ----           -----------------------------------------------------------
 <funcName>     What this function does.

History:
 who            when       what
 ---            --------   ------------------------------------------------
 PMM            18/11/96   Original
 PMM            13/10/97   Recast as VxWorks device driver.
 ANJ            09/03/99   Merged into ipac <supporttop>, fixed warnings.
**************************************************************************/

/*
 * vxWorks includes
 */ 
#include <vxWorks.h>
#include <iv.h>
#include <intLib.h>
#include <errnoLib.h>
#include <msgQLib.h>
#include <rngLib.h>
#include <sysLib.h>
#include <tickLib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <semLib.h>
#include <logLib.h>
#include <taskLib.h>
#include <tyLib.h>

#include "ip_modules.h"     /* GreenSpring IP modules */
#include "scc2698.h"        /* SCC 2698 UART register map */
#include "tyGSOctal.h"      /* Device driver includes */
#include "drvIpac.h"        /* IP management (from drvIpac) */

QUAD_TABLE *tyGSOctalModules;
int tyGSOctalMaxModules;
int tyGSOctalLastModule;

int TYGSOCTAL_ISR_LOG = 0;
               
LOCAL int tyGSOctalDrvNum;  /* driver number assigned to this driver */

/*
 * forward declarations
 */
void         tyGSOctalInt(int);
LOCAL void   tyGSOctalInitChannel(QUAD_TABLE *, int);
LOCAL void   tyGSOctalRS232(TY_GSOCTAL_DEV *);
LOCAL void   tyGSOctalRS485(TY_GSOCTAL_DEV *);


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

    printf("allocating %d structures of %ld bytes\n", maxModules,
           sizeof(QUAD_TABLE));
    
    if (!(tyGSOctalModules =
          (QUAD_TABLE *)malloc(maxModules*sizeof(QUAD_TABLE)))) {
        logMsg("%s: Quad table allocation failed!",
               (int)fn_nm,
               NULL,NULL,NULL,NULL,NULL);
        return (ERROR);
    }
  
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
 *    idx = tyGSOctalModuleInit("GSIP_OCTAL232", 0x60, 0, 1);
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
    char *      type,           /* IP module type                   */
    int         int_num,        /* Interrupt number                 */
    int         carrier,        /* which carrier board [0-n]         */
    int         module          /* module number on carrier [0-m]   */
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

    /*
     * Check the IP module type.
     */
    if (!strcmp(type, "GSIP_OCTAL232"))
        modelID = GSIP_OCTAL232;
    else if (!strcmp(type, "GSIP_OCTAL422"))
        modelID = GSIP_OCTAL422;
    else if (!strcmp(type, "GSIP_OCTAL485"))
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
        logMsg("%s: Unable to validate IP module",
               (int)fn_nm,
               NULL,NULL,NULL,NULL,NULL);
        logMsg("%s: carrier:%d module:%d modelID:%d",
                (int)fn_nm, carrier, module, modelID,
                NULL,NULL);
        
        switch(status) {
            case S_IPAC_badAddress:
                logMsg("%s: Bad carrier or module number",
                       (int)fn_nm,
                       NULL,NULL,NULL,NULL,NULL);
                break;
            case S_IPAC_noModule:
                logMsg("%s: No module installed",
                       (int)fn_nm,NULL,NULL,NULL,NULL,NULL);
                break;
            case S_IPAC_noIpacId:
                logMsg("%s: IPAC identifier not found",
                       (int)fn_nm,NULL,NULL,NULL,NULL,NULL);
                break;
            case S_IPAC_badCRC:
                logMsg("%s: CRC Check failed",
                       (int)fn_nm,NULL,NULL,NULL,NULL,NULL);
                break;
            case S_IPAC_badModule:
                logMsg("%s: Manufacturer or model IDs wrong",
                      (int)fn_nm,NULL,NULL,NULL,NULL,NULL);
                break;
            default:
                logMsg("%s: Bad error code: 0x%x",
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
	uint16_t *addrMem;
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
        qt->carrier = carrier;
        qt->module = module;
        
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
        addrMem = (uint16_t *) ipmBaseAddr(carrier, module, ipac_addrMem);
	if (addrMem == NULL) {
	    logMsg("%s: No memory allocated for carrier %d slot %d",
		   (int)fn_nm, carrier, module,
		   NULL,NULL,NULL);
            return(ERROR);
	}
	*addrMem = int_num;
	
        if (ipmIntConnect(carrier, module, int_num, 
	    		  tyGSOctalInt, tyGSOctalLastModule)) {
            logMsg("%s: Unable to connect ISR",
                   (int)fn_nm,
                   NULL,NULL,NULL,NULL,NULL);
            return(ERROR);
        }
        ipmIrqCmd(carrier, module, 0, ipac_irqEnable);
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
 *    idx = tyGSOctalModuleInit("GSIP_OCTAL232", 0x60, 0, 1);
 *    dev = tyGSOctalDevCreate ("/tyGSOctal/0/1/3", idx, 3, 512, 512);
 * .CE
 *
 * RETURNS: Pointer to descriptor, or ERROR if the driver is not
 * installed, the channel is invalid, or the device already exists.
 *
 * SEE ALSO: tyGSOctalDrv()
*/
TY_GSOCTAL_DEV *tyGSOctalDevCreate
    (
    char *      name,           /* name to use for this device      */
    int         idx,            /* index into module table          */
    int         port,           /* port on module for this device [0-7] */
    int         rdBufSize,      /* read buffer size, in bytes       */
    int         wrtBufSize      /* write buffer size, in bytes      */
    )
{
    TY_GSOCTAL_DEV *pTyGSOctalDv;
    QUAD_TABLE *qt;

    /* if this doesn't represent a valid module, don't do it */
    if (idx < 0 || idx > tyGSOctalLastModule)
        return ((TY_GSOCTAL_DEV *)ERROR);
    
    /* if this doesn't represent a valid port, don't do it */
    if (port < 0 || port > 7)
	return ((TY_GSOCTAL_DEV *)ERROR);
    
    qt = &tyGSOctalModules[idx];
    pTyGSOctalDv = &qt->port[port];

    /* if there is a device already on this channel, don't do it */
    if (pTyGSOctalDv->created)
	return ((TY_GSOCTAL_DEV *)ERROR);
    
    /* initialize the ty descriptor */
    if (tyDevInit (&pTyGSOctalDv->tyDev, rdBufSize, wrtBufSize,
		   (FUNCPTR) tyGSOctalStartup) != OK)
    {
	return ((TY_GSOCTAL_DEV *)ERROR);
    }
    
    /* initialize the channel hardware */
    tyGSOctalInitChannel(qt, port);

    /* mark the device as created, and add the device to the I/O system */
    pTyGSOctalDv->created = TRUE;

    if (iosDevAdd(&pTyGSOctalDv->tyDev.devHdr, name,
                      tyGSOctalDrvNum) != OK)
    {
        return ((TY_GSOCTAL_DEV *)ERROR);
    }

    return (pTyGSOctalDv);
}

/******************************************************************************
 *
 * tyGSOctalInitChannel - initialize a single channel
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

    pTyGSOctalDv->port = port;
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
    tyGSOctalConfig(pTyGSOctalDv, 9600, 'N', 1, 8, 'N');

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
*/
int tyGSOctalOpen
    (
        TY_GSOCTAL_DEV *pTyGSOctalDv,
        char      *name,
        int        mode
    )
{
    return ((int) pTyGSOctalDv);
}


/******************************************************************************
 * tyGSOctalWrite - Outputs a specified number of characters on a serial port
 *
*/
int	
tyGSOctalWrite( TY_GSOCTAL_DEV *pTyGSOctalDv, /* device descriptor block */
               char *write_bfr,     /* ptr to an output buffer */
               long write_size )    /* # bytes to write */

{
    static  char    *fn_nm = "tyGSOctalWrite";
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
        
          /*
           *  Determine how much data exists in the write buffer
            
            tyIoctl( &(pTyGSOctalDv->tyDev), FIONWRITE, (int)&nbytes);
            */
        nbytes = tyWrite(&pTyGSOctalDv->tyDev, write_bfr, write_size);
            
        if (pTyGSOctalDv->mode == RS485)
        {
            /* make sure all data sent */
            while(!(chan->u.r.sr & 0x08)); /* TxEMT */
            /* enable recv, 1001=negate RTSN high */
            chan->u.w.cr = 0x91;
        }
        
        return nbytes;
    }
}

/******************************************************************************
 *
 * tyGSOctalIoctl - special device control
 *
 * This routine handles FIOBAUDRATE requests and passes all others to
 * tyIoctl().
 *
 * RETURNS: OK, or ERROR if invalid baud rate, or whatever tyIoctl() returns.
*/
STATUS tyGSOctalIoctl
    (
    TY_GSOCTAL_DEV *pTyGSOctalDv,		/* device to control */
    int        request,		/* request code */
    int        arg		/* some argument */
    )
{
    FAST STATUS  status;

    switch (request)
    {
	case FIOBAUDRATE:
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
 * flow control for the specified port
 *
*/
void tyGSOctalConfig
(
    TY_GSOCTAL_DEV *pTyGSOctalDv,
    unsigned int baud,
    char parity,
    int stop,
    int bits,
    char flow
    )
{
    SCC2698_CHAN *chan = pTyGSOctalDv->chan;
    QUAD_TABLE *qt = (QUAD_TABLE *)pTyGSOctalDv->qt;
    UCHAR mr1, mr2;
    

/*
 * mode registers
*/ 
    chan->u.w.cr = 0x10; /* point MR to MR1 */
    
    mr1 = 0x00; /*  RxRTS=No, RxINT=RxRDY, Error=char */
    mr2 = 0x00; /*  normal, TxRTS=No, CTS=No, stop-bit-length=0.563 */

    switch(parity) /* parity */
    {
        case 'E': break;            /* leave zero for even parity */
	case 'O': mr1|=0x04; break; /* odd parity */
	case 'N':
	default:  mr1|=0x10; break; /* no parity is also default */
    }

    switch(bits) /* per character */
    {
	case 5: break;	/* leave alone */ 
	case 6: mr1|=0x01; break;
	case 7: mr1|=0x02; break;
	case 8:
	default: mr1|=0x03; break; /* default is also 8 bits */
    }

    switch(stop) /* number of stop bits */
    {
	case 2:  mr2|=0x0f; break;
	case 1:
	default: mr2|=0x07; break;
    }

    switch(flow) /* set up flow control */
    {
	case 'H': mr1|=0x80; mr2|=0x10; break;
	case 'N':
	default: break; /* do nothing */
    }
    pTyGSOctalDv->mr1 = mr1;
    pTyGSOctalDv->mr2 = mr2;
    chan->u.w.mr=mr1;
    chan->u.w.mr=mr2;

    switch(baud) /* clock select */
    {
	case 1200: chan->u.w.csr=0x66; break; 
	case 2400: chan->u.w.csr=0x88; break; 
	case 4800: chan->u.w.csr=0x99; break; 
	case 9600: chan->u.w.csr=0xbb; break; 
	case 38400: chan->u.w.csr=0x22; break; 
	default:
	case 19200: chan->u.w.csr=0xcc; break; 
    }

    pTyGSOctalDv->opcr = 0x80;
  
    if(!ipmValidate(qt->carrier, qt->module,
                    GREEN_SPRING_ID, GSIP_OCTAL485))
        tyGSOctalRS485(pTyGSOctalDv);
    else
        tyGSOctalRS232(pTyGSOctalDv);
}

LOCAL void tyGSOctalRS232
    (
    TY_GSOCTAL_DEV *pTyGSOctalDv
    )
{
    SCC2698_CHAN *chan = pTyGSOctalDv->chan;
    SCC2698 *regs = pTyGSOctalDv->regs;
    UCHAR mr1 = pTyGSOctalDv->mr1;
    UCHAR mr2 = pTyGSOctalDv->mr2;
    
    pTyGSOctalDv->mode = RS232;
    
/*
 * allow RTS (MPOa) to be turned on/off automatically
*/
    regs->u.w.opcr = 0x87;  /* out,MPOb=RTSN,MPOa=FIFO full */
    
    chan->u.w.cr = 0x10 ; /* point MR to MR1 */
    mr1 |= 0x80;
    chan->u.w.mr = mr1;  /* use RxRTS (auto mode) */
    mr2 |= 0x20;
    chan->u.w.mr = mr2; /* use TxRTS (auto mode),CTS enable Tx */
    
    pTyGSOctalDv->mr1 = mr1;
    pTyGSOctalDv->mr2 = mr2;
}

LOCAL void tyGSOctalRS485
    (
    TY_GSOCTAL_DEV *pTyGSOctalDv
    )
{
    SCC2698_CHAN *chan = pTyGSOctalDv->chan;
    SCC2698 *regs = pTyGSOctalDv->regs;
    UCHAR mr1 = pTyGSOctalDv->mr1;
    UCHAR mr2 = pTyGSOctalDv->mr2;
    
    pTyGSOctalDv->mode = RS485;
    
/*
 * allow RTS (MPOa) to be turned on/off manually through control reg
*/
    regs->u.w.opcr = 0x80; /* out,MPOb=RTSN,MPOa=RTSN */

    chan->u.w.cr = 0x10; /* point MR to MR1 */
    mr1 &= 0x7f;
    chan->u.w.mr = mr1; /* no auto RxRTS */
    mr2 &= 0xcf;
    chan->u.w.mr = mr2; /* no auto TxRTS and no CTS enable Tx */

    pTyGSOctalDv->mr1 = mr1;
    pTyGSOctalDv->mr2 = mr2;
}

void tyGSOctalSetcr(TY_GSOCTAL_DEV *pTyGSOctalDv, unsigned char crval)
{
    SCC2698_CHAN *chan = pTyGSOctalDv->chan;
    chan->u.w.cr = crval;
}

void tyGSOctalSetopcr(TY_GSOCTAL_DEV *pTyGSOctalDv, unsigned char opcrval)
{
    SCC2698 *regs = pTyGSOctalDv->regs;
    regs->u.w.opcr = opcrval;
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
    SCC2698 *regs;

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
                if (TYGSOCTAL_ISR_LOG)
                    logMsg("%d/%dR%02x %02x\n", idx, i, inChar,
                           isr, NULL, NULL);

                if (tyIRd(&(pTyGSOctalDv->tyDev), inChar) != OK)
                    if (TYGSOCTAL_ISR_LOG)
                        logMsg("tyIRd failed!\n",
                               NULL,NULL,NULL,NULL,NULL, NULL);
            }

            if (isr & 0x01) /* a byte needs to be sent */
            {
                if (tyITx(&(pTyGSOctalDv->tyDev), &(outChar)) == OK) {
                    if (TYGSOCTAL_ISR_LOG)
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
                    
                    if (TYGSOCTAL_ISR_LOG)
                        logMsg("TxInt disabled: %d/%d isr=%02x\n",
                               idx, i, isr,
                               NULL, NULL, NULL);
                    
                }
            }
            
            if (sr & 0xf0) /* error condition present */
            {
                if (TYGSOCTAL_ISR_LOG)
                    logMsg("%d/%dE% 02x\n",
                       idx, i,
                       sr, NULL, NULL, NULL);
                       
                /* reset error status */
                chan->u.w.cr = 0x40;
            }
        }
    }
}

/******************************************************************************
 *
 * tyGSOctalStartup - transmitter startup routine
 *
 * Call interrupt level character output routine.
*/
int tyGSOctalStartup
    (
    TY_GSOCTAL_DEV *pTyGSOctalDv 		/* ty device to start up */
    )
{
    static  char    *fn_nm = "tyGSOctalStartup";
    char outChar;
    QUAD_TABLE *qt = (QUAD_TABLE *)pTyGSOctalDv->qt;
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
