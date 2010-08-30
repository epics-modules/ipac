/*******************************************************************************

Project:
    Hytec 8002 Carrier Driver for EPICS

File:
    drvHy8002.c

Description:
    IPAC Carrier Driver for the Hytec 8002 IndustryPack Carrier VME board, 
    it provides the interface between IPAC driver and the hardware.  
    This carrier is 6U high and can support VME Extended mode addresses. 
    The carrier support 4 sites of IP cards. It can be configured to use 
    any of the 7 interrupt levels (1 ~ 7). It has hotswap capability but 
    during the past, it was experienced not too be much useful in 
    practical. As a result, this version takes this part out of the loop.

Author:
    Oringinal code framework:   Andrew Johnson <anjohnson@iee.org>,
    First version developer:	Walter Scott (aka Scotty), HyTec Electronics Ltd,
    Second version developer:	Jim Chen, HyTec Electronics Ltd     

Created:
	5 July 1995 code base drvIpMv162.c
    20 November 2002 first version.
    24 May 2010 This new version.
Version:
    $Id: drvHy8002.c 2.0 2010-05-24 13:00$

Original copyright (c) 1995-2003 Andrew Johnson, APS - Argonne National Laboratories, USA;
Subsequent modifications by Hytec Electronics Ltd, UK.

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
	5 July 1995	Andrew Johnson	Code base drvIpMv162.c
    20 November 2002 Walter Scott   First release version with hotswap
    24 May 2010  Jim Chen       Second version without hotswap
    07 July 2010 Jim Chen       1.Removed VxWorks dependence
                                2.Fixed a couple of bugs in arguments parsing routine. 
                                3.Added comments. 
                                4.Changed Hy8002CarrierInfo to ipacHy8002CarrierInfo for consistency
                                5.ipacAddHy8002 now returns latest carrier number if successful
                                6.Tidy up the return value for success and errors
    20 August 2010 Jim Chen     Modified checkprom routine to check both configuration ROM space and GreenSpring spaces.
								Before this driver only checks the ID and model in GreenSpring space
								and other version of 8002 drivers check only in configuration ROM.
    24 August 2010 Jim Chen     Added interrupt connection routine. This used to be missing so that 
								all IP module drivers have to use devLib devConnectInterruptVME directly which 
								makes the drivers hardware dependent. 
*******************************************************************************/

/*ANSI standard*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*EPICS specific*/
#include <devLib.h>
#include <drvIpac.h>
#include "epicsExport.h"

#include "iocsh.h"

/* Hytec IDs */
#define HYTECID    0x8003
#define PROM_MODEL 0x8002
#define PROM_MODEL_8003 0x8003
#define MANUFACTURER_HYTEC	0x80
#define HYTEC_PROM_MODEL	0x82
#define HYTEC_PROM_MODEL_8003	0x83

/* define individual bits in the carrier board's CSR register*/
#define CSR_RESET   0x0001
#define CSR_INTEN   0x0002
#define CSR_CLKSEL  0x0020
#define CSR_MEMMODE 0x0040
#define CSR_BADDSEL 0x0080
#define CSR_INTRELS 0x0200
#define CSR_CD32    0x4000
#define CSR_AB32    0x8000

#define CSR_INTSELSHIFT 2
#define CSR_IPMEMSHIFT  7
#define CSROFF     ~(CSR_INTEN)

/* Characteristics of the card */
#define NUMIPSLOTS 4                            /* number of IP slot */
#define IP_MEM_SIZE 0x0100                      /* the memory size reserved for an IP module (A16) */
#define ONEMB 0x100000                          /* one MB : reserve so much space for IP RAM (A32) */
#define ERR -1
#define VME_MEM_SIZE 0xA0                       /* the size of the memory to register for this carrier board.
                                                Don't make this too big or it will interfere with the
                                                memory space of the IP cards. */

/* prototype define */
typedef unsigned short word;
typedef unsigned int longword;

/* private structure used to keep track of a carrier card.
 We also keep a list of all Hy8002 carriers (carlist)
 for Hotswap support (scanning) */
typedef struct privinfo{
  struct privinfo* next;
  word vmeslotnum;                              /* vme slot */
  word carrier;                                 /* carrier number */
  word IPintlevel;                              /* interrupt level */
  int baseadr;                                  /* base address */
  int model;                                    /* carrier model, 8002/8003 */
  int clock;                                    /* carrier clock frequency */
  int intrels;                                  /* interrupt release on acknowledgement or on register read */
  word ipmemmode;                               /* 1,2,4 or 8 MB RAM per IP slot */
  word isgeomem;                                /* the card uses geographical IP card addressing. Please note, 
                                                due to the desing issue, Hytec 8002 cannot disable geographical addressing
                                                if VME64x crate is used. Yet if VME64 or VME is used, then a set of jumpers
                                                on board can be set for the base address of the carrier */
  /*these reflect the hardware registers*/
  word memoffs;                                 /* memory offset if non-geographical addressing is used */
  word csrcb;                                   /* CSR register */
  word ipintsel;                                /* IP cards interrupt settings in CSR */
  int ipadresses[NUMIPSLOTS][IPAC_ADDR_SPACES]; /* address mapping */
}privinfo;


/************GLOBAL VARIABLES********************/
static privinfo* carlist=NULL;
static char* charid="drvHy8002";

/*these are all offsets from the A16 base address*/
#define CARR_IPSTAT  0x00
#define CARR_MEMOFF  0x04                       /* added by JC 08-04-10 for memory offset register */
#define CARR_CSR     0x08
#define CARR_INTSEL  0x0C
#define CARR_HOTSWAP 0x10

#define CARR_IDENT   0x81
#define CARR_MANID   0x89
#define CARR_MODID   0x8B
#define CARR_REVN    0x8D
#define CARR_DRID1   0x91
#define CARR_DRID2   0x93
#define CARR_NUMB    0x95
#define CARR_CRC     0x97

/* function prototype */
static int regaddr(privinfo* pv);
static int checkVMEprom(unsigned int base);
static int scanparm(char* cp, int* vmeslotnum, int* IPintlevel, int* ipmem, int* ipclck, int* roak, int* domemreg, int* memoffs);
int ipacHy8002CarrierInfo(epicsUInt16 carrier);

/*******************************************************************************

Routine:
    initialise

Purpose:
    Registers a new Hy8002 with addresses and interrupt given by cardParams

Description:
    Reads the parameter string and set up the 8002 registers for initialisation.

Parameters:
    *cp: The parameter string please refer to ipacAddHy8002 routine.   
    **cPrivate: private info structure
    carrier: the latest added carrier number

Returns:
    OK(0): if successful 
    Error code otherwise.

*/
static int initialise(const char *cp, void **cPrivate, epicsUInt16 carrier)
{
    int vmeslotnum, IPintlevel;
    unsigned int carbase;
    size_t ccbase;
    int res,ipmem,ipclck,roak,domemreg,memoffs;
    privinfo* pv;
    word csr;
    long status;
    /*begin*/

    res=scanparm(cp, &vmeslotnum, &IPintlevel,
	       &ipmem,&ipclck,&roak,&domemreg,&memoffs);
    if (res!=OK) return res;

    ccbase= (size_t)((vmeslotnum<<11)+(1<<10));
    res=devRegisterAddress(charid,atVMEA16,
			 ccbase,
			 VME_MEM_SIZE,(void*)(&carbase));
    if (res!=OK) 
        return S_IPAC_badAddress;

    /*see if this really is a HyTec 8002*/
    res = checkVMEprom(carbase);
    if (res!=OK){
        res=devUnregisterAddress(atVMEA16,ccbase,charid);
        return res;
    }

    pv=(privinfo*)calloc(1,sizeof(privinfo));
    if(pv==NULL) return S_IPAC_noMemory;

    /*determine the CSR*/
    csr=IPintlevel<<CSR_INTSELSHIFT;

    /* if (domemreg)csr|=CSR_BADDSEL; */ /* a bug, to use memory offset, need to set bit6, not 8. JC 08-04-10 */
    if (domemreg) csr|=CSR_MEMMODE;
    if (ipclck == 32) csr|=CSR_CLKSEL;                  /* this clock bit was missing before. JC 26-05-2010 */
    if (roak) csr|=CSR_INTRELS;                         /* this ROAK or RORA bit was also missing before. JC 26-05-2010 */

    switch (ipmem){
        case 1:
            break;
        case 2:
            csr|=(1<<CSR_IPMEMSHIFT);
            break;
        case 4:
            csr|=(2<<CSR_IPMEMSHIFT);
            break;
        case 8:
            csr|=(3<<CSR_IPMEMSHIFT);
            break;
        default:
            printf("%s: INTERNAL ERROR 1\n",charid);
            return S_IPAC_badAddress;
    }

    /*in the ipmem==2 with geographical addressing,
    vmeslotnum must be [0..15] */
    if (ipmem==2 && vmeslotnum>15){
        printf("%s: vmeslot number must be <16 when geographical addressing with 2MB IP RAM size",charid);
        return S_IPAC_badAddress;
    }
    if (ipmem>=4 && domemreg==0){
        printf("%s: geographical adressing is not supported with 4MB IP RAM size",charid);
        return S_IPAC_badAddress;
    }

    pv->next=carlist;carlist=pv;

    pv->vmeslotnum=vmeslotnum;
    pv->carrier=carrier;
    pv->IPintlevel=IPintlevel;
    pv->baseadr=carbase;
    pv->memoffs=memoffs;
    pv->clock=ipclck;
    pv->isgeomem=(domemreg==0);
    pv->csrcb=csr;
    pv->ipintsel=0;
    pv->ipmemmode=ipmem;

    *((word*)(pv->baseadr+CARR_CSR   )) =pv->csrcb;                 /* set csr register */
    if(!pv->isgeomem)                                               /* This part is missing before. added by JC 08-04-10 */
        *((word*)(pv->baseadr+CARR_MEMOFF   )) =pv->memoffs;        /* set memroy offset */

    /*register the IP memory space for this card*/
    res=regaddr(pv);
    if (res != OK) return res;

    status=devEnableInterruptLevelVME(IPintlevel);
    if(status) return S_IPAC_badIntLevel;

    *((word*)(pv->baseadr+CARR_INTSEL)) =pv->ipintsel;
    
    *cPrivate=(void*)pv;
    return OK;
}


/*******************************************************************************

Routine:
    report

Purpose:
    Returns a status string for the requested slot

Description:
    This routine reports the interrupt level of the carrier card and
    the specified IP card interrupt setting.

Returns:
    A static string containing the slot's status.

*/
static char* report(void *cPrivate, ushort_t slot){
  /* Return string with giving status of this slot
     static char* report(ushort_t carrier, ushort_t slot){*/
    /*begin*/
    privinfo *cp = (privinfo *)cPrivate;
    static char output[IPAC_REPORT_LEN];
    sprintf(output, "INT Level %d, INT0: %s, INT1: %s", 
	    cp->IPintlevel,
        (cp->ipintsel & (1 << slot)) ? "active" : "",
	    (cp->ipintsel & (1 << (slot+4))) ? "active" : "");
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

static void* baseAddr(void *cPrivate,
		      ushort_t slot,
		      ipac_addr_t space)
{
    privinfo* pv=(privinfo*)cPrivate;
    return (void*)pv->ipadresses[slot][space];
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

static int irqCmd(void *cPrivate, ushort_t slot,
		  ushort_t irqnum, ipac_irqCmd_t cmd)
{
    int retval=S_IPAC_notImplemented;
    privinfo* cp=(privinfo*)cPrivate;
    word ipstat,mymask;
    word dodump=0;
    /*begin*/
    /*irqnumber is 0 or 1.*/
    if (irqnum !=0 && irqnum!=1)return S_IPAC_notImplemented;

    /*is the IP card valid*/
    if (slot>3) return S_IPAC_badAddress;
  
    switch(cmd)
    {
        /*We don't allow the IP driver to set the carrier's int level.
        It's set for the carrier in the init string*/
        case ipac_irqLevel0:
        case ipac_irqLevel1:
        case ipac_irqLevel2:
        case ipac_irqLevel3:
        case ipac_irqLevel4:
        case ipac_irqLevel5:
        case ipac_irqLevel6:/* Highest priority */
        case ipac_irqLevel7:/* Non-maskable, don't use */
            break;
        case ipac_irqGetLevel:
            /* Returns level set (or hard-coded) */
            retval=cp->IPintlevel;
            break;
        case ipac_irqEnable:
            /* Required to use interrupts */
            if (irqnum==0)
                cp->ipintsel|=(1<<(slot));
            else
                cp->ipintsel|=(1<<(slot+4));

            cp->csrcb|=CSR_INTEN;dodump=1;
            retval=OK;
            break;
        case ipac_irqDisable:
            /* Not necessarily supported */
            cp->csrcb&=CSROFF;dodump=1;
            retval=OK;
            break;
        case ipac_irqPoll:
            /* Returns interrupt state */
            ipstat=*((word*)(cp->baseadr+CARR_IPSTAT));
            mymask=1<<(4+slot)|(1<<slot);
            retval=ipstat&mymask;
            break;
        case ipac_irqSetEdge: /* Sets edge-triggered interrupts */
        case ipac_irqSetLevel:/* Sets level-triggered (default) */
        case ipac_irqClear:   /* Only needed if using edge-triggered */
            break;
        default:
            break;
    }/*switch*/

    if (dodump){
        *((word*)(cp->baseadr+CARR_CSR   )) =cp->csrcb;
        *((word*)(cp->baseadr+CARR_INTSEL)) =cp->ipintsel;
    }
    return retval;
}

/*******************************************************************************

Routine:
    intConnect

Purpose:
    Connect routine to interrupt vector

Description:
    This function connects the user's interrupt service routine to the vector by calling 
	the VME devLib interrupt connectoin routine. This devConnectInterruptVME is osi independent
	but is not hardware architecture independent. For other but rather than VME has to use 
	different devLib.
    
Returns:
    return 0 = OK,
    otherwise Error.

*/
static int
intConnect(void *cPrivate, epicsUInt16 slot, epicsUInt16 vecNum, void (*routine)(int parameter), int parameter)
{
  /*
    begin
  */
  return devConnectInterruptVME(vecNum, (void (*)()) routine, (void *) parameter);
}


/*THIS IS THE CARRIER JUMPTABLE */
ipac_carrier_t Hy8002={
    "Hytec VICB8002",
    4,
    initialise,
    report,
    baseAddr,
    irqCmd,
    intConnect
};


/*******************************************************************************

Routine:
    ipacAddHy8002

Purpose:
    shell command to be used in start up script

Description:
    The carrier board can be registered by using this function during 
    the system start up.

Parameters:
    The parameter "cardParams" is a string that should comprise 2 (the first two are mandatory) 
    to 6 parameters that are seperated by commas.   

    - first parameter is the VME slot number (decimal string)
    - second parameter is the VME interrupt level (decimal string)
    - third parameter is a name/value pair defines the IP memory size. 
      "IPMEM=" followed by a number "1", "2" ,"4" or "8" for 1M, 2M, 4M or 8M respectively
    - fourth parameter is a name/value pair defines the IP module clock. 
      "IPCLCK=" followed number either "8" or "32" for 8M or 32M respectively
    - fiveth parameter is a name/value pair defines the type of releasing interrupt. 
      "ROAK=1" means to release interrupt upon acknowledgement; "ROAK=0" means to release by ISR.
    - sixth parameter defines IP memory mapping base address offset when neither geographical 
      addressing nor jumpers are used. "MEMOFFS=128". Please refer to the user manual.

    Examples:
        IPAC1 = ipacAddHy8002("3,2") 
            
        where: 3 = vme slot number 3
               2 = interrupt level 2

        IPAC1 = ipacAddHy8002("3,2,IPMEM=1,IPCLCK=8,ROAK=1,MEMOFFS=2048)

        where: 3 -> vme slot number 3
               2 -> interrupt level 2
               IPMEM=1 -> IP memory space as 1M
               IPCLCK=8 -> IP clock is 8MHz
               ROAK=1 -> release the interrupt on acknowledgement
               MEMOFFS=2048 -> IP memory mapping base address offset when not using 
                   geographical addressing (Please see manual for detail usage)

        Please note, no space allowed in the parameter string

Returns:
    >=0 and < IPAC_MAX_CARRIERS (21): newly added carrier number
    > M_ipac(600 << 16): error code

*/
int ipacAddHy8002(const char *cardParams) {
    int rt;
    rt = ipacAddCarrier(&Hy8002, cardParams);                       /* add 8002 carrier */
    if(rt == OK)
        return ipacLatestCarrier();                                 /* If added OK, return the latest carrier number */
    else
    	return rt;                                                  /* Otherwise return error code  */
}

static const iocshArg Hy8002Arg0 = { "cardParams",iocshArgString};
static const iocshArg * const Hy8002Args[1] = {&Hy8002Arg0};
static const iocshFuncDef Hy8002FuncDef = {"ipacAddHy8002", 1, Hy8002Args};
static void Hy8002CallFunc(const iocshArgBuf *args) {
    ipacAddHy8002(args[0].sval);
}

static const iocshArg Hy8002InfoArg0 = {"carrier", iocshArgInt};
static const iocshArg * const Hy8002InfoArgs[1] =  {&Hy8002InfoArg0};
static const iocshFuncDef Hy8002InfoFuncDef = {"ipacHy8002CarrierInfo", 1, Hy8002InfoArgs};
static void Hy8002InfoCallFunc(const iocshArgBuf *args)
{
    ipacHy8002CarrierInfo(args[0].ival);
}

static void epicsShareAPI Hy8002Registrar(void) {
    iocshRegister(&Hy8002FuncDef, Hy8002CallFunc);
    iocshRegister(&Hy8002InfoFuncDef, Hy8002InfoCallFunc);
}

epicsExportRegistrar(Hy8002Registrar);




/***** Private function part *********/



/*******************************************************************************

Routine: internal function
    checkVMEprom

Purpose:
    check if the carrier is 8002 or 8003

Description:
    This function checks the carrier against a valid Hytec 8002 or 8003 card.  

Parameters:
    base: VME carrier base address.

Return:
    OK(0): if it is 8002 or 8003
    Error code otherwise.

*/

#define VME_CARR_MAN1  0x22B
#define VME_CARR_MAN2  0x22F

#define VME_CARR_MOD1  0x233
#define VME_CARR_MOD2  0x237

#define VME_CARR_REVN  0x243
#define VME_CARR_XIL1  0x247
#define VME_CARR_XIL2  0x24B
#define VME_CARR_XIL3  0x24F

#define VME_CARR_SER1  0x2CB
#define VME_CARR_SER2  0x2CF
#define VME_CARR_SER3  0x2D3
#define VME_CARR_SER4  0x2D7
#define VME_CARR_SER5  0x2DB
#define VME_CARR_SER6  0x2DF


static int checkVMEprom(unsigned int base)
{
    char* hytecstr=" (HyTec Electronics Ltd., Reading, UK)";
    int manid,ismodel,modelnum,ishytec;

	/* This checks the ID in Configuration ROM */
    manid=(*((char*)(base+VME_CARR_MAN1))<<8)+
        (*((char*)(base+VME_CARR_MAN2)));

/* bug fix PHO 29-1-02 
*  manid gets sign extended on a 167
*/
    manid &= 0xffff;
    ishytec=(manid==HYTECID);

	/* If ID in Configuration ROM fails, also check GreenSpring space */
	if (!ishytec)
	{
		manid = (int) (*((char *) (base + CARR_MANID)));
		manid &= 0xff;
		ishytec = (manid == MANUFACTURER_HYTEC);
	}

	/* This checks the model in Configuration ROM  */
    modelnum=((int)(*((char*)base+VME_CARR_MOD1))<<8)+
        (int)(*((char*)base+VME_CARR_MOD2));

/* bug fix PHO 29-1-02 as for manid
*/
    modelnum &= 0xffff;
    ismodel=((modelnum==PROM_MODEL) || (modelnum==PROM_MODEL_8003));

	/* If model in Configuration ROM fails, also check GreenSpring space */
	if(!ismodel)	
	{
		modelnum = (int) (*((char *) base + CARR_MODID));
		modelnum &= 0xff;
		ismodel=((modelnum==HYTEC_PROM_MODEL) || (modelnum==HYTEC_PROM_MODEL_8003));
	}

    if (!ishytec)
		printf("PROM UNSUPPORTED MANUFACTURER ID:%x;\nPROM EXPECTED 0x%08X, %s\n",
	       manid,HYTECID,hytecstr);
    if(!ismodel)
        printf("PROM UNSUPPORTED BOARD MODEL NUMBER:%x EXPECTED 0x%04hx or 0x%04hx\n",
	       modelnum, PROM_MODEL, PROM_MODEL_8003);
    return (ishytec && ismodel) ? OK : S_IPAC_badModule;
}


/*******************************************************************************

Routine: internal function
    scanparm

Purpose:
    parsing parameters

Description:
    This function parses the parameter passed by ipacAddHy8002 routine  
    to get the vme slot number, interrupt level, IP memory size, IP clcok
    setting, interrupt release type and memory offset for base address etc.

Parameters:
    Please refer to ipacAddHy8002 routine.

Return:
    OK(0): if successful
    Error code otherwise.

*/
static int scanparm(char* cp,
		    int* vmeslotnum,
		    int* IPintlevel,
		    int* ipmem,
		    int* ipclck,
            int* roak,
		    int* domemreg,
		    int* memoffs
		    )
{
    int vme=0, itr=0, ipm=0, ipc=0, ro=0, mem=0;
    int skip=0;
    char *pstart;
    int count;

    if (cp == NULL || strlen(cp) == 0) {
        return S_IPAC_badAddress;             
    }

    count = sscanf(cp, "%d,%d,%n", &vme, &itr, &skip);
    if (count != 2){   
        printf("********Number error. %s  num:%d\n",cp, count);
        return S_IPAC_badAddress;     
    }

    /*vme slot number parsing*/
    if (vme<0 || vme>21)
    {
        printf("********Slot error.\n");
        return S_IPAC_badAddress;
    }
    else
        *vmeslotnum = vme;

    /*Interrupt level parsing*/
    if (itr<0 || itr>7)
        return S_IPAC_badAddress;
    else
        *IPintlevel = itr;

    cp += skip;

    /*set defaults: 1M memeory, 8MHz clock, ROAK, do not use geographical addressing*/
    *ipmem=1;
    *ipclck=8;
    *roak=0;
    *domemreg=*memoffs=0;

    /*parsing IP memory size*/
    if((pstart=strstr(cp, "IPMEM=")) != NULL){
        if((1 != sscanf(pstart+6, "%d", &ipm)) || (ipm !=1 && ipm !=2 && ipm !=4 && ipm !=8)){
            return S_IPAC_badAddress;
        }
        *ipmem = ipm;    
    }
            
    /*parsing IP clock frequency*/
    if((pstart=strstr(cp, "IPCLCK=")) != NULL){
        if((1 != sscanf(pstart+7, "%d", &ipc)) || (ipc !=8 && ipc !=32)){
            return S_IPAC_badAddress;
        }
        *ipclck = ipc;    
    }
            
    /*parsing ROAK request*/
    if((pstart=strstr(cp, "ROAK=")) != NULL){
        if((1 != sscanf(pstart+5, "%d", &ro)) || (ro !=0 && ro !=1)){
            return S_IPAC_badAddress;
        }
        *roak = ro;    
    }
            
    /*parsing memory offset*/
    if((pstart=strstr(cp, "MEMOFFS=")) != NULL){
        if((1 != sscanf(pstart+8, "%d", &mem)) || mem <0 || mem >(1<<17)){
            return S_IPAC_badAddress;
        }
        *domemreg = 1;
        *memoffs = mem;
    }
            
    return OK;
}


/*******************************************************************************

Routine: internal function
    regaddr

Purpose:
    register base address

Description:
    It registers the IP carrier card memory..

Parameters:
    *pv: private structure

Return:
    OK(0): if successful. 
    Error code otherwise.

*/
static int regaddr(privinfo* pv){
    int* ipadr=(int*)(pv->ipadresses);
    int vmeslotnum=pv->vmeslotnum;
    int ip,ia,ipinc;
    int memspace;
    size_t basetmp=0;
    /*  int retval=(int)NULL;*/  /* Who would initialize like this??? */
    volatile longword retval = 0;
    int space;
    int moffs,status;
    /*begin*/
    for (ip=0;ip<NUMIPSLOTS;ip++)
        for (ia=0;ia<IPAC_ADDR_SPACES;ia++)*ipadr++ =0;
  
    /* init the ipac_addrIO and ipac_addrID spaces*/
    for(ip=0;ip<NUMIPSLOTS;ip++){
        basetmp=(size_t)((vmeslotnum<<11)+(ip<<8));
        status=devRegisterAddress(charid, atVMEA16,
			      basetmp,
			      IP_MEM_SIZE,
			      (void *) &retval);
        if (status!= OK){
            return S_IPAC_badAddress;
        }
        pv->ipadresses[ip][ipac_addrIO]=retval;
        pv->ipadresses[ip][ipac_addrID]=retval+0x80;
    }

    /*IP RAM space. This depends on the memory mode.
     See section 2.2.1 in the VICB8802 User's Manual.
     The 32 bit dual slot case is handled the same
     way as the 16 bit case but has larger memory space*/
    ipinc=1;
    for(ip=0;ip<NUMIPSLOTS;ip+=ipinc){
        space=ipac_addrMem;

        if (pv->isgeomem) {
        /*geographic addressing*/
            switch (pv->ipmemmode) {
                case 1:
	               basetmp=(size_t)((vmeslotnum<<22)|(ip<<20));
	               break;
                case 2:
	               basetmp=(size_t)((vmeslotnum<<23)|(ip<<21));
	               break;
                case 4:
	           /*shouldn't happen, catch this case in initialise()*/
	               break;
                case 8:
	               basetmp=(size_t)((vmeslotnum<<27)|(ip<<23));
	               break;
                default:
	               printf("INTERNAL ERROR: unknown ipmemmode %d\n",pv->ipmemmode);
	               break;
            }/*switch*/
        } else {
            /*use the memory base register*/
            moffs=(pv->memoffs>>6);
            switch (pv->ipmemmode) {
                case 1:
	               basetmp=(size_t)((moffs<<22)|(ip<<20));
	               break;
                case 2:
	               basetmp=(size_t)((moffs<<23)|(ip<<21));
	               break;
                case 4:
	               basetmp=(size_t)((moffs<<24)|(ip<<22));
	               break;
                case 8:
	               basetmp=(size_t)((moffs<<25)|(ip<<23));
	               break;
                default:
	               printf("INTERNAL ERROR: unknown ipmemmode %d\n",pv->ipmemmode);
	               break;
            }
        }
        /*now register the address*/
        if ((int)basetmp==0)
             return S_IPAC_badAddress;

        if (space==ipac_addrMem) memspace=ONEMB;
        else/*double wide space*/memspace=2*ONEMB;

        /* printf("%s: Try to map address=0x%x\n)",charid, basetmp); */

        status=devRegisterAddress(charid, atVMEA32,
			      basetmp,
			      memspace,
			      (void *)&(retval));
        if (status!=OK) {
            return S_IPAC_badAddress;
        }

        /* printf("%s: Mapped address=0x%x\n)",charid, retval); */
        pv->ipadresses[ip][space]=retval;
    } 
    return OK;
}

/*******************************************************************************

Routine: 
    ipacHy8002CarrierInfo

Purpose:
    print ROM info of the carrier card

Description:

    This is an. It registers the IP carrier card memory..

Parameters:
    carrier: carrier card number

Return:
    OK(0): if successful. 
    Error code otherwise .

*/
/*return carrier PROM info of specified carrier or all if argument carrier is 0xFFFF */
int ipacHy8002CarrierInfo(epicsUInt16 carrier)
{
    privinfo *cp=carlist; 
    char* hytecstr=" (HyTec Electronics Ltd., Reading, UK)";
    int manid,modelnum;

    if(carlist == NULL){
        printf("No carrier is registered.");
        return S_IPAC_badAddress;
    }
 
    while(cp!=NULL){																	/* loop all carriers */
        if((cp->carrier == carrier) || (carrier == 0xFFFF)){                            /* print when carrier matches specified or all */
            /*begin*/
            manid=((*((char*)(cp->baseadr+VME_CARR_MAN1))<<8)+
            (*((char*)(cp->baseadr+VME_CARR_MAN2)))) & 0xffff;

            printf("PROM manufacturer ID: 0x%02X. %s\n",manid, hytecstr);
        
            modelnum=(((int)(*((char*)cp->baseadr+VME_CARR_MOD1))<<8)+
                (int)(*((char*)cp->baseadr+VME_CARR_MOD2))) & 0xffff;

            printf("\nPROM model #: 0x%02hx, board rev. 0x%02hx\n",
	           modelnum,(int)(*((char*)(cp->baseadr+VME_CARR_REVN))));
            printf("PROM Xilinx rev.: 0x%02hx, 0x%02hx, 0x%02hx\n",
	           (int)(*((char*)(cp->baseadr+VME_CARR_XIL1))),
	           (int)(*((char*)(cp->baseadr+VME_CARR_XIL2))),
	           (int)(*((char*)(cp->baseadr+VME_CARR_XIL3))));

            printf("PROM Serial #: 0x%02hx 0x%02hx 0x%02hx 0x%02hx 0x%02hx 0x%02hx\n",
	           *((char*)(cp->baseadr+VME_CARR_SER1)),
	           *((char*)(cp->baseadr+VME_CARR_SER2)),
	           *((char*)(cp->baseadr+VME_CARR_SER3)),
	           *((char*)(cp->baseadr+VME_CARR_SER4)),
	           *((char*)(cp->baseadr+VME_CARR_SER5)),
	           *((char*)(cp->baseadr+VME_CARR_SER6)));

            if(cp->carrier == carrier) break;
        }
        cp=cp->next;
    }
    return OK;
}




