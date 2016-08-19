/*******************************************************************************

Project:
    IndustryPack Driver Interface for EPICS

File:
    drvIpac.h

Description:
    IPAC Driver header file, defines the software interfaces:
	1. Upwards to the IPAC Module driver
	2. Downwards to the IPAC Carrier driver

Author:
    Andrew Johnson <anjohnson@iee.org>
Created:
    1 July 1995

Copyright (c) 1995-2007 Andrew Johnson

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


#ifndef INCdrvIpacH
#define INCdrvIpacH

#include "epicsTypes.h"
#include "errMdef.h"
#include "shareLib.h"


/* This type is defined here for backwards compatibility - the original drvIpac
 * used this type, and replacing it with an OSI equivalent would break any
 * IPAC carrier drivers that are outside of the drvIpac distribution.  The type
 * is #defined to use an explicitly-sized type from epicsTypes.h instead (we
 * can't use a typedef because that could fail on vxWorks).
 */
#ifndef ushort_t
#define ushort_t epicsUInt16
#endif


#ifdef __cplusplus
extern "C" {
#endif


/* Error numbers */

#ifndef OK
#define OK 0
#endif

#ifndef M_ipac
#define M_ipac		(600 << 16)
#endif

#define S_IPAC_badTable     (M_ipac| 1) /*IPAC Carrier Table invalid*/
#define S_IPAC_tooMany      (M_ipac| 2) /*Too many IPAC carriers, table full*/
#define S_IPAC_badAddress   (M_ipac| 3) /*Bad IPAC carrier or slot number*/
#define S_IPAC_badDriver    (M_ipac| 4) /*Bad value from IPAC carrier driver*/
#define S_IPAC_noModule     (M_ipac| 5) /*No IP module installed*/
#define S_IPAC_noIpacId     (M_ipac| 6) /*IPAC/VITA4 identifier not found*/
#define S_IPAC_badCRC       (M_ipac| 7) /*IPAC CRC Check failed*/
#define S_IPAC_badModule    (M_ipac| 8) /*IPAC Manufacturer or model ID wrong*/
#define S_IPAC_notImplemented (M_ipac| 9) /*IPAC Driver command not available*/
#define S_IPAC_badVector   (M_ipac| 10) /*Bad interrupt vector*/
#define S_IPAC_vectorInUse (M_ipac| 11) /*Interrupt vector in use*/
#define S_IPAC_badIntLevel (M_ipac| 12) /*Bad interrupt level*/
#define S_IPAC_noMemory   (M_ipac | 13) /*Malloc failed*/


/* Maximum size of IP carrier report string */

#define IPAC_REPORT_LEN	256


/* Structure of the IPAC ID Prom, located in the pack ID space.
 */

typedef volatile struct {
    epicsUInt16 asciiI;
    epicsUInt16 asciiP;
    epicsUInt16 asciiA;
    epicsUInt16 asciiC;
    epicsUInt16 manufacturerId;
    epicsUInt16 modelId;
    epicsUInt16 revision;
    epicsUInt16 reserved;
    epicsUInt16 driverIdLow;
    epicsUInt16 driverIdHigh;
    epicsUInt16 bytesUsed;
    epicsUInt16 CRC;
    epicsUInt16 packSpecific[52];
} ipac_idProm_t;

typedef volatile struct {
    epicsUInt16 asciiVI;
    epicsUInt16 asciiTA;
    epicsUInt16 ascii4_;
    epicsUInt16 manufacturerIdHigh;
    epicsUInt16 manufacturerIdLow;
    epicsUInt16 modelId;
    epicsUInt16 revision;
    epicsUInt16 reserved;
    epicsUInt16 driverIdLow;
    epicsUInt16 driverIdHigh;
    epicsUInt16 flags;
    epicsUInt16 bytesUsed;
    epicsUInt16 CRC;
    epicsUInt16 packSpecific[51];
} ipac_idProm2_t;


/* These are the types of address space implemented in the IP
   specification.  Some IP modules only use the ID and IO spaces. */

#define IPAC_ADDR_SPACES 4

typedef enum {
    ipac_addrID = 0,	/* ID Prom space */
    ipac_addrIO = 1,	/* Registers etc */
    ipac_addrIO32 = 2,	/* Registers for 32-bit dual-slot */
    ipac_addrMem = 3	/* Memory space */
} ipac_addr_t;


/* The following are the possible commands to the carrier driver to 
   handle configuration for the IP modules.  Most carriers will only be 
   able to implement a subset of these commands.  Note that irqEnable 
   should call the vxWorks sysBusEnable routine if this is needed to 
   pass the carrier interrupts through to the CPU. The ipac_stat commands
   were added for the VIPC664, and provide a means for showing the current
   status of each module using LEDs provided for each slot. */

typedef enum {
    ipac_irqLevel0 = 0,	/* Disables interrupts */
    ipac_irqLevel1 = 1,	/* Lowest priority */
    ipac_irqLevel2 = 2,	
    ipac_irqLevel3 = 3,	
    ipac_irqLevel4 = 4,	
    ipac_irqLevel5 = 5,	
    ipac_irqLevel6 = 6,	/* Highest priority */
    ipac_irqLevel7 = 7,	/* Non-maskable, don't use */
    ipac_irqGetLevel,	/* Returns level set (or hard-coded) */
    ipac_irqEnable,	/* Required to use interrupts, sets statActive */
    ipac_irqDisable,	/* Not necessarily supported */
    ipac_irqPoll,	/* Returns interrupt state */
    ipac_irqSetEdge,	/* Sets edge-triggered interrupts */
    ipac_irqSetLevel,	/* Sets level-triggered (default) */
    ipac_irqClear,	/* Only needed if using edge-triggered */
    ipac_statUnused,	/* Empty/uninitialized (Red LED on) */
    ipac_statActive,	/* Slot in use (Green LED on) */
    ipac_slotReset	/* Reset slot (blocks until complete) */
} ipac_irqCmd_t;


/* This is a table which each IPAC carrier driver provides to allow
   it to be queried by the IPAC driver.  One table is required for
   each type of carrier.  The cPrivate pointer is returned by the
   carrier driver initialise routine, and passed to all of the other
   routines as a means of identification of the carrier board. */

typedef struct {
    char *carrierType;
			/* String containing carrier board type */
    epicsUInt16 numberSlots;
			/* Number of IPAC devices this carrier can hold */
    int (*initialise)(const char *cardParms, void **cPrivate, epicsUInt16 carrier);
			/* Initialise carrier and return *cPrivate */
    char *(*report)(void *cPrivate, epicsUInt16 slot);
			/* Return string giving status of this slot */
    void *(*baseAddr)(void *cPrivate, epicsUInt16 slot, ipac_addr_t space);
			/* Return base addresses for this slot */
    int (*irqCmd)(void *cPrivate, epicsUInt16 slot,
		epicsUInt16 irqNumber, ipac_irqCmd_t cmd);
			/* Interrupt manipulation */
    /* The remaining routines are optional. Some drvIpac routines may call
     * devLib if these routines are NULL or not provided. */
    int (*intConnect)(void *cPrivate, epicsUInt16 slot, epicsUInt16 vecNum,
		void (*routine)(int parameter), int parameter);
			/* Connect routine to interrupt vector */
    int (*moduleProbe)(void *cPrivate, epicsUInt16 slot);
			/* Return 0 if reading ID Prom would Bus Error */
} ipac_carrier_t;


/* Functions for startup and interactive use */

epicsShareFunc int ipacAddCarrier(ipac_carrier_t *pcarrier, const char *cardParams);
epicsShareFunc int ipacReport(int interest);
epicsShareFunc int ipacAddNullCarrier (void);
epicsShareFunc int ipacLatestCarrier(void);


/* Functions for use in IPAC carrier drivers */

epicsShareFunc int ipcCheckId(ipac_idProm_t *id);


/* Functions for use in IPAC module drivers */

epicsShareFunc int ipmCheck(int carrier, int slot);
epicsShareFunc int ipmValidate(int carrier, int slot,
		int manufacturerId, int modelId);
epicsShareFunc char *ipmReport(int carrier, int slot);
epicsShareFunc void *ipmBaseAddr(int carrier, int slot, ipac_addr_t space);
epicsShareFunc int ipmIrqCmd(int carrier, int slot, 
		int irqNumber, ipac_irqCmd_t cmd);
epicsShareFunc int ipmIntConnect(int carrier, int slot, int vector, 
		void (*routine)(int parameter), int parameter);


#ifdef __cplusplus
}
#endif

#endif /* INCipacH */

