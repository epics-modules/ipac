/*******************************************************************************

Project:
    Gemini/UKIRT CAN Bus Driver for EPICS

File:
    drvIpac.c

Description:
    IPAC Driver, provides a standard  interface between IPAC Module
    drivers and the IPAC Carrier drivers.

Author:
    Andrew Johnson
Created:
    3 July 1995
Version:
    $Id: drvIpac.c,v 1.4 1999-07-28 20:37:51 anj Exp $

(c) 1995 Royal Greenwich Observatory

*******************************************************************************/


#ifndef NO_EPICS
#  include <drvSup.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vxLib.h>
#include <intLib.h>
#include <iv.h>
#include "drvIpac.h"


#define IPAC_MAX_CARRIERS 21


/* Private carrier data structures */
struct carrierInfo {
    ipac_carrier_t *driver;
    void *cPrivate;
};

LOCAL struct {
    ushort_t number;
    struct carrierInfo *info[IPAC_MAX_CARRIERS];
} carriers = {
    0
};


/* Null carrier stuff */

LOCAL ipac_carrier_t nullCarrier = {
    "Null carrier (place holder)",
    0,			/* No slots */
    NULL, NULL, NULL, NULL, NULL
};

LOCAL struct carrierInfo nullInfo = {
    &nullCarrier,
    NULL
};


#ifndef NO_EPICS

/* EPICS Driver Support Entry Table */

struct drvet drvIpac = {
    3, 
    (DRVSUPFUN) ipacReport, 
    (DRVSUPFUN) ipacInitialise,
    NULL
};


#endif


/*******************************************************************************

Routine:
    ipacAddCarrier

Purpose:
    Used to register a carrier board & carrier driver with the IPAC driver.

Description:
    Usually called from the vxWorks (EPICS) startup script.  Some types of 
    carrier may need additional initilisation before or after registering,
    but the card parameter string should be sufficient for most carriers.  
    Note that only the carrier initialise routine is called at this stage.  
    The order in which carriers are registered with this routine specifies 
    the carrier number which they will be allocated, starting from zero.

    Checks that the carrier descriptor table looks sensible, then calls the
    initialise routine with the given card parameters, and saves the carrier 
    private pointer and carrier table address.  The card number allows the 
    same descriptor to be used for all carriers of the same type.

    It may be necessary to remove a carrier temporarily from a system in 
    some circumstances without wanting to have to change the carrier number 
    allocated to higher numbered carriers.  To allow this, it is legal to 
    call this routine with a NULL (zero) carrier table address, which 
    switches in the null carrier table instead.

Returns:
    0 = OK,
    S_IPAC_tooMany = Carrier Info Table full,
    S_IPAC_badTable = Carrier Table invalid.

Example:
    ipacAddCarrier(&vipc310, "0x6000");

*/

int ipacAddCarrier (
    ipac_carrier_t *pcarrierTable,
    char *cardParams
) {
    void *cPrivate;
    int status;

    if (carriers.number >= IPAC_MAX_CARRIERS) {
	printf("ipacAddCarrier: Too many carriers registered.\n");
	return S_IPAC_tooMany;
    }

    if (pcarrierTable == NULL) {
	carriers.info[carriers.number] = &nullInfo;
	carriers.number++;
	return OK;
    }

    if (pcarrierTable->numberSlots == 0 ||
	pcarrierTable->initialise == NULL ||
	pcarrierTable->baseAddr == NULL ||
	pcarrierTable->irqCmd == NULL) {
	printf("ipacAddCarrier: Bad carrier table (arg 1).\n");
	return S_IPAC_badTable;
    }

    status = pcarrierTable->initialise(cardParams, &cPrivate, carriers.number);
    if (status) {
	printf("ipacAddCarrier: %s driver returned an error.\n", 
		pcarrierTable->carrierType);
	return status;
    }

    carriers.info[carriers.number] = malloc(sizeof (struct carrierInfo));
    carriers.info[carriers.number]->driver = pcarrierTable;
    carriers.info[carriers.number]->cPrivate = cPrivate;
    carriers.number++;

    return OK;
}


/*******************************************************************************

Routine:
    ipmCheck

Function:
    Check on presence of an IPAC module at the given carrier & slot number.

Description:
    Does a quick check to make sure the carrier and slot numbers are legal, 
    probes the IDprom space to ensure an IPAC is installed, and checks that 
    the IDprom starts with the "IPAC" identifier.

Returns:
    0 = OK,
    S_IPAC_badAddress = Bad carrier or slot number,
    S_IPAC_noModule = No module installed,
    S_IPAC_noIpacId = "IPAC" identifier not found

*/

int ipmCheck (
    ushort_t carrier, 
    ushort_t slot
) {
    ipac_idProm_t *id;
    ushort_t dummy;

    if (carrier >= carriers.number ||
	slot >= carriers.info[carrier]->driver->numberSlots) {
	return S_IPAC_badAddress;
    }

    id = (ipac_idProm_t *) ipmBaseAddr(carrier, slot, ipac_addrID);
    if (id == NULL) {
	return S_IPAC_badDriver;
    }

    if (vxMemProbe((void *)&id->asciiI, READ, sizeof(dummy), (char *)&dummy)) {
	return S_IPAC_noModule;
    }
    
    /*
     * The following code is deliberately de-optimized to fix a problem with
     * a particular GPIB module which can't handle the back-to-back accesses
     * that the compiler generates if you combine the conditions in one if.
     */
    
    if ((id->asciiI & 0xff) != 'I') {
	return S_IPAC_noIpacId;
    }
    if ((id->asciiP & 0xff) != 'P') {
	return S_IPAC_noIpacId;
    }
    if ((id->asciiA & 0xff) != 'A') {
	return S_IPAC_noIpacId;
    }
    if ((id->asciiC & 0xff) != 'C') {
	return S_IPAC_noIpacId;
    }

    return OK;
}


/*******************************************************************************

Routine:
    checkCRC

Function:
    Calculate the CRC of the IDprom at the given address.

Description:
    Generates an industry standard CRC of the ID Prom data as described 
    in the GreenSpring Industry Pack specification.  The CRC byte in the 
    Prom (at address 0x17) is set to zero for the purpose of calculating 
    the CRC.

Returns:
    The low 8 bits of the calculated CRC value.

*/

LOCAL int checkCRC (
    uint16_t *data, 
    ushort_t length
) {
    uint_t i, crc = 0xffff;
    uint16_t mask;

    for (i = 0; i < length; i++) {
	mask = 0x80;
	while (mask) {
	    if ((data[i] & mask) && (i != 0xb)) {
		crc ^= 0x8000;
	    }
	    crc += crc;
	    if (crc > 0xffff) {
		crc = (crc & 0xffff) ^ 0x1021;
	    }
	    mask >>= 1;
	}
    }

    return (~crc) & 0xff;
}


/*******************************************************************************

Routine:
    ipmValidate

Function:
    Validate a particular IPAC module type at the given carrier & slot number.

Description:
    Uses ipmCheck to ensure the carrier and slot numbers are legal, probe the 
    IDprom and check that the IDprom looks like an IPAC module.  Calculates 
    the CRC for the ID Prom, and compares the manufacturer and model ID values 
    in the Prom to the ones given.

Returns:
    0 = OK,
    S_IPAC_badAddress = Bad carrier or slot number,
    S_IPAC_noModule = No module installed,
    S_IPAC_noIpacId = "IPAC" identifier not found
    S_IPAC_badCRC = CRC Check failed,
    S_IPAC_badModule = Manufacturer or model IDs wrong

*/

int ipmValidate (
    ushort_t carrier, 
    ushort_t slot,
    uchar_t manufacturerId, 
    uchar_t modelId
) {
    ipac_idProm_t *id;
    int crc;
    int status;

    status = ipmCheck(carrier, slot);
    if (status) {
	return status;
    }

    id = (ipac_idProm_t *) ipmBaseAddr(carrier, slot, ipac_addrID);
    crc = checkCRC((uint16_t *) id, id->bytesUsed);
    if (crc != (id->CRC & 0xff)) {
	return S_IPAC_badCRC;
    }

    if (id->manufacturerId != manufacturerId ||
	id->modelId != modelId) {
	return S_IPAC_badModule;
    }

    return OK;
}


/*******************************************************************************

Routine:
    ipmReport

Function:
    returns printable string giving status of module at given carrier/slot.

Description:
    Generates a report string describing the given IPAC slot.  If a module 
    is installed, it includes the manufacturer and model ID numbers.  If 
    the report function is supported by the carrier driver this report 
    string is appended.

Returns:
    Pointer to static, printable string.

Sample Output:
    "C0 S1 : 0xB1/0x01 - M0 L4,5"

*/

char *ipmReport (
    ushort_t carrier, 
    ushort_t slot
) {
    static char report[80];
    int status;

    sprintf(report, "C%hd S%hd : ", carrier, slot);

    status = ipmCheck(carrier, slot);
    if (status == S_IPAC_badAddress) {
	strcat(report, "No such carrier/slot");
	return report;
    }

    if (status == S_IPAC_noModule) {
	strcat(report, "No Module");
    } else {
	ipac_idProm_t *id;
	char module[16];

	id = (ipac_idProm_t *) ipmBaseAddr(carrier, slot, ipac_addrID);
	sprintf(module, "0x%02hX/0x%02hX", (short) id->manufacturerId,
		(short) id->modelId);
	strcat(report, module);
    }

    if (carriers.info[carrier]->driver->report != NULL) {
	strcat(report, " - ");
	strcat(report, carriers.info[carrier]->driver->report(
			carriers.info[carrier]->cPrivate, slot));
    }

    return report;
}


/*******************************************************************************

Routine:
    ipmBaseAddr

Function:
    Returns a pointer to the selected IP address space

Description:
    Checks its input parameters, then calls the carrier driver.  This will
    return a pointer to the location of the address space indicated by the 
    space parameter.  All IP modules must provide an ID prom to indicate 
    the module type (space = ipac_addrID).  Most modules need register I/O 
    locations, which are in the I/O space (space = ipac_addrIO).  Some 
    types of module also provide memory (space = ipac_addrMem), but if 
    this is not required the carrier may allow it to be disabled, in which 
    case the driver should return a NULL for this address space.  Some 
    carriers provide a 32-bit wide I/O space for Dual-slot IP modules; 
    carriers which do not should return NULL for this space.

Returns:
    Base CPU address of IP address space, or NULL pointer.

*/

void *ipmBaseAddr (
    ushort_t carrier, 
    ushort_t slot,
    ipac_addr_t space
) {
    if (carrier >= carriers.number ||
	slot >= carriers.info[carrier]->driver->numberSlots) {
	return NULL;
    }
    return carriers.info[carrier]->driver->baseAddr(
		carriers.info[carrier]->cPrivate, slot, space);
}


/*******************************************************************************

Routine:
    ipmIrqCmd

Function:
    Send command to slot interrupt controller.

Description:
    Checks input parameters, then passes the interrupt command request to 
    the carrier driver routine.  The driver is only required to support 
    the command ipac_irqEnable; for other commands it may return the status 
    code S_IPAC_notImplemented and do nothing.

Returns:
    0 = OK,
    S_IPAC_badAddress = illegal carrier, slot or irqNumber,
    S_IPAC_notImplemented = Driver does not support that command,
    other, depending on command.

*/

int ipmIrqCmd (
    ushort_t carrier, 
    ushort_t slot, 
    ushort_t irqNumber, 
    ipac_irqCmd_t cmd
) {
    if (irqNumber > 1 ||
	carrier >= carriers.number ||
	slot >= carriers.info[carrier]->driver->numberSlots) {
	return S_IPAC_badAddress;
    }

    return carriers.info[carrier]->driver->irqCmd(
		carriers.info[carrier]->cPrivate, slot, irqNumber, cmd);
}


/*******************************************************************************

Routine:
    ipmIntConnect

Function:
    Connect module driver to interrupt vector number

Description:
    Checks input parameters, then passes the request to the carrier driver 
    routine.  If no carrier routine is provided it calls the standard vxWorks
    intConnect routine instead.  This is not quite a straight replacement for
    intConnect; as well as providing the carrier and slot numbers the module
    driver must not use the INUM_TO_IVEC() macro but just give the vector
    number.

    VxWorks' interrupt vectoring mechanism varies between bus types, and this
    routine allows a module driver to connect its routine to an interrupt
    vector from a  particular IPAC module without knowing the requirements of
    the particular bus type.  Some carrier drivers will need to maintain a
    private interrupt dispatch table if the bus type (i.e. ISA) does not
    support interrupt vectoring.

Returns:
    0 = OK,
    S_IPAC_badAddress = illegal carrier, slot or vector

*/
int ipmIntConnect (
	ushort_t carrier, 
	ushort_t slot, 
	ushort_t vecNum, 
	void (*routine)(int parameter), 
	int parameter
) {
    if (vecNum > 0xff ||
    	carrier >= carriers.number ||
	slot >= carriers.info[carrier]->driver->numberSlots) {
	return S_IPAC_badAddress;
    }

    /* Use intConnect if carrier driver doesn't provide one */
    if (carriers.info[carrier]->driver->intConnect == NULL) {
    	return intConnect (INUM_TO_IVEC(vecNum), routine, parameter);
    }

    return carriers.info[carrier]->driver->intConnect(
		carriers.info[carrier]->cPrivate, slot, vecNum, 
		routine, parameter);
}


/*******************************************************************************

Routine:
    ipacReport

Function:
    Report status of all known IPAC carriers

Description:
    Prints information on each known carrier board and slot according to the
    specified interest level.  Level 0 lists carriers only, with the number 
    of slots it supports.  Level 1 gives each slot, manufacturer & model ID 
    of the installed module (if any), and the carrier driver report for that
    slot.  Level 2 adds the address of each memory space for the slot.

Returns:
    OK.

*/

int ipacReport (
    int interest
) {
    ushort_t carrier, slot;

    for (carrier=0; carrier < carriers.number; carrier++) {
	printf("  IP Carrier %2d: %s, %d slots\n", carrier, 
		carriers.info[carrier]->driver->carrierType,
		carriers.info[carrier]->driver->numberSlots);

	if (interest > 0) {
	    void *memBase, *io32Base;

	    for (slot=0; slot < carriers.info[carrier]->driver->numberSlots; 
		 slot++) {
		printf("    %s\n", ipmReport(carrier, slot));

		if (interest > 1) {
		    printf("      ID = %p, I/O = %p", 
			    ipmBaseAddr(carrier, slot, ipac_addrID),
			    ipmBaseAddr(carrier, slot, ipac_addrIO));
		    io32Base = ipmBaseAddr(carrier, slot, ipac_addrIO32);
		    if (io32Base != NULL) {
			printf(", I/O32 = %p", io32Base);
		    }
		    memBase = ipmBaseAddr(carrier, slot, ipac_addrMem);
		    if (memBase != NULL) {
			printf(", Mem = %p", memBase);
		    }
		    printf("\n");
		}
	    }
	}
    }
    return OK;
}


/*******************************************************************************

Routine:
    ipacInitialise

Function:
    Initialise the IPAC driver

Description:
    Null routine.

Returns:
    OK.

*/

int ipacInitialise (
    int after
) {
    return OK;
}

