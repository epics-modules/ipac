/*******************************************************************************

Project:
    CAN Bus Driver for EPICS

File:
    devAoCan.c

Description:
    CANBUS Analogue Output device support

Author:
    Andrew Johnson <anjohnson@iee.org>
Created:
    9 August 1995

Copyright (c) 1995-2000 Andrew Johnson

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


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>

#include <epicsTypes.h>
#include <epicsTimer.h>
#include <errMdef.h>
#include <devLib.h>
#include <dbAccess.h>
#include <dbScan.h>
#include <callback.h>
#include <cvtTable.h>
#include <link.h>
#include <alarm.h>
#include <recGbl.h>
#include <recSup.h>
#include <devSup.h>
#include <dbCommon.h>
#include <aoRecord.h>
#include <menuConvert.h>
#include <epicsExport.h>

#include "canBus.h"


#define DO_NOT_CONVERT	2


typedef struct aoCanPrivate_s {
    struct aoCanPrivate_s *nextPrivate;
    IOSCANPVT ioscanpvt;
    dbCommon *prec;
    canIo_t out;
    epicsUInt32 mask;
    epicsUInt32 sign;
    epicsUInt32 data;
    int status;
} aoCanPrivate_t;

typedef struct aoCanBus_s {
    CALLBACK callback;
    struct aoCanBus_s *nextBus;
    aoCanPrivate_t *firstPrivate;
    void *canBusID;
    int status;
} aoCanBus_t;

static long init_ao(struct aoRecord *prec);
static long get_ioint_info(int cmd, struct aoRecord *prec, IOSCANPVT *ppvt);
static long write_ao(struct aoRecord *prec);
static long special_linconv(struct aoRecord *prec, int after);
static void aoMessage(void *private, const canMessage_t *pmessage);
static void busSignal(void *private, int status);
static void busCallback(CALLBACK *pCallback);

struct {
    long number;
    DEVSUPFUN report;
    DEVSUPFUN init;
    DEVSUPFUN init_record;
    DEVSUPFUN get_ioint_info;
    DEVSUPFUN write_ao;
    DEVSUPFUN special_linconv;
} devAoCan = {
    6,
    NULL,
    NULL,
    init_ao,
    get_ioint_info,
    write_ao,
    special_linconv
};
epicsExportAddress(dset, devAoCan);

static aoCanBus_t *firstBus;


static long init_ao (
    struct aoRecord *prec
) {
    aoCanPrivate_t *pcanAo;
    aoCanBus_t *pbus;
    int status;
    epicsUInt32 fsd;

    if (prec->out.type != INST_IO) {
	recGblRecordError(S_db_badField, prec,
			  "devAoCan (init_record) Illegal OUT field");
	return S_db_badField;
    }

    pcanAo = malloc(sizeof(aoCanPrivate_t));
    if (pcanAo == NULL) {
	return S_dev_noMemory;
    }
    prec->dpvt = pcanAo;
    pcanAo->prec = (dbCommon *) prec;
    pcanAo->ioscanpvt = NULL;
    pcanAo->status = NO_ALARM;

    /* Convert the parameter string into members of the canIo structure */
    status = canIoParse(prec->out.value.instio.string, &pcanAo->out);
    if (status) {
	if (canSilenceErrors) {
	    pcanAo->out.canBusID = NULL;
	    prec->pact = TRUE;
	    return DO_NOT_CONVERT;
	} else {
	    recGblRecordError(S_can_badAddress, prec,
			      "devAoCan (init_record) bad CAN address");
	    return S_can_badAddress;
	}
    }

    #ifdef DEBUG
	printf("canAo %s: Init bus=%s, id=%#x, off=%d, parm=%ld\n",
		    prec->name, pcanAo->out.busName, pcanAo->out.identifier,
		    pcanAo->out.offset, pcanAo->out.parameter);
    #endif

    /* For ao records, the final parameter specifies the raw output size. 
       eg 0xfff or 0x1000 specify a 12-bit unsigned value.  -ve numbers
       specify a signed value, eg -256 means an 8-bit signed value.
       The range does not have to be a power of two, eg 99 is legal. */

    fsd = abs(pcanAo->out.parameter);
    if (fsd > 0) {
	if ((fsd & (fsd-1)) == 0) {
	    fsd--;
	}

	/* Make a mask to contain only the valid out bits based on fsd */
	pcanAo->mask = 1;
	while (pcanAo->mask < fsd) {
	    pcanAo->mask <<= 1;
	}
	pcanAo->mask--;

	if (pcanAo->out.parameter < 0) {
	    /* signed data */
	    pcanAo->sign = (pcanAo->mask >> 1) + 1;
	} else {
	    pcanAo->sign = 0;
	}
	if (prec->linr == menuConvertLINEAR) {
	    prec->roff = pcanAo->sign;
	    prec->eslo = (prec->eguf - prec->egul) / fsd;
	} else {
	    prec->roff = 0;
	}
    } else {
	pcanAo->mask = pcanAo->sign = 0;
	if (pcanAo->out.paramStr) {
	    if (strcmp(pcanAo->out.paramStr, "float") == 0) {
		pcanAo->sign = 4;
	    } else if (strcmp(pcanAo->out.paramStr, "double") == 0) {
		pcanAo->sign = 8;
	    }
	}
    }

    #ifdef DEBUG
	printf("  fsd=%ld, eslo=%g, roff=%ld, mask=%#lx, sign=%ld\n", 
		fsd, prec->eslo, prec->roff, pcanAo->mask, pcanAo->sign);
    #endif

    /* Find the bus matching this record */
    for (pbus = firstBus; pbus != NULL; pbus = pbus->nextBus) {
    	if (pbus->canBusID == pcanAo->out.canBusID) break;
    }

    /* If not found, create one */
    if (pbus == NULL) {
	pbus = malloc(sizeof (aoCanBus_t));
	if (pbus == NULL) return S_dev_noMemory;

	/* Fill it in */
	pbus->firstPrivate = NULL;
	pbus->canBusID = pcanAo->out.canBusID;
	callbackSetUser(pbus, &pbus->callback);
	callbackSetCallback(busCallback, &pbus->callback);
	callbackSetPriority(priorityMedium, &pbus->callback);

	/* and add it to the list of busses we know about */
	pbus->nextBus = firstBus;
	firstBus = pbus;

	/* Ask driver for error signals */
	canSignal(pbus->canBusID, busSignal, pbus);
    }

    /* Insert private record structure into linked list for this CANbus */
    pcanAo->nextPrivate = pbus->firstPrivate;
    pbus->firstPrivate = pcanAo;

    /* Register the message handler with the Canbus driver */
    canMessage(pcanAo->out.canBusID, pcanAo->out.identifier, aoMessage, pcanAo);

    return DO_NOT_CONVERT;
}

static long get_ioint_info (
    int cmd,
    struct aoRecord *prec, 
    IOSCANPVT *ppvt
) {
    aoCanPrivate_t *pcanAo = prec->dpvt;

    if (pcanAo->ioscanpvt == NULL) {
	scanIoInit(&pcanAo->ioscanpvt);
    }

    #ifdef DEBUG
	printf("aoCan %s: get_ioint_info %d\n", prec->name, cmd);
    #endif

    *ppvt = pcanAo->ioscanpvt;
    return 0;
}

static long write_ao (
    struct aoRecord *prec
) {
    aoCanPrivate_t *pcanAo = prec->dpvt;

    if (pcanAo->out.canBusID == NULL) {
	return -1;
    }

    #ifdef DEBUG
	printf("aoCan %s: write_ao status=%#x\n", prec->name, pcanAo->status);
    #endif

    switch (pcanAo->status) {
	case COMM_ALARM:
	    recGblSetSevr(prec, pcanAo->status, INVALID_ALARM);
	    pcanAo->status = NO_ALARM;
	    return -1;

	case NO_ALARM:
	    {
		canMessage_t message;
		int status;

		message.identifier = pcanAo->out.identifier;
		message.rtr = SEND;

		pcanAo->data = prec->rval & pcanAo->mask;

		if (pcanAo->mask == 0) {
		    /* FIXME: These have endian problems... */
		    float oval;
		    message.length = pcanAo->sign;
		    switch (message.length) {
		    case 4:	/* float */
			if (fabs(prec->oval) < FLT_MIN) {
			    oval = 0.0;
			} else if (fabs (prec->oval) > FLT_MAX) {
			    recGblSetSevr(prec, WRITE_ALARM, INVALID_ALARM);
			    return -1;
			} else {
			    oval = prec->oval;
			}
			memcpy(&message.data[0], &oval, sizeof(float));
			break;
		    case 8:	/* double */
		        memcpy(&message.data[0], &prec->oval, sizeof(double));
			break;
		    default:
		        break;
		    }
		} else if (pcanAo->mask <= 0xff) {
		    message.data[0] = (pcanAo->data      ) & 0xff;
		    message.length  = 1;
		} else if (pcanAo->mask <= 0xffff) {
		    message.data[0] = (pcanAo->data >>  8) & 0xff;
		    message.data[1] = (pcanAo->data      ) & 0xff;
		    message.length  = 2;
		} else if (pcanAo->mask <= 0xffffff) {
		    message.data[0] = (pcanAo->data >> 16) & 0xff;
		    message.data[1] = (pcanAo->data >>  8) & 0xff;
		    message.data[2] = (pcanAo->data      ) & 0xff;
		    message.length  = 3;
		} else {
		    message.data[0] = (pcanAo->data >> 24) & 0xff;
		    message.data[1] = (pcanAo->data >> 16) & 0xff;
		    message.data[2] = (pcanAo->data >>  8) & 0xff;
		    message.data[3] = (pcanAo->data      ) & 0xff;
		    message.length  = 4;
		}

		#ifdef DEBUG
		    printf("canAo %s: SEND id=%#x, length=%d, data=%#lx\n", 
			    prec->name, message.identifier, message.length, 
			    pcanAo->data);
		#endif

		status = canWrite(pcanAo->out.canBusID, &message, 
				  pcanAo->out.timeout);
		if (status) {
		    #ifdef DEBUG
			printf("canAo %s: canWrite status=%#x\n", 
				prec->name, status);
		    #endif

		    recGblSetSevr(prec, TIMEOUT_ALARM, INVALID_ALARM);
		    return -1;
		}
		return 0;
	    }
	default:
	    recGblSetSevr(prec, UDF_ALARM, INVALID_ALARM);
	    pcanAo->status = NO_ALARM;
	    return -1;
    }
}

static long special_linconv (
    struct aoRecord *prec,
    int after
) {
    if (after) {
	aoCanPrivate_t *pcanAo = prec->dpvt;
	epicsUInt32 fsd = abs(pcanAo->out.parameter);
	if (fsd > 0) {
	    if ((fsd & (fsd-1)) == 0) {
		fsd--;
	    }
	    prec->roff = pcanAo->sign;
	    prec->eslo = (prec->eguf - prec->egul) / fsd;
	}
    }
    return 0;
}

static void aoMessage (
    void *private,
    const canMessage_t *pmessage
) {
    aoCanPrivate_t *pcanAo = private;

    if (!interruptAccept) return;

    if (pcanAo->prec->scan == SCAN_IO_EVENT &&
	pmessage->rtr == RTR) {
	pcanAo->status = NO_ALARM;
	scanIoRequest(pcanAo->ioscanpvt);
    }
}

static void busSignal (
    void *private,
    int status
) {
    aoCanBus_t *pbus = private;

    if (!interruptAccept) return;

    switch(status) {
	case CAN_BUS_OK:
	    pbus->status = NO_ALARM;
	    break;
	case CAN_BUS_ERROR:
	    pbus->status = COMM_ALARM;
	    callbackRequest(&pbus->callback);
	    break;
	case CAN_BUS_OFF:
	    pbus->status = COMM_ALARM;
	    callbackRequest(&pbus->callback);
	    break;
    }
}

static void busCallback (
    CALLBACK *pCallback
) {
    aoCanBus_t *pbus;
    aoCanPrivate_t *pcanAo;

    callbackGetUser(pbus, pCallback);
    pcanAo = pbus->firstPrivate;

    while (pcanAo != NULL) {
	dbCommon *prec = pcanAo->prec;
	pcanAo->status = pbus->status;
	dbScanLock(prec);
	prec->rset->process(prec);
	dbScanUnlock(prec);
	pcanAo = pcanAo->nextPrivate;
    }
}
