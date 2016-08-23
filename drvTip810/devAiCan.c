/*******************************************************************************

Project:
    CAN Bus Driver for EPICS

File:
    devAiCan.c

Description:
    CANBUS Analogue Input device support

Author:
    Andrew Johnson <anjohnson@iee.org>
Created:
    8 August 1995

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

#include <epicsTypes.h>
#include <epicsTimer.h>
#include <errMdef.h>
#include <devLib.h>
#include <dbDefs.h>
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
#include <aiRecord.h>
#include <menuConvert.h>
#include <epicsExport.h>

#include "canBus.h"


#define CONVERT 0
#define DO_NOT_CONVERT 2


typedef struct aiCanPrivate_s {
    CALLBACK callback;
    struct aiCanPrivate_s *nextPrivate;
    epicsTimerId timId;
    IOSCANPVT ioscanpvt;
    dbCommon *prec;
    canIo_t inp;
    epicsUInt32 mask;
    epicsUInt32 sign;
    epicsUInt32 data;
    double dval;
    int status;
} aiCanPrivate_t;

typedef struct aiCanBus_s {
    CALLBACK callback;
    struct aiCanBus_s *nextBus;
    aiCanPrivate_t *firstPrivate;
    void *canBusID;
    int status;
} aiCanBus_t;

static long init_ai(struct aiRecord *prec);
static long get_ioint_info(int cmd, struct aiRecord *prec, IOSCANPVT *ppvt);
static long read_ai(struct aiRecord *prec);
static long special_linconv(struct aiRecord *prec, int after);
static void ProcessCallback(CALLBACK *pcallback);
static void aiMessage(void *private, const canMessage_t *pmessage);
static void busSignal(void *private, int status);
static void busCallback(CALLBACK *pCallback);

struct {
    long number;
    DEVSUPFUN report;
    DEVSUPFUN init;
    DEVSUPFUN init_record;
    DEVSUPFUN get_ioint_info;
    DEVSUPFUN read_ai;
    DEVSUPFUN special_linconv;
} devAiCan = {
    6,
    NULL,
    NULL,
    init_ai,
    get_ioint_info,
    read_ai,
    special_linconv
};
epicsExportAddress(dset, devAiCan);

static aiCanBus_t *firstBus;


static long init_ai (
    struct aiRecord *prec
) {
    aiCanPrivate_t *pcanAi;
    aiCanBus_t *pbus;
    int status;
    epicsUInt32 fsd;

    if (prec->inp.type != INST_IO) {
	recGblRecordError(S_db_badField, prec,
			  "devAiCan (init_record) Illegal INP field");
	return S_db_badField;
    }

    pcanAi = malloc(sizeof(aiCanPrivate_t));
    if (pcanAi == NULL) {
	return S_dev_noMemory;
    }
    prec->dpvt = pcanAi;
    pcanAi->prec = (dbCommon *) prec;
    pcanAi->ioscanpvt = NULL;
    pcanAi->status = NO_ALARM;

    /* Convert the address string into members of the canIo structure */
    status = canIoParse(prec->inp.value.instio.string, &pcanAi->inp);
    if (status) {
	if (canSilenceErrors) {
	    pcanAi->inp.canBusID = NULL;
	    prec->pact = TRUE;
	    return 0;
	} else {
	    recGblRecordError(S_can_badAddress, prec,
			      "devAiCan (init_record) bad CAN address");
	    return S_can_badAddress;
	}
    }

    #ifdef DEBUG
	printf("aiCan %s: Init bus=%s, id=%#x, off=%u, parm=%ld str=%s\n",
		    prec->name, pcanAi->inp.busName, pcanAi->inp.identifier,
		    pcanAi->inp.offset, pcanAi->inp.parameter,
		    pcanAi->inp.paramStr);
    #endif

    /* For ai records, the final parameter specifies the raw input size.
	eg 0xff or 0x100 specify an 8-bit unsigned value. -ve numbers 
	specify a signed value, eg -4095 means a 12-bit signed value.
	The range does not have to be a power of two, eg 99 is legal. */

    fsd = abs(pcanAi->inp.parameter);
    if (fsd > 0) {
	if ((fsd & (fsd-1)) == 0) {
	    fsd--;
	}

	/* Make a mask to contain only the valid input bits based on fsd */
	pcanAi->mask = 1;
	while (pcanAi->mask < fsd) {
	    pcanAi->mask <<= 1;
	}
	pcanAi->mask--;

	if (pcanAi->inp.parameter < 0) {
	    /* signed: rval = sign-extend(data & mask) */
	    pcanAi->sign = (pcanAi->mask >> 1) + 1;
	} else {
	    /* unsigned: rval = data & mask */
	    pcanAi->sign = 0;
	}

	if (prec->linr == menuConvertLINEAR) {
	    prec->roff = pcanAi->sign;
	    prec->eslo = (prec->eguf - prec->egul) / fsd;
	} else {
	    prec->roff = 0;
	}
    } else {
	pcanAi->mask = pcanAi->sign = 0;
	if (pcanAi->inp.paramStr) {
	    if (strcmp(pcanAi->inp.paramStr, "float") == 0) {
		pcanAi->sign = 4;
	    } else if (strcmp(pcanAi->inp.paramStr, "double") == 0) {
		pcanAi->sign = 8;
	    }
	}
    }

    #ifdef DEBUG
	printf("  fsd=%ld, eslo=%g, roff = %ld, mask=%#lx, sign=%lu\n", 
		fsd, prec->eslo, prec->roff, pcanAi->mask, pcanAi->sign);
    #endif

    /* Find the bus matching this record */
    for (pbus = firstBus; pbus != NULL; pbus = pbus->nextBus) {
	if (pbus->canBusID == pcanAi->inp.canBusID) break;
    }

    /* If not found, create one */
    if (pbus == NULL) {
	pbus = malloc(sizeof (aiCanBus_t));
	if (pbus == NULL) return S_dev_noMemory;

	/* Fill it in */
	pbus->firstPrivate = NULL;
	pbus->canBusID = pcanAi->inp.canBusID;
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
    pcanAi->nextPrivate = pbus->firstPrivate;
    pbus->firstPrivate = pcanAi;

    /* Set the callback parameters for asynchronous processing */
    callbackSetUser(prec, &pcanAi->callback);
    callbackSetCallback(ProcessCallback, &pcanAi->callback);
    callbackSetPriority(prec->prio, &pcanAi->callback);

    /* and create a timer for CANbus RTR timeouts */
    pcanAi->timId = epicsTimerQueueCreateTimer(canTimerQ,
		(epicsTimerCallback) callbackRequest, pcanAi);
    if (pcanAi->timId == NULL) {
	return S_dev_noMemory;
    }

    /* Register the message handler with the Canbus driver */
    canMessage(pcanAi->inp.canBusID, pcanAi->inp.identifier, aiMessage, pcanAi);

    return 0;
}

static long get_ioint_info (
    int cmd,
    struct aiRecord *prec, 
    IOSCANPVT *ppvt
) {
    aiCanPrivate_t *pcanAi = prec->dpvt;

    if (pcanAi->ioscanpvt == NULL) {
	scanIoInit(&pcanAi->ioscanpvt);
    }

    #ifdef DEBUG
	printf("canAi %s: get_ioint_info %d\n", prec->name, cmd);
    #endif

    *ppvt = pcanAi->ioscanpvt;
    return 0;
}

static long read_ai (
    struct aiRecord *prec
) {
    aiCanPrivate_t *pcanAi = prec->dpvt;

    if (pcanAi->inp.canBusID == NULL) {
	return DO_NOT_CONVERT;
    }

    #ifdef DEBUG
	printf("canAi %s: read_ai status=%#x\n", prec->name, pcanAi->status);
    #endif

    switch (pcanAi->status) {
	case TIMEOUT_ALARM:
	case COMM_ALARM:
	    recGblSetSevr(prec, pcanAi->status, INVALID_ALARM);
	    pcanAi->status = NO_ALARM;
	    return DO_NOT_CONVERT;

	case NO_ALARM:
	    if (prec->pact || prec->scan == SCAN_IO_EVENT) {
		#ifdef DEBUG
		    printf("canAi %s: message id=%#x, data=%#lx\n", 
			    prec->name, pcanAi->inp.identifier, pcanAi->data);
		#endif

		if ((pcanAi->mask == 0) && pcanAi->sign) {
		    #ifdef DEBUG
			printf("canAi %s: VAL=%g\n", prec->name, pcanAi->dval);
		    #endif
		    prec->val = pcanAi->dval;
		    prec->udf = FALSE;
		    return DO_NOT_CONVERT;
		}
		prec->rval = pcanAi->data & pcanAi->mask;
		if (pcanAi->sign & prec->rval) {
		    prec->rval |= ~pcanAi->mask;
		}
		return CONVERT;
	    } else {
		canMessage_t message;

		message.identifier = pcanAi->inp.identifier;
		message.rtr = RTR;
		message.length = 8;

		#ifdef DEBUG
		    printf("canAi %s: RTR, id=%#x\n", 
			    prec->name, pcanAi->inp.identifier);
		#endif

		prec->pact = TRUE;
		pcanAi->status = TIMEOUT_ALARM;

		epicsTimerStartDelay(pcanAi->timId, pcanAi->inp.timeout);
		canWrite(pcanAi->inp.canBusID, &message, pcanAi->inp.timeout);
		return CONVERT;
	    }
	default:
	    recGblSetSevr(prec, UDF_ALARM, INVALID_ALARM);
	    pcanAi->status = NO_ALARM;
	    return DO_NOT_CONVERT;
    }
}

static long special_linconv (
    struct aiRecord *prec,
    int after
) {
    if (after) {
	aiCanPrivate_t *pcanAi = prec->dpvt;
	epicsUInt32 fsd = abs(pcanAi->inp.parameter);
	if (fsd > 0) {
	    if ((fsd & (fsd-1)) == 0) {
		fsd--;
	    }
	    prec->roff = pcanAi->sign;
	    prec->eslo = (prec->eguf - prec->egul) / fsd;
	}
    }
    return 0;
}

static void ProcessCallback(CALLBACK *pcallback)
{
    dbCommon *pRec;

    callbackGetUser(pRec, pcallback);
    dbScanLock(pRec);
    (*pRec->rset->process)(pRec);
    dbScanUnlock(pRec);
}

static void aiMessage (
    void *private,
    const canMessage_t *pmessage
) {
    aiCanPrivate_t *pcanAi = private;

    if (!interruptAccept ||
	pmessage->rtr == RTR) {
	return;
    }

    if (pcanAi->mask == 0) {
	/* FIXME: These have FP format problems... */
	float ival;
	switch (pcanAi->sign) {
	case 4: 	/* float */
	    memcpy(&ival, &pmessage->data[pcanAi->inp.offset], sizeof(float));
	    pcanAi->dval = ival;
	    break;
	case 8: 	/* double */
	    memcpy(&pcanAi->dval, &pmessage->data[0], sizeof(double));
	    break;
	default:
	    pcanAi->data = 0;
	}
    } else if (pcanAi->mask <= 0xff) {
	pcanAi->data = pmessage->data[pcanAi->inp.offset+0];
    } else if (pcanAi->mask <= 0xffff) {
	pcanAi->data = pmessage->data[pcanAi->inp.offset+0] <<  8 |
		       pmessage->data[pcanAi->inp.offset+1];
    } else if (pcanAi->mask <= 0xffffff) {
	pcanAi->data = pmessage->data[pcanAi->inp.offset+0] << 16 |
		       pmessage->data[pcanAi->inp.offset+1] <<  8 |
		       pmessage->data[pcanAi->inp.offset+2];
    } else {
	pcanAi->data = pmessage->data[pcanAi->inp.offset+0] << 24 |
		       pmessage->data[pcanAi->inp.offset+1] << 16 |
		       pmessage->data[pcanAi->inp.offset+2] <<  8 |
		       pmessage->data[pcanAi->inp.offset+3];
    }

    if (pcanAi->prec->scan == SCAN_IO_EVENT) {
	pcanAi->status = NO_ALARM;
	scanIoRequest(pcanAi->ioscanpvt);
    } else if (pcanAi->status == TIMEOUT_ALARM) {
	pcanAi->status = NO_ALARM;
	epicsTimerCancel(pcanAi->timId);
	callbackRequest(&pcanAi->callback);
    }
}

static void busSignal (
    void *private,
    int status
) {
    aiCanBus_t *pbus = private;

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
    aiCanBus_t *pbus;
    aiCanPrivate_t *pcanAi;

    callbackGetUser(pbus, pCallback);
    pcanAi = pbus->firstPrivate;

    while (pcanAi != NULL) {
	dbCommon *prec = pcanAi->prec;
	pcanAi->status = pbus->status;
	dbScanLock(prec);
	prec->rset->process(prec);
	dbScanUnlock(prec);
	pcanAi = pcanAi->nextPrivate;
    }
}
