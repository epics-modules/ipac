/*******************************************************************************

Project:
    CAN Bus Driver for EPICS

File:
    devBiCan.c

Description:
    CANBUS Binary Input device support

Author:
    Andrew Johnson <anjohnson@iee.org>
Created:
    14 August 1995

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
#include <biRecord.h>
#include <epicsExport.h>

#include "canBus.h"


#define CONVERT 0
#define DO_NOT_CONVERT 2


typedef struct biCanPrivate_s {
    CALLBACK callback;
    struct biCanPrivate_s *nextPrivate;
    epicsTimerId timId;
    IOSCANPVT ioscanpvt;
    struct dbCommon *prec;
    canIo_t inp;
    epicsUInt32 data;
    int status;
} biCanPrivate_t;

typedef struct biCanBus_s {
    CALLBACK callback;
    struct biCanBus_s *nextBus;
    biCanPrivate_t *firstPrivate;
    void *canBusID;
    int status;
} biCanBus_t;

static long init_bi(struct biRecord *prec);
static long get_ioint_info(int cmd, struct biRecord *prec, IOSCANPVT *ppvt);
static long read_bi(struct biRecord *prec);
static void ProcessCallback(CALLBACK *pcallback);
static void biMessage(void *private, const canMessage_t *pmessage);
static void busSignal(void *private, int status);
static void busCallback(CALLBACK *pcallback);

struct {
    long number;
    DEVSUPFUN report;
    DEVSUPFUN init;
    DEVSUPFUN init_record;
    DEVSUPFUN get_ioint_info;
    DEVSUPFUN read_bi;
} devBiCan = {
    5,
    NULL,
    NULL,
    init_bi,
    get_ioint_info,
    read_bi
};
epicsExportAddress(dset, devBiCan);

static biCanBus_t *firstBus;


static long init_bi (
    struct biRecord *prec
) {
    biCanPrivate_t *pcanBi;
    biCanBus_t *pbus;
    int status;

    if (prec->inp.type != INST_IO) {
	recGblRecordError(S_db_badField, prec,
			  "devBiCan (init_record) Illegal INP field");
	return S_db_badField;
    }

    pcanBi = malloc(sizeof(biCanPrivate_t));
    if (pcanBi == NULL) {
	return S_dev_noMemory;
    }
    prec->dpvt = pcanBi;
    pcanBi->prec = (dbCommon *) prec;
    pcanBi->ioscanpvt = NULL;
    pcanBi->status = NO_ALARM;

    /* Convert the address string into members of the canIo structure */
    status = canIoParse(prec->inp.value.instio.string, &pcanBi->inp);
    if (status ||
	pcanBi->inp.parameter < 0 ||
	pcanBi->inp.parameter > 7) {
	if (canSilenceErrors) {
	    pcanBi->inp.canBusID = NULL;
	    prec->pact = TRUE;
	    return 0;
	} else {
	    recGblRecordError(S_can_badAddress, prec,
			      "devBiCan (init_record) bad CAN address");
	    return S_can_badAddress;
	}
    }

    #ifdef DEBUG
	printf("biCan %s: Init bus=%s, id=%#x, off=%d, parm=%ld\n",
		    prec->name, pcanBi->inp.busName, pcanBi->inp.identifier,
		    pcanBi->inp.offset, pcanBi->inp.parameter);
    #endif

    /* For bi records, the final parameter specifies the input bit number,
       with offset specifying the message byte number. */
    prec->mask = 1 << pcanBi->inp.parameter;

    #ifdef DEBUG
	printf("  bit=%ld, mask=%#lx\n", 
		pcanBi->inp.parameter, prec->mask);
    #endif

    /* Find the bus matching this record */
    for (pbus = firstBus; pbus != NULL; pbus = pbus->nextBus) {
    	if (pbus->canBusID == pcanBi->inp.canBusID) break;
    }

    /* If not found, create one */
    if (pbus == NULL) {
	pbus = malloc(sizeof (biCanBus_t));
	if (pbus == NULL) return S_dev_noMemory;

	/* Fill it in */
	pbus->firstPrivate = NULL;
	pbus->canBusID = pcanBi->inp.canBusID;
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
    pcanBi->nextPrivate = pbus->firstPrivate;
    pbus->firstPrivate = pcanBi;

    /* Set the callback parameters for asynchronous processing */
    callbackSetUser(prec, &pcanBi->callback);
    callbackSetCallback(ProcessCallback, &pcanBi->callback);
    callbackSetPriority(prec->prio, &pcanBi->callback);

    /* and create a timer for CANbus RTR timeouts */
    pcanBi->timId = epicsTimerQueueCreateTimer(canTimerQ,
		(epicsTimerCallback)callbackRequest, pcanBi);
    if (pcanBi->timId == NULL) {
	return S_dev_noMemory;
    }

    /* Register the message handler with the Canbus driver */
    canMessage(pcanBi->inp.canBusID, pcanBi->inp.identifier, biMessage, pcanBi);

    return 0;
}

static long get_ioint_info (
    int cmd,
    struct biRecord *prec, 
    IOSCANPVT *ppvt
) {
    biCanPrivate_t *pcanBi = prec->dpvt;

    if (pcanBi->ioscanpvt == NULL) {
	scanIoInit(&pcanBi->ioscanpvt);
    }

    #ifdef DEBUG
	printf("canBi %s: get_ioint_info %d\n", prec->name, cmd);
    #endif

    *ppvt = pcanBi->ioscanpvt;
    return 0;
}

static long read_bi (
    struct biRecord *prec
) {
    biCanPrivate_t *pcanBi = prec->dpvt;

    if (pcanBi->inp.canBusID == NULL) {
	return DO_NOT_CONVERT;
    }

    #ifdef DEBUG
	printf("canBi %s: read_bi status=%#x\n", prec->name, pcanBi->status);
    #endif

    switch (pcanBi->status) {
	case TIMEOUT_ALARM:
	case COMM_ALARM:
	    recGblSetSevr(prec, pcanBi->status, INVALID_ALARM);
	    pcanBi->status = NO_ALARM;
	    return DO_NOT_CONVERT;

	case NO_ALARM:
	    if (prec->pact || prec->scan == SCAN_IO_EVENT) {
		#ifdef DEBUG
		    printf("canBi %s: message id=%#x, data=%#lx\n", 
			    prec->name, pcanBi->inp.identifier, pcanBi->data);
		#endif

		prec->rval = pcanBi->data & prec->mask;
		return CONVERT;
	    } else {
		canMessage_t message;

		message.identifier = pcanBi->inp.identifier;
		message.rtr = RTR;
		message.length = 8;

		#ifdef DEBUG
		    printf("canBi %s: RTR, id=%#x\n", 
			    prec->name, pcanBi->inp.identifier);
		#endif

		prec->pact = TRUE;
		pcanBi->status = TIMEOUT_ALARM;

		epicsTimerStartDelay(pcanBi->timId, pcanBi->inp.timeout);
		canWrite(pcanBi->inp.canBusID, &message, pcanBi->inp.timeout);
		return DO_NOT_CONVERT;
	    }
	default:
	    recGblSetSevr(prec, UDF_ALARM, INVALID_ALARM);
	    pcanBi->status = NO_ALARM;
	    return DO_NOT_CONVERT;
    }
}

static void ProcessCallback(CALLBACK *pcallback)
{
    dbCommon    *pRec;

    callbackGetUser(pRec, pcallback);
    dbScanLock(pRec);
    (*pRec->rset->process)(pRec);
    dbScanUnlock(pRec);
}

static void biMessage (
    void *private,
    const canMessage_t *pmessage
) {
    biCanPrivate_t *pcanBi = private;

    if (!interruptAccept ||
	pmessage->rtr == RTR) {
	return;
    }

    pcanBi->data = pmessage->data[pcanBi->inp.offset];

    if (pcanBi->prec->scan == SCAN_IO_EVENT) {
	pcanBi->status = NO_ALARM;
	scanIoRequest(pcanBi->ioscanpvt);
    } else if (pcanBi->status == TIMEOUT_ALARM) {
	pcanBi->status = NO_ALARM;
	epicsTimerCancel(pcanBi->timId);
	callbackRequest(&pcanBi->callback);
    }
}

static void busSignal (
    void *private,
    int status
) {
    biCanBus_t *pbus = private;

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
    biCanBus_t *pbus;
    biCanPrivate_t *pcanBi;

    callbackGetUser(pbus, pCallback);
    pcanBi = pbus->firstPrivate;

    while (pcanBi != NULL) {
	struct dbCommon *prec = pcanBi->prec;
	pcanBi->status = pbus->status;
	dbScanLock(prec);
	prec->rset->process(prec);
	dbScanUnlock(prec);
	pcanBi = pcanBi->nextPrivate;
    }
}
