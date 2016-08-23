/*******************************************************************************

Project:
    CAN Bus Driver for EPICS

File:
    devBoCan.c

Description:
    CANBUS Binary Output device support

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
#include <boRecord.h>
#include <epicsExport.h>

#include "canBus.h"


#define DO_NOT_CONVERT	2


typedef struct boCanPrivate_s {
    struct boCanPrivate_s *nextPrivate;
    IOSCANPVT ioscanpvt;
    dbCommon *prec;
    canIo_t out;
    epicsUInt32 data;
    int status;
} boCanPrivate_t;

typedef struct boCanBus_s {
    CALLBACK callback;
    struct boCanBus_s *nextBus;
    boCanPrivate_t *firstPrivate;
    void *canBusID;
    int status;
} boCanBus_t;

static long init_bo(struct boRecord *prec);
static long get_ioint_info(int cmd, struct boRecord *prec, IOSCANPVT *ppvt);
static long write_bo(struct boRecord *prec);
static void boMessage(void *private, const canMessage_t *pmessage);
static void busSignal(void *private, int status);
static void busCallback(CALLBACK *pCallback);

struct {
    long number;
    DEVSUPFUN report;
    DEVSUPFUN init;
    DEVSUPFUN init_record;
    DEVSUPFUN get_ioint_info;
    DEVSUPFUN write_bo;
} devBoCan = {
    5,
    NULL,
    NULL,
    init_bo,
    get_ioint_info,
    write_bo
};
epicsExportAddress(dset, devBoCan);

static boCanBus_t *firstBus;


static long init_bo (
    struct boRecord *prec
) {
    boCanPrivate_t *pcanBo;
    boCanBus_t *pbus;
    int status;

    if (prec->out.type != INST_IO) {
	recGblRecordError(S_db_badField, prec,
			  "devBoCan (init_record) Illegal OUT field");
	return S_db_badField;
    }

    pcanBo = malloc(sizeof(boCanPrivate_t));
    if (pcanBo == NULL) {
	return S_dev_noMemory;
    }
    prec->dpvt = pcanBo;
    pcanBo->prec = (dbCommon *) prec;
    pcanBo->ioscanpvt = NULL;
    pcanBo->status = NO_ALARM;

    /* Convert the parameter string into members of the canIo structure */
    status = canIoParse(prec->out.value.instio.string, &pcanBo->out);
    if (status ||
	pcanBo->out.parameter < 0 ||
	pcanBo->out.parameter > 7) {
	if (canSilenceErrors) {
	    pcanBo->out.canBusID = NULL;
	    prec->pact = TRUE;
	    return DO_NOT_CONVERT;
	} else {
	    recGblRecordError(S_can_badAddress, prec,
			      "devBoCan (init_record) bad CAN address");
	    return S_can_badAddress;
	}
    }

    #ifdef DEBUG
	printf("canBo %s: Init bus=%s, id=%#x, off=%d, parm=%ld\n",
		    prec->name, pcanBo->out.busName, pcanBo->out.identifier,
		    pcanBo->out.offset, pcanBo->out.parameter);
    #endif

    /* For bo records, the final parameter specifies the output bit number,
       with the offset specifying the message byte number. */
    prec->mask = 1 << pcanBo->out.parameter;

    #ifdef DEBUG
	printf("  bit=%ld, mask=%#lx\n", pcanBo->out.parameter, prec->mask);
    #endif

    /* Find the bus matching this record */
    for (pbus = firstBus; pbus != NULL; pbus = pbus->nextBus) {
    	if (pbus->canBusID == pcanBo->out.canBusID) break;
    }

    /* If not found, create one */
    if (pbus == NULL) {
	pbus = malloc(sizeof (boCanBus_t));
	if (pbus == NULL) return S_dev_noMemory;

	/* Fill it in */
	pbus->firstPrivate = NULL;
	pbus->canBusID = pcanBo->out.canBusID;
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
    pcanBo->nextPrivate = pbus->firstPrivate;
    pbus->firstPrivate = pcanBo;

    /* Register the message handler with the Canbus driver */
    canMessage(pcanBo->out.canBusID, pcanBo->out.identifier, boMessage, pcanBo);

    return DO_NOT_CONVERT;
}

static long get_ioint_info (
    int cmd,
    struct boRecord *prec, 
    IOSCANPVT *ppvt
) {
    boCanPrivate_t *pcanBo = prec->dpvt;

    if (pcanBo->ioscanpvt == NULL) {
	scanIoInit(&pcanBo->ioscanpvt);
    }

    #ifdef DEBUG
	printf("boCan %s: get_ioint_info %d\n", prec->name, cmd);
    #endif

    *ppvt = pcanBo->ioscanpvt;
    return 0;
}

static long write_bo (
    struct boRecord *prec
) {
    boCanPrivate_t *pcanBo = prec->dpvt;

    if (pcanBo->out.canBusID == NULL) {
	return -1;
    }

    #ifdef DEBUG
	printf("boCan %s: write_bo status=%#x\n", prec->name, pcanBo->status);
    #endif

    switch (pcanBo->status) {
	case COMM_ALARM:
	    recGblSetSevr(prec, pcanBo->status, INVALID_ALARM);
	    pcanBo->status = NO_ALARM;
	    return -1;

	case NO_ALARM:
	    {
		canMessage_t message;
		int status;

		message.identifier = pcanBo->out.identifier;
		message.rtr = SEND;

		pcanBo->data = prec->rval & prec->mask;

		message.data[pcanBo->out.offset] = pcanBo->data;
		message.length  = pcanBo->out.offset + 1;

		#ifdef DEBUG
		    printf("canBo %s: SEND id=%#x, length=%d, data=%#lx\n", 
			    prec->name, message.identifier, message.length, 
			    pcanBo->data);
		#endif

		status = canWrite(pcanBo->out.canBusID, &message, 
				  pcanBo->out.timeout);
		if (status) {
		    #ifdef DEBUG
			printf("canBo %s: canWrite status=%#x\n",
				prec->name, status);
		    #endif

		    recGblSetSevr(prec, TIMEOUT_ALARM, INVALID_ALARM);
		    return -1;
		}
		return 0;
	    }
	default:
	    recGblSetSevr(prec, UDF_ALARM, INVALID_ALARM);
	    pcanBo->status = NO_ALARM;
	    return -1;
    }
}

static void boMessage (
    void *private,
    const canMessage_t *pmessage
) {
    boCanPrivate_t *pcanBo = private;

    if (!interruptAccept) return;

    if (pcanBo->prec->scan == SCAN_IO_EVENT &&
	pmessage->rtr == RTR) {
	pcanBo->status = NO_ALARM;
	scanIoRequest(pcanBo->ioscanpvt);
    }
}

static void busSignal (
    void *private,
    int status
) {
    boCanBus_t *pbus = private;

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
    boCanBus_t *pbus;
    boCanPrivate_t *pcanBo;

    callbackGetUser(pbus, pCallback);
    pcanBo = pbus->firstPrivate;

    while (pcanBo != NULL) {
	dbCommon *prec = pcanBo->prec;
	pcanBo->status = pbus->status;
	dbScanLock(prec);
	prec->rset->process(pcanBo->prec);
	dbScanUnlock(prec);
	pcanBo = pcanBo->nextPrivate;
    }
}
