/*******************************************************************************

Project:
    CAN Bus Driver for EPICS

File:
    devMbbiCan.c

Description:
    CANBUS Multi-Bit Binary Input device support

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
#include <mbbiRecord.h>
#include <epicsExport.h>

#include "canBus.h"


#define CONVERT 0
#define DO_NOT_CONVERT 2


typedef struct mbbiCanPrivate_s {
    CALLBACK callback;
    struct mbbiCanPrivate_s *nextPrivate;
    epicsTimerId timId;
    IOSCANPVT ioscanpvt;
    dbCommon *prec;
    canIo_t inp;
    epicsUInt32 data;
    int status;
} mbbiCanPrivate_t;

typedef struct mbbiCanBus_s {
    CALLBACK callback;
    struct mbbiCanBus_s *nextBus;
    mbbiCanPrivate_t *firstPrivate;
    void *canBusID;
    int status;
} mbbiCanBus_t;

static long init_mbbi(struct mbbiRecord *prec);
static long get_ioint_info(int cmd, struct mbbiRecord *prec, IOSCANPVT *ppvt);
static long read_mbbi(struct mbbiRecord *prec);
static void ProcessCallback(CALLBACK *pCallback);
static void mbbiMessage(void *private, const canMessage_t *pmessage);
static void busSignal(void *private, int status);
static void busCallback(CALLBACK *pCallback);

struct {
    long number;
    DEVSUPFUN report;
    DEVSUPFUN init;
    DEVSUPFUN init_record;
    DEVSUPFUN get_ioint_info;
    DEVSUPFUN read_mbbi;
} devMbbiCan = {
    5,
    NULL,
    NULL,
    init_mbbi,
    get_ioint_info,
    read_mbbi
};
epicsExportAddress(dset, devMbbiCan);

static mbbiCanBus_t *firstBus;


static long init_mbbi (
    struct mbbiRecord *prec
) {
    mbbiCanPrivate_t *pcanMbbi;
    mbbiCanBus_t *pbus;
    int status;

    if (prec->inp.type != INST_IO) {
	recGblRecordError(S_db_badField, prec,
			  "devMbbiCan (init_record) Illegal INP field");
	return S_db_badField;
    }

    pcanMbbi = malloc(sizeof(mbbiCanPrivate_t));
    if (pcanMbbi == NULL) {
	return S_dev_noMemory;
    }
    prec->dpvt = pcanMbbi;
    pcanMbbi->prec = (dbCommon *) prec;
    pcanMbbi->ioscanpvt = NULL;
    pcanMbbi->status = NO_ALARM;

    /* Convert the address string into members of the canIo structure */
    status = canIoParse(prec->inp.value.instio.string, &pcanMbbi->inp);
    if (status ||
	pcanMbbi->inp.parameter < 0 ||
	pcanMbbi->inp.parameter > 7) {
	if (canSilenceErrors) {
	    pcanMbbi->inp.canBusID = NULL;
	    prec->pact = TRUE;
	    return 0;
	} else {
	    recGblRecordError(S_can_badAddress, prec,
			      "devMbbiCan (init_record) bad CAN address");
	    return S_can_badAddress;
	}
    }

    #ifdef DEBUG
	printf("mbbiCan %s: Init bus=%s, id=%#x, off=%d, parm=%ld\n",
		prec->name, pcanMbbi->inp.busName, pcanMbbi->inp.identifier,
		pcanMbbi->inp.offset, pcanMbbi->inp.parameter);
    #endif

    /* For mbbi records, the final parameter specifies the input bit shift,
       with offset specifying the message byte number. */
    prec->shft = pcanMbbi->inp.parameter;
    prec->mask <<= pcanMbbi->inp.parameter;

    #ifdef DEBUG
	printf("  shft=%ld, mask=%#lx\n", 
		pcanMbbi->inp.parameter, prec->mask);
    #endif

    /* Find the bus matching this record */
    for (pbus = firstBus; pbus != NULL; pbus = pbus->nextBus) {
      if (pbus->canBusID == pcanMbbi->inp.canBusID) break;
    }  

    /* If not found, create one */
    if (pbus == NULL) {
	pbus = malloc(sizeof (mbbiCanBus_t));
	if (pbus == NULL) return S_dev_noMemory;

	/* Fill it in */
	pbus->firstPrivate = NULL;
	pbus->canBusID = pcanMbbi->inp.canBusID;
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
    pcanMbbi->nextPrivate = pbus->firstPrivate;
    pbus->firstPrivate = pcanMbbi;

    /* Set the callback parameters for asynchronous processing */
    callbackSetUser(prec, &pcanMbbi->callback);
    callbackSetCallback(ProcessCallback, &pcanMbbi->callback);
    callbackSetPriority(prec->prio, &pcanMbbi->callback);

    /* and create a timer for CANbus RTR timeouts */
    pcanMbbi->timId = epicsTimerQueueCreateTimer( canTimerQ,
		(epicsTimerCallback) callbackRequest, pcanMbbi);
    if (pcanMbbi->timId == NULL) {
	return S_dev_noMemory;
    }

    /* Register the message handler with the Canbus driver */
    canMessage(pcanMbbi->inp.canBusID, pcanMbbi->inp.identifier, 
	       mbbiMessage, pcanMbbi);

    return 0;
}

static long get_ioint_info (
    int cmd,
    struct mbbiRecord *prec,
    IOSCANPVT *ppvt
) {
    mbbiCanPrivate_t *pcanMbbi = prec->dpvt;

    if (pcanMbbi->ioscanpvt == NULL) {
	scanIoInit(&pcanMbbi->ioscanpvt);
    }

    #ifdef DEBUG
	printf("canMbbi %s: get_ioint_info %d\n", prec->name, cmd);
    #endif

    *ppvt = pcanMbbi->ioscanpvt;
    return 0;
}

static long read_mbbi (
    struct mbbiRecord *prec
) {
    mbbiCanPrivate_t *pcanMbbi = prec->dpvt;

    if (pcanMbbi->inp.canBusID == NULL) {
	return DO_NOT_CONVERT;
    }

    #ifdef DEBUG
	printf("canMbbi %s: read_mbbi status=%#x\n", prec->name, pcanMbbi->status);
    #endif

    switch (pcanMbbi->status) {
	case TIMEOUT_ALARM:
	case COMM_ALARM:
	    recGblSetSevr(prec, pcanMbbi->status, INVALID_ALARM);
	    pcanMbbi->status = NO_ALARM;
	    return DO_NOT_CONVERT;

	case NO_ALARM:
	    if (prec->pact || prec->scan == SCAN_IO_EVENT) {
		#ifdef DEBUG
		    printf("canMbbi %s: message id=%#x, data=%#lx\n", 
			    prec->name, pcanMbbi->inp.identifier, pcanMbbi->data);
		#endif

		prec->rval = pcanMbbi->data & prec->mask;
		return CONVERT;
	    } else {
		canMessage_t message;

		message.identifier = pcanMbbi->inp.identifier;
		message.rtr = RTR;
		message.length = 8;

		#ifdef DEBUG
		    printf("canMbbi %s: RTR, id=%#x\n", 
			    prec->name, pcanMbbi->inp.identifier);
		#endif

		prec->pact = TRUE;
		pcanMbbi->status = TIMEOUT_ALARM;

		epicsTimerStartDelay(pcanMbbi->timId, pcanMbbi->inp.timeout);
		canWrite(pcanMbbi->inp.canBusID, &message, pcanMbbi->inp.timeout);
		return DO_NOT_CONVERT;
	    }
	default:
	    recGblSetSevr(prec, UDF_ALARM, INVALID_ALARM);
	    pcanMbbi->status = NO_ALARM;
	    return DO_NOT_CONVERT;
    }
}

static void ProcessCallback(CALLBACK *pcallback)
{
    dbCommon *pRec;

    callbackGetUser(pRec, pcallback);
    dbScanLock(pRec);
    (*pRec->rset->process)(pRec);
    dbScanUnlock(pRec);
}

static void mbbiMessage (
    void *private,
    const canMessage_t *pmessage
) {
    mbbiCanPrivate_t *pcanMbbi = private;

    if (!interruptAccept ||
	pmessage->rtr == RTR) {
	return;
    }

    pcanMbbi->data = pmessage->data[pcanMbbi->inp.offset];

    if (pcanMbbi->prec->scan == SCAN_IO_EVENT) {
	pcanMbbi->status = NO_ALARM;
	scanIoRequest(pcanMbbi->ioscanpvt);
    } else if (pcanMbbi->status == TIMEOUT_ALARM) {
	pcanMbbi->status = NO_ALARM;
	epicsTimerCancel(pcanMbbi->timId);
	callbackRequest(&pcanMbbi->callback);
    }
}

static void busSignal (
    void *private,
    int status
) {
    mbbiCanBus_t *pbus = private;

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
    mbbiCanBus_t *pbus;
    mbbiCanPrivate_t *pcanMbbi;

    callbackGetUser(pbus, pCallback);
    pcanMbbi = pbus->firstPrivate;

    while (pcanMbbi != NULL) {
	dbCommon *prec = pcanMbbi->prec;
	pcanMbbi->status = pbus->status;
	dbScanLock(prec);
	prec->rset->process(prec);
	dbScanUnlock(prec);
	pcanMbbi = pcanMbbi->nextPrivate;
    }
}
