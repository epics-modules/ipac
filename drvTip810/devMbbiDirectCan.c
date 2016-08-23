/*******************************************************************************

Project:
    CAN Bus Driver for EPICS

File:
    devMbbiDirectCan.c

Description:
    CANBUS Multi-Bit Binary Input Direct device support

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
#include <mbbiDirectRecord.h>
#include <epicsExport.h>

#include "canBus.h"


#define CONVERT 0
#define DO_NOT_CONVERT 2


typedef struct mbbiDirectCanPrivate_s {
    CALLBACK callback;
    struct mbbiDirectCanPrivate_s *nextPrivate;
    epicsTimerId timId;
    IOSCANPVT ioscanpvt;
    dbCommon *prec;
    canIo_t inp;
    epicsUInt32 data;
    int status;
} mbbiDirectCanPrivate_t;

typedef struct mbbiDirectCanBus_s {
    CALLBACK callback;
    struct mbbiDirectCanBus_s *nextBus;
    mbbiDirectCanPrivate_t *firstPrivate;
    void *canBusID;
    int status;
} mbbiDirectCanBus_t;

static long init_mbbiDirect(struct mbbiDirectRecord *prec);
static long get_ioint_info(int cmd, struct mbbiDirectRecord *prec, IOSCANPVT *ppvt);
static long read_mbbiDirect(struct mbbiDirectRecord *prec);
static void ProcessCallback(CALLBACK *pcallback);
static void mbbiDirectMessage(void *private, const canMessage_t *pmessage);
static void busSignal(void *private, int status);
static void busCallback(CALLBACK *pCallback);

struct {
    long number;
    DEVSUPFUN report;
    DEVSUPFUN init;
    DEVSUPFUN init_record;
    DEVSUPFUN get_ioint_info;
    DEVSUPFUN read_mbbiDirect;
} devMbbiDirectCan = {
    5,
    NULL,
    NULL,
    init_mbbiDirect,
    get_ioint_info,
    read_mbbiDirect
};
epicsExportAddress(dset, devMbbiDirectCan);

static mbbiDirectCanBus_t *firstBus;


static long init_mbbiDirect (
    struct mbbiDirectRecord *prec
) {
    mbbiDirectCanPrivate_t *pcanMbbiDirect;
    mbbiDirectCanBus_t *pbus;
    int status;

    if (prec->inp.type != INST_IO) {
	recGblRecordError(S_db_badField, prec,
			  "devMbbiDirectCan (init_record) Illegal INP field");
	return S_db_badField;
    }

    pcanMbbiDirect = malloc(sizeof(mbbiDirectCanPrivate_t));
    if (pcanMbbiDirect == NULL) {
	return S_dev_noMemory;
    }
    prec->dpvt = pcanMbbiDirect;
    pcanMbbiDirect->prec = (dbCommon *) prec;
    pcanMbbiDirect->ioscanpvt = NULL;
    pcanMbbiDirect->status = NO_ALARM;

    /* Convert the address string into members of the canIo structure */
    status = canIoParse(prec->inp.value.instio.string, &pcanMbbiDirect->inp);
    if (status ||
	pcanMbbiDirect->inp.parameter < 0 ||
	pcanMbbiDirect->inp.parameter > 7) {
	if (canSilenceErrors) {
	    pcanMbbiDirect->inp.canBusID = NULL;
	    prec->pact = TRUE;
	    return 0;
	} else {
	    recGblRecordError(S_can_badAddress, prec,
			      "devMbbiDirectCan (init_record) bad CAN address");
	    return S_can_badAddress;
	}
    }

    #ifdef DEBUG
	printf("mbbiDirectCan %s: Init bus=%s, id=%#x, off=%d, parm=%ld\n",
		    prec->name, pcanMbbiDirect->inp.busName, pcanMbbiDirect->inp.identifier,
		    pcanMbbiDirect->inp.offset, pcanMbbiDirect->inp.parameter);
    #endif

    /* For mbbiDirect records, the final parameter specifies the input bit shift,
       with offset specifying the message byte number. */
    prec->shft = pcanMbbiDirect->inp.parameter;
    prec->mask <<= pcanMbbiDirect->inp.parameter;

    #ifdef DEBUG
	printf("  shft=%ld, mask=%#lx\n", 
		pcanMbbiDirect->inp.parameter, prec->mask);
    #endif

    /* Find the bus matching this record */
    for (pbus = firstBus; pbus != NULL; pbus = pbus->nextBus) {
      if (pbus->canBusID == pcanMbbiDirect->inp.canBusID) break;
    }  

    /* If not found, create one */
    if (pbus == NULL) {
      pbus = malloc(sizeof (mbbiDirectCanBus_t));
      if (pbus == NULL) return S_dev_noMemory;

      /* Fill it in */
      pbus->firstPrivate = NULL;
      pbus->canBusID = pcanMbbiDirect->inp.canBusID;
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
    pcanMbbiDirect->nextPrivate = pbus->firstPrivate;
    pbus->firstPrivate = pcanMbbiDirect;

    /* Set the callback parameters for asynchronous processing */
    callbackSetUser(prec, &pcanMbbiDirect->callback);
    callbackSetCallback(ProcessCallback, &pcanMbbiDirect->callback);
    callbackSetPriority(prec->prio, &pcanMbbiDirect->callback);

    /* and create a timer for CANbus RTR timeouts */
    pcanMbbiDirect->timId = epicsTimerQueueCreateTimer(canTimerQ,
		(epicsTimerCallback) callbackRequest, pcanMbbiDirect);
    if (pcanMbbiDirect->timId == NULL) {
	return S_dev_noMemory;
    }

    /* Register the message handler with the Canbus driver */
    canMessage(pcanMbbiDirect->inp.canBusID, pcanMbbiDirect->inp.identifier, 
		mbbiDirectMessage, pcanMbbiDirect);

    return 0;
}

static long get_ioint_info (
    int cmd,
    struct mbbiDirectRecord *prec, 
    IOSCANPVT *ppvt
) {
    mbbiDirectCanPrivate_t *pcanMbbiDirect = prec->dpvt;

    if (pcanMbbiDirect->ioscanpvt == NULL) {
	scanIoInit(&pcanMbbiDirect->ioscanpvt);
    }

    #ifdef DEBUG
	printf("canMbbiDirect %s: get_ioint_info %d\n", prec->name, cmd);
    #endif

    *ppvt = pcanMbbiDirect->ioscanpvt;
    return 0;
}

static long read_mbbiDirect (
    struct mbbiDirectRecord *prec
) {
    mbbiDirectCanPrivate_t *pcanMbbiDirect = prec->dpvt;

    if (pcanMbbiDirect->inp.canBusID == NULL) {
	return DO_NOT_CONVERT;
    }

    #ifdef DEBUG
	printf("canMbbiDirect %s: read_mbbiDirect status=%#x\n", prec->name, pcanMbbiDirect->status);
    #endif

    switch (pcanMbbiDirect->status) {
	case TIMEOUT_ALARM:
	case COMM_ALARM:
	    recGblSetSevr(prec, pcanMbbiDirect->status, INVALID_ALARM);
	    pcanMbbiDirect->status = NO_ALARM;
	    return DO_NOT_CONVERT;

	case NO_ALARM:
	    if (prec->pact || prec->scan == SCAN_IO_EVENT) {
		#ifdef DEBUG
		    printf("canMbbiDirect %s: message id=%#x, data=%#lx\n", 
			    prec->name, pcanMbbiDirect->inp.identifier, pcanMbbiDirect->data);
		#endif

		prec->rval = pcanMbbiDirect->data & prec->mask;
		return CONVERT;
	    } else {
		canMessage_t message;

		message.identifier = pcanMbbiDirect->inp.identifier;
		message.rtr = RTR;
		message.length = 8;

		#ifdef DEBUG
		    printf("canMbbiDirect %s: RTR, id=%#x\n", 
			    prec->name, pcanMbbiDirect->inp.identifier);
		#endif

		prec->pact = TRUE;
		pcanMbbiDirect->status = TIMEOUT_ALARM;

		epicsTimerStartDelay(pcanMbbiDirect->timId,
			pcanMbbiDirect->inp.timeout);
		canWrite(pcanMbbiDirect->inp.canBusID, &message,
			 pcanMbbiDirect->inp.timeout);
		return DO_NOT_CONVERT;
	    }
	default:
	    recGblSetSevr(prec, UDF_ALARM, INVALID_ALARM);
	    pcanMbbiDirect->status = NO_ALARM;
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

static void mbbiDirectMessage (
    void *private,
    const canMessage_t *pmessage
) {
    mbbiDirectCanPrivate_t *pcanMbbiDirect = private;

    if (!interruptAccept ||
	pmessage->rtr == RTR) {
	return;
    }

    pcanMbbiDirect->data = pmessage->data[pcanMbbiDirect->inp.offset];

    if (pcanMbbiDirect->prec->scan == SCAN_IO_EVENT) {
	pcanMbbiDirect->status = NO_ALARM;
	scanIoRequest(pcanMbbiDirect->ioscanpvt);
    } else if (pcanMbbiDirect->status == TIMEOUT_ALARM) {
	pcanMbbiDirect->status = NO_ALARM;
	epicsTimerCancel(pcanMbbiDirect->timId);
	callbackRequest(&pcanMbbiDirect->callback);
    }
}

static void busSignal (
    void *private,
    int status
) {
    mbbiDirectCanBus_t *pbus = private;

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
    mbbiDirectCanBus_t *pbus;
    mbbiDirectCanPrivate_t *pcanMbbiDirect;

    callbackGetUser(pbus, pCallback);
    pcanMbbiDirect = pbus->firstPrivate;

    while (pcanMbbiDirect != NULL) {
	dbCommon *prec = pcanMbbiDirect->prec;
	pcanMbbiDirect->status = pbus->status;
	dbScanLock(prec);
	prec->rset->process(pcanMbbiDirect->prec);
	dbScanUnlock(prec);
	pcanMbbiDirect = pcanMbbiDirect->nextPrivate;
    }
}
