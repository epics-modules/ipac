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
Version:
    $Id: devMbbiDirectCan.c,v 1.13 2003-05-28 12:20:21 mrk Exp $

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


#include <vxWorks.h>
#include <stdio.h>
#include <stdlib.h>
#include <wdLib.h>
#include <logLib.h>

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
#include <canBus.h>
#include <alarm.h>
#include <recGbl.h>


#define CONVERT 0
#define DO_NOT_CONVERT 2


typedef struct mbbiDirectCanPrivate_s {
    CALLBACK callback;		/* This *must* be first member */
    struct mbbiDirectCanPrivate_s *nextPrivate;
    WDOG_ID wdId;
    IOSCANPVT ioscanpvt;
    struct mbbiDirectRecord *prec;
    canIo_t inp;
    long data;
    int status;
} mbbiDirectCanPrivate_t;

typedef struct mbbiDirectCanBus_s {
    CALLBACK callback;		/* This *must* be first member */
    struct mbbiDirectCanBus_s *nextBus;
    mbbiDirectCanPrivate_t *firstPrivate;
    void *canBusID;
    int status;
} mbbiDirectCanBus_t;

LOCAL long init_mbbiDirect(struct mbbiDirectRecord *prec);
LOCAL long get_ioint_info(int cmd, struct mbbiDirectRecord *prec, IOSCANPVT *ppvt);
LOCAL long read_mbbiDirect(struct mbbiDirectRecord *prec);
LOCAL void mbbiDirectProcess(mbbiDirectCanPrivate_t *pcanMbbiDirect);
LOCAL void mbbiDirectMessage(mbbiDirectCanPrivate_t *pcanMbbiDirect, canMessage_t *pmessage);
LOCAL void busSignal(mbbiDirectCanBus_t *pbus, int status);
LOCAL void busCallback(mbbiDirectCanBus_t *pbus);

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

LOCAL mbbiDirectCanBus_t *firstBus;


LOCAL long init_mbbiDirect (
    struct mbbiDirectRecord *prec
) {
    mbbiDirectCanPrivate_t *pcanMbbiDirect;
    mbbiDirectCanBus_t *pbus;
    int status;
    
    if (prec->inp.type != INST_IO) {
	recGblRecordError(S_db_badField, (void *) prec,
			  "devMbbiDirectCan (init_record) Illegal INP field");
	return S_db_badField;
    }

    pcanMbbiDirect = (mbbiDirectCanPrivate_t *) malloc(sizeof(mbbiDirectCanPrivate_t));
    if (pcanMbbiDirect == NULL) {
	return S_dev_noMemory;
    }
    prec->dpvt = pcanMbbiDirect;
    pcanMbbiDirect->prec = prec;
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
	    return OK;
	} else {
	    recGblRecordError(S_can_badAddress, (void *) prec,
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
      callbackSetCallback((VOIDFUNCPTR) busCallback, &pbus->callback);
      callbackSetPriority(priorityMedium, &pbus->callback);

      /* and add it to the list of busses we know about */
      pbus->nextBus = firstBus;
      firstBus = pbus;

      /* Ask driver for error signals */
      canSignal(pbus->canBusID, (canSigCallback_t *) busSignal, pbus);
    }  

    /* Insert private record structure into linked list for this CANbus */
    pcanMbbiDirect->nextPrivate = pbus->firstPrivate;
    pbus->firstPrivate = pcanMbbiDirect;

    /* Set the callback parameters for asynchronous processing */
    callbackSetCallback((VOIDFUNCPTR) mbbiDirectProcess, &pcanMbbiDirect->callback);
    callbackSetPriority(prec->prio, &pcanMbbiDirect->callback);

    /* and create a watchdog for CANbus RTR timeouts */
    pcanMbbiDirect->wdId = wdCreate();
    if (pcanMbbiDirect->wdId == NULL) {
	return S_dev_noMemory;
    }

    /* Register the message handler with the Canbus driver */
    canMessage(pcanMbbiDirect->inp.canBusID, pcanMbbiDirect->inp.identifier, 
	       (canMsgCallback_t *) mbbiDirectMessage, pcanMbbiDirect);

    return OK;
}

LOCAL long get_ioint_info (
    int cmd,
    struct mbbiDirectRecord *prec, 
    IOSCANPVT *ppvt
) {
    mbbiDirectCanPrivate_t *pcanMbbiDirect = (mbbiDirectCanPrivate_t *) prec->dpvt;

    if (pcanMbbiDirect->ioscanpvt == NULL) {
	scanIoInit(&pcanMbbiDirect->ioscanpvt);
    }

    #ifdef DEBUG
	printf("canMbbiDirect %s: get_ioint_info %d\n", prec->name, cmd);
    #endif

    *ppvt = pcanMbbiDirect->ioscanpvt;
    return OK;
}

LOCAL long read_mbbiDirect (
    struct mbbiDirectRecord *prec
) {
    mbbiDirectCanPrivate_t *pcanMbbiDirect = (mbbiDirectCanPrivate_t *) prec->dpvt;

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

		callbackSetPriority(prec->prio, &pcanMbbiDirect->callback);
		wdStart(pcanMbbiDirect->wdId, pcanMbbiDirect->inp.timeout, 
			(FUNCPTR) callbackRequest, (int) pcanMbbiDirect);
		canWrite(pcanMbbiDirect->inp.canBusID, &message, pcanMbbiDirect->inp.timeout);
		return DO_NOT_CONVERT;
	    }
	default:
	    recGblSetSevr(prec, UDF_ALARM, INVALID_ALARM);
	    pcanMbbiDirect->status = NO_ALARM;
	    return DO_NOT_CONVERT;
    }
}

LOCAL void mbbiDirectProcess (
    mbbiDirectCanPrivate_t *pcanMbbiDirect
) {
    dbScanLock((struct dbCommon *) pcanMbbiDirect->prec);
    (*((struct rset *) pcanMbbiDirect->prec->rset)->process)(pcanMbbiDirect->prec);
    dbScanUnlock((struct dbCommon *) pcanMbbiDirect->prec);
}

LOCAL void mbbiDirectMessage (
    mbbiDirectCanPrivate_t *pcanMbbiDirect,
    canMessage_t *pmessage
) {
    if (!interruptAccept) return;
    
    if (pmessage->rtr == RTR) {
	return;		/* Ignore RTRs */
    }

    pcanMbbiDirect->data = pmessage->data[pcanMbbiDirect->inp.offset];

    if (pcanMbbiDirect->prec->scan == SCAN_IO_EVENT) {
	pcanMbbiDirect->status = NO_ALARM;
	scanIoRequest(pcanMbbiDirect->ioscanpvt);
    } else if (pcanMbbiDirect->status == TIMEOUT_ALARM) {
	pcanMbbiDirect->status = NO_ALARM;
	wdCancel(pcanMbbiDirect->wdId);
	callbackRequest(&pcanMbbiDirect->callback);
    }
}

LOCAL void busSignal (
    mbbiDirectCanBus_t *pbus,
    int status
) {
    if (!interruptAccept) return;
    
    switch(status) {
	case CAN_BUS_OK:
	    logMsg("devMbbiDirectCan: Bus Ok event from %s\n",
		   (int) pbus->firstPrivate->inp.busName, 0, 0, 0, 0, 0);
		pbus->status = NO_ALARM;
	    break;
	case CAN_BUS_ERROR:
	    logMsg("devMbbiDirectCan: Bus Error event from %s\n",
		   (int) pbus->firstPrivate->inp.busName, 0, 0, 0, 0, 0);
	    pbus->status = COMM_ALARM;
	    callbackRequest(&pbus->callback);
	    break;
	case CAN_BUS_OFF:
	    logMsg("devMbbiDirectCan: Bus Off event from %s\n",
		   (int) pbus->firstPrivate->inp.busName, 0, 0, 0, 0, 0);
	    pbus->status = COMM_ALARM;
	    callbackRequest(&pbus->callback);
	    break;
    }
}

LOCAL void busCallback (
    mbbiDirectCanBus_t *pbus
) {
    mbbiDirectCanPrivate_t *pcanMbbiDirect = pbus->firstPrivate;
    
    while (pcanMbbiDirect != NULL) {
	pcanMbbiDirect->status = pbus->status;
	mbbiDirectProcess(pcanMbbiDirect);
	pcanMbbiDirect = pcanMbbiDirect->nextPrivate;
    }
}
