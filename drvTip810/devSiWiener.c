/*******************************************************************************

Project:
    CAN Bus Driver for EPICS

File:
    devSiCan.c

Description:
    CANBUS String Input device support for Wiener VME crate - NOT a general-
    purpose stringin device support.

Authors:
    Carl Lionberger and Andrew Johnson

Created:
    25 August 1998
Version:
    $Id: devSiWiener.c,v 1.2 2002-04-17 19:30:50 anj Exp $


Copyright (c) 1995-2000 Carl Lionberger and Andrew Johnson

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
#include <recSup.h>
#include <devSup.h>
#include <dbCommon.h>
#include <stringinRecord.h>
#include <canBus.h>
#include <string.h>


typedef struct siCanPrivate_s {
    CALLBACK callback;		/* This *must* be first member */
    struct siCanPrivate_s *nextPrivate;
    WDOG_ID wdId;
    IOSCANPVT ioscanpvt;
    struct stringinRecord *prec;
    canIo_t inp;
    char data[CAN_DATA_SIZE + 1];
    int status;
} siCanPrivate_t;

typedef struct siCanBus_s {
    CALLBACK callback;		/* This *must* be first member */
    struct siCanBus_s *nextBus;
    siCanPrivate_t *firstPrivate;
    void *canBusID;
    int status;
} siCanBus_t;

LOCAL long init_si(struct stringinRecord *prec);
LOCAL long get_ioint_info(int cmd, struct stringinRecord *prec, IOSCANPVT *ppvt);
LOCAL long read_si(struct stringinRecord *prec);
LOCAL void siProcess(siCanPrivate_t *pcanSi);
LOCAL void siMessage(siCanPrivate_t *pcanSi, canMessage_t *pmessage);
LOCAL void busSignal(siCanBus_t *pbus, int status);
LOCAL void busCallback(siCanBus_t *pbus);

struct {
    long number;
    DEVSUPFUN report;
    DEVSUPFUN init;
    DEVSUPFUN init_record;
    DEVSUPFUN get_ioint_info;
    DEVSUPFUN read_si;
} devSiWiener = {
    5,
    NULL,
    NULL,
    init_si,
    get_ioint_info,
    read_si
};

LOCAL siCanBus_t *firstBus;

LOCAL long init_si (
    struct stringinRecord *prec
) {
    siCanPrivate_t *pcanSi;
    siCanBus_t *pbus;
    int status;

    if (prec->inp.type != INST_IO) {
	recGblRecordError(S_db_badField, (void *) prec,
			  "devSiCan (init_record) Illegal INP field");
	return S_db_badField;
    }

    pcanSi = (siCanPrivate_t *) malloc(sizeof(siCanPrivate_t));
    if (pcanSi == NULL) {
	return S_dev_noMemory;
    }
    prec->dpvt = pcanSi;
    pcanSi->prec = prec;
    pcanSi->ioscanpvt = NULL;
    pcanSi->status = NO_ALARM;

    /* Convert the address string into members of the canIo structure */
    status = canIoParse(prec->inp.value.instio.string, &pcanSi->inp);
    if (status) {
	if (canSilenceErrors) {
	    pcanSi->inp.canBusID = NULL;
	    prec->pact = TRUE;
	    return OK;
	} else {
	    recGblRecordError(S_can_badAddress, (void *) prec,
			      "devSiCan (init_record) bad CAN address");
	    return S_can_badAddress;
	}
    }

    #ifdef DEBUG
	printf("siCan %s: Init bus=%s, id=%#x, off=%d, parm=%ld\n",
		    prec->name, pcanSi->inp.busName, pcanSi->inp.identifier,
		    pcanSi->inp.offset, pcanSi->inp.parameter);
    #endif

    /* Find the bus matching this record */
    for (pbus = firstBus; pbus != NULL; pbus = pbus->nextBus) {
    	if (pbus->canBusID == pcanSi->inp.canBusID) break;
    }
    
    /* If not found, create one */
    if (pbus == NULL) {
    	pbus = malloc(sizeof (siCanBus_t));
    	if (pbus == NULL) return S_dev_noMemory;
    	
    	/* Fill it in */
    	pbus->firstPrivate = NULL;
    	pbus->canBusID = pcanSi->inp.canBusID;
    	callbackSetCallback((CALLBACKFUNC)busCallback, &pbus->callback);
    	callbackSetPriority(priorityMedium, &pbus->callback);
    	
    	/* and add it to the list of busses we know about */
    	pbus->nextBus = firstBus;
    	firstBus = pbus;
    	
    	/* Ask driver for error signals */
    	canSignal(pbus->canBusID, (canSigCallback_t *) busSignal, pbus);
    }
    
    /* Insert private record structure into linked list for this CANbus */
    pcanSi->nextPrivate = pbus->firstPrivate;
    pbus->firstPrivate = pcanSi;

    /* Set the callback parameters for asynchronous processing */
    callbackSetCallback((CALLBACKFUNC)siProcess, &pcanSi->callback);
    callbackSetPriority(prec->prio, &pcanSi->callback);

    /* and create a watchdog for CANbus RTR timeouts */
    pcanSi->wdId = wdCreate();
    if (pcanSi->wdId == NULL) {
	return S_dev_noMemory;
    }

    /* Register the message handler with the Canbus driver */
    canMessage(pcanSi->inp.canBusID, pcanSi->inp.identifier, 
	       (canMsgCallback_t *) siMessage, pcanSi);

    return OK;
}

LOCAL long get_ioint_info (
    int cmd,
    struct stringinRecord *prec, 
    IOSCANPVT *ppvt
) {
    siCanPrivate_t *pcanSi = (siCanPrivate_t *) prec->dpvt;

    if (pcanSi->ioscanpvt == NULL) {
	scanIoInit(&pcanSi->ioscanpvt);
    }

    #ifdef DEBUG
	printf("canSi %s: get_ioint_info %d\n", prec->name, cmd);
    #endif

    *ppvt = pcanSi->ioscanpvt;
    return OK;
}

LOCAL long read_si (
    struct stringinRecord *prec
) {
    siCanPrivate_t *pcanSi = (siCanPrivate_t *) prec->dpvt;

    if (pcanSi->inp.canBusID == NULL) {
	return ERROR;
    }

    #ifdef DEBUG
	printf("canSi %s: read_si status=%#x\n", prec->name, pcanSi->status);
    #endif

    switch (pcanSi->status) {
	case TIMEOUT_ALARM:
	case COMM_ALARM:
	    recGblSetSevr(prec, pcanSi->status, INVALID_ALARM);
	    pcanSi->status = NO_ALARM;
	    return ERROR;

	case NO_ALARM:
	    if (prec->pact || prec->scan == SCAN_IO_EVENT) {
		#ifdef DEBUG
		    printf("canSi %s: message id=%#x, data=%p\n", 
			    prec->name, pcanSi->inp.identifier, pcanSi->data);
		#endif

                strcpy(prec->val, pcanSi->data);
		return OK;
	    } else {
		canMessage_t message;

		message.identifier = pcanSi->inp.identifier;
		message.rtr = RTR;
		message.length = 8;

		#ifdef DEBUG
		    printf("canSi %s: RTR, id=%#x\n", 
			    prec->name, pcanSi->inp.identifier);
		#endif

		prec->pact = TRUE;
		pcanSi->status = TIMEOUT_ALARM;

		callbackSetPriority(prec->prio, &pcanSi->callback);
		wdStart(pcanSi->wdId, pcanSi->inp.timeout, 
			(FUNCPTR) callbackRequest, (int) pcanSi);
		canWrite(pcanSi->inp.canBusID, &message, pcanSi->inp.timeout);
		return OK;
	    }
	default:
	    recGblSetSevr(prec, UDF_ALARM, INVALID_ALARM);
	    pcanSi->status = NO_ALARM;
	    return ERROR;
    }
}

LOCAL void siProcess (
    siCanPrivate_t *pcanSi
) {
    dbScanLock((struct dbCommon *) pcanSi->prec);
    (*((struct rset *) pcanSi->prec->rset)->process)(pcanSi->prec);
    dbScanUnlock((struct dbCommon *) pcanSi->prec);
}

LOCAL void siMessage (
    siCanPrivate_t *pcanSi,
    canMessage_t *pmessage
) {
    if (!interruptAccept) return;

    if (pmessage->rtr == RTR) {
	return;		/* Ignore RTRs */
    }

    if ((pcanSi->inp.offset == 1) &&
	(pcanSi->inp.parameter != pmessage->data[0]))
        return;		/* Subaddressing ala wiener, but wrong one. */

    memcpy(pcanSi->data, pmessage->data + pcanSi->inp.offset, 
                         CAN_DATA_SIZE - pcanSi->inp.offset);
    pcanSi->data[8 - pcanSi->inp.offset] = '\0';

    if (pcanSi->prec->scan == SCAN_IO_EVENT) {
	pcanSi->status = NO_ALARM;
	scanIoRequest(pcanSi->ioscanpvt);
    } else if (pcanSi->status == TIMEOUT_ALARM) {
	pcanSi->status = NO_ALARM;
	wdCancel(pcanSi->wdId);
	callbackRequest(&pcanSi->callback);
    }
}

LOCAL void busSignal (
    siCanBus_t *pbus,
    int status
) {
    if (!interruptAccept) return;
    
    switch(status) {
	case CAN_BUS_OK:
	    logMsg("devSiCan: Bus Ok event from %s\n", 
	    	   (int) pbus->firstPrivate->inp.busName, 0, 0, 0, 0, 0);
	    pbus->status = NO_ALARM;
	    break;
	case CAN_BUS_ERROR:
	    logMsg("devSiCan: Bus Error event from %s\n", 
	    	   (int) pbus->firstPrivate->inp.busName, 0, 0, 0, 0, 0);
	    pbus->status = COMM_ALARM;
	    callbackRequest(&pbus->callback);
	    break;
	case CAN_BUS_OFF:
	    logMsg("devSiCan: Bus Off event from %s\n", 
	    	   (int) pbus->firstPrivate->inp.busName, 0, 0, 0, 0, 0);
	    pbus->status = COMM_ALARM;
	    callbackRequest(&pbus->callback);
	    break;
    }
}

LOCAL void busCallback (
    siCanBus_t *pbus
) {
    siCanPrivate_t *pcanSi = pbus->firstPrivate;
    
    while (pcanSi != NULL) {
	pcanSi->status = pbus->status;
	siProcess(pcanSi);
	pcanSi = pcanSi->nextPrivate;
    }
}
