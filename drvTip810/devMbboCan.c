/*******************************************************************************

Project:
    CAN Bus Driver for EPICS

File:
    devMbboCan.c

Description:
    CANBUS Multi-Bit Binary Output device support

Author:
    Andrew Johnson <anjohnson@iee.org>
Created:
    14 August 1995
Version:
    $Id: devMbboCan.c,v 1.11 2003-05-28 12:20:21 mrk Exp $

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
#include <mbboRecord.h>
#include <canBus.h>
#include <alarm.h>
#include <recGbl.h>


#define DO_NOT_CONVERT	2


typedef struct mbboCanPrivate_s {
    struct mbboCanPrivate_s *nextPrivate;
    IOSCANPVT ioscanpvt;
    struct mbboRecord *prec;
    canIo_t out;
    long data;
    int status;
} mbboCanPrivate_t;

typedef struct mbboCanBus_s {
    CALLBACK callback;		/* This *must* be first member */
    struct mbboCanBus_s *nextBus;
    mbboCanPrivate_t *firstPrivate;
    void *canBusID;
    int status;
} mbboCanBus_t;

LOCAL long init_mbbo(struct mbboRecord *prec);
LOCAL long get_ioint_info(int cmd, struct mbboRecord *prec, IOSCANPVT *ppvt);
LOCAL long write_mbbo(struct mbboRecord *prec);
LOCAL void mbboMessage(mbboCanPrivate_t *pcanMbbo, canMessage_t *pmessage);
LOCAL void busSignal(mbboCanBus_t *pbus, int status);
LOCAL void busCallback(mbboCanBus_t *pbus);

struct {
    long number;
    DEVSUPFUN report;
    DEVSUPFUN init;
    DEVSUPFUN init_record;
    DEVSUPFUN get_ioint_info;
    DEVSUPFUN write_mbbo;
} devMbboCan = {
    5,
    NULL,
    NULL,
    init_mbbo,
    get_ioint_info,
    write_mbbo
};

LOCAL mbboCanBus_t *firstBus;


LOCAL long init_mbbo (
    struct mbboRecord *prec
) {
    mbboCanPrivate_t *pcanMbbo;
    mbboCanBus_t *pbus;
    int status;

    if (prec->out.type != INST_IO) {
	recGblRecordError(S_db_badField, (void *) prec,
			  "devMbboCan (init_record) Illegal OUT field");
	return S_db_badField;
    }

    pcanMbbo = (mbboCanPrivate_t *) malloc(sizeof(mbboCanPrivate_t));
    if (pcanMbbo == NULL) {
	return S_dev_noMemory;
    }
    prec->dpvt = pcanMbbo;
    pcanMbbo->prec = prec;
    pcanMbbo->ioscanpvt = NULL;
    pcanMbbo->status = NO_ALARM;

    /* Convert the parameter string into members of the canIo structure */
    status = canIoParse(prec->out.value.instio.string, &pcanMbbo->out);
    if (status ||
	pcanMbbo->out.parameter < 0 ||
	pcanMbbo->out.parameter > 7) {
	if (canSilenceErrors) {
	    pcanMbbo->out.canBusID = NULL;
	    prec->pact = TRUE;
	    return DO_NOT_CONVERT;
	} else {
	    recGblRecordError(S_can_badAddress, (void *) prec,
			      "devMbboCan (init_record) bad CAN address");
	    return S_can_badAddress;
	}
    }

    #ifdef DEBUG
	printf("canMbbo %s: Init bus=%s, id=%#x, off=%d, parm=%ld\n",
		    prec->name, pcanMbbo->out.busName, pcanMbbo->out.identifier,
		    pcanMbbo->out.offset, pcanMbbo->out.parameter);
    #endif

    /* For mbbo records, the final parameter specifies the output bit shift,
       with the offset specifying the message byte number. */
    prec->shft = pcanMbbo->out.parameter;
    prec->mask <<= pcanMbbo->out.parameter;

    #ifdef DEBUG
	printf("  bit=%ld, mask=%#lx\n", pcanMbbo->out.parameter, prec->mask);
    #endif

    /* Find the bus matching this record */
    for (pbus = firstBus; pbus != NULL; pbus = pbus->nextBus) {
    	if (pbus->canBusID == pcanMbbo->out.canBusID) break;
    }
    
    /* If not found, create one */
    if (pbus == NULL) {
    	pbus = malloc(sizeof (mbboCanBus_t));
    	if (pbus == NULL) return S_dev_noMemory;
    	
    	/* Fill it in */
    	pbus->firstPrivate = NULL;
    	pbus->canBusID = pcanMbbo->out.canBusID;
    	callbackSetCallback((VOIDFUNCPTR) busCallback, &pbus->callback);
    	callbackSetPriority(priorityMedium, &pbus->callback);
    	
    	/* and add it to the list of busses we know about */
    	pbus->nextBus = firstBus;
    	firstBus = pbus;
    	
    	/* Ask driver for error signals */
    	canSignal(pbus->canBusID, (canSigCallback_t *) busSignal, pbus);
    }
    
    /* Insert private record structure into linked list for this CANbus */
    pcanMbbo->nextPrivate = pbus->firstPrivate;
    pbus->firstPrivate = pcanMbbo;

    /* Register the message handler with the Canbus driver */
    canMessage(pcanMbbo->out.canBusID, pcanMbbo->out.identifier, 
	       (canMsgCallback_t *) mbboMessage, pcanMbbo);

    return DO_NOT_CONVERT;
}

LOCAL long get_ioint_info (
    int cmd,
    struct mbboRecord *prec, 
    IOSCANPVT *ppvt
) {
    mbboCanPrivate_t *pcanMbbo = (mbboCanPrivate_t *) prec->dpvt;

    if (pcanMbbo->ioscanpvt == NULL) {
	scanIoInit(&pcanMbbo->ioscanpvt);
    }

    #ifdef DEBUG
	printf("mbboCan %s: get_ioint_info %d\n", prec->name, cmd);
    #endif

    *ppvt = pcanMbbo->ioscanpvt;
    return OK;
}

LOCAL long write_mbbo (
    struct mbboRecord *prec
) {
    mbboCanPrivate_t *pcanMbbo = (mbboCanPrivate_t *) prec->dpvt;

    if (pcanMbbo->out.canBusID == NULL) {
	return ERROR;
    }

    #ifdef DEBUG
	printf("mbboCan %s: write_mbbo status=%#x\n", prec->name, pcanMbbo->status);
    #endif

    switch (pcanMbbo->status) {
	case COMM_ALARM:
	    recGblSetSevr(prec, pcanMbbo->status, INVALID_ALARM);
	    pcanMbbo->status = NO_ALARM;
	    return ERROR;

	case NO_ALARM:
	    {
		canMessage_t message;
		int status;

		message.identifier = pcanMbbo->out.identifier;
		message.rtr = SEND;

		pcanMbbo->data = prec->rval & prec->mask;

		message.data[pcanMbbo->out.offset] = pcanMbbo->data;
		message.length  = pcanMbbo->out.offset + 1;

		#ifdef DEBUG
		    printf("canMbbo %s: SEND id=%#x, length=%d, data=%#lx\n", 
			    prec->name, message.identifier, message.length, 
			    pcanMbbo->data);
		#endif

		status = canWrite(pcanMbbo->out.canBusID, &message, 
				  pcanMbbo->out.timeout);
		if (status) {
		    #ifdef DEBUG
			printf("canMbbo %s: canWrite status=%#x\n",
				prec->name, status);
		    #endif

		    recGblSetSevr(prec, TIMEOUT_ALARM, INVALID_ALARM);
		    return ERROR;
		}
		return OK;
	    }
	default:
	    recGblSetSevr(prec, UDF_ALARM, INVALID_ALARM);
	    pcanMbbo->status = NO_ALARM;
	    return ERROR;
    }
}

LOCAL void mbboMessage (
    mbboCanPrivate_t *pcanMbbo,
    canMessage_t *pmessage
) {
    if (!interruptAccept) return;
    
    if (pcanMbbo->prec->scan == SCAN_IO_EVENT &&
	pmessage->rtr == RTR) {
	pcanMbbo->status = NO_ALARM;
	scanIoRequest(pcanMbbo->ioscanpvt);
    }
}

LOCAL void busSignal (
    mbboCanBus_t *pbus,
    int status
) {
    if (!interruptAccept) return;
    
    switch(status) {
	case CAN_BUS_OK:
	    logMsg("devMbboCan: Bus Ok event from %s\n",
	    	   (int) pbus->firstPrivate->out.busName, 0, 0, 0, 0, 0);
	    pbus->status = NO_ALARM;
	    break;
	case CAN_BUS_ERROR:
	    logMsg("devMbboCan: Bus Error event from %s\n",
	    	   (int) pbus->firstPrivate->out.busName, 0, 0, 0, 0, 0);
	    pbus->status = COMM_ALARM;
	    callbackRequest(&pbus->callback);
	    break;
	case CAN_BUS_OFF:
	    logMsg("devMbboCan: Bus Off event from %s\n",
	    	   (int) pbus->firstPrivate->out.busName, 0, 0, 0, 0, 0);
	    pbus->status = COMM_ALARM;
	    callbackRequest(&pbus->callback);
	    break;
    }
}

LOCAL void busCallback (
    mbboCanBus_t *pbus
) {
    mbboCanPrivate_t *pcanMbbo = pbus->firstPrivate;
    
    while (pcanMbbo != NULL) {
	pcanMbbo->status = pbus->status;
	dbScanLock((struct dbCommon *) pcanMbbo->prec);
	(*((struct rset *) pcanMbbo->prec->rset)->process)(pcanMbbo->prec);
	dbScanUnlock((struct dbCommon *) pcanMbbo->prec);
	pcanMbbo = pcanMbbo->nextPrivate;
    }
}
