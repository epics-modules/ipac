/*******************************************************************************

Project:
    CAN Bus Driver for EPICS

File:
    devMbboDirectCan.c

Description:
    CANBUS Multi-Bit Binary Output Direct device support

Author:
    Andrew Johnson <anjohnson@iee.org>
Created:
    14 August 1995
Version:
    $Id: devMbboDirectCan.c,v 1.11 2003-05-28 12:20:21 mrk Exp $

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
#include <alarm.h>
#include <recGbl.h>
#include <link.h>
#include <recSup.h>
#include <devSup.h>
#include <dbCommon.h>
#include <mbboDirectRecord.h>
#include <canBus.h>
#include <alarm.h>
#include <recGbl.h>


#define DO_NOT_CONVERT	2


typedef struct mbboDirectCanPrivate_s {
    struct mbboDirectCanPrivate_s *nextPrivate;
    IOSCANPVT ioscanpvt;
    struct mbboDirectRecord *prec;
    canIo_t out;
    long data;
    int status;
} mbboDirectCanPrivate_t;

typedef struct mbboDirectCanBus_s {
    CALLBACK callback;		/* This *must* be first member */
    struct mbboDirectCanBus_s *nextBus;
    mbboDirectCanPrivate_t *firstPrivate;
    void *canBusID;
    int status;
} mbboDirectCanBus_t;

LOCAL long init_mbboDirect(struct mbboDirectRecord *prec);
LOCAL long get_ioint_info(int cmd, struct mbboDirectRecord *prec, IOSCANPVT *ppvt);
LOCAL long write_mbboDirect(struct mbboDirectRecord *prec);
LOCAL void mbboDirectMessage(mbboDirectCanPrivate_t *pcanMbboDirect, canMessage_t *pmessage);
LOCAL void busSignal(mbboDirectCanBus_t *pbus, int status);
LOCAL void busCallback(mbboDirectCanBus_t *pbus);

struct {
    long number;
    DEVSUPFUN report;
    DEVSUPFUN init;
    DEVSUPFUN init_record;
    DEVSUPFUN get_ioint_info;
    DEVSUPFUN write_mbboDirect;
} devMbboDirectCan = {
    5,
    NULL,
    NULL,
    init_mbboDirect,
    get_ioint_info,
    write_mbboDirect
};

LOCAL mbboDirectCanBus_t *firstBus;


LOCAL long init_mbboDirect (
    struct mbboDirectRecord *prec
) {
    mbboDirectCanPrivate_t *pcanMbboDirect;
    mbboDirectCanBus_t *pbus;
    int status;

    if (prec->out.type != INST_IO) {
	recGblRecordError(S_db_badField, (void *) prec,
			  "devMbboDirectCan (init_record) Illegal OUT field");
	return S_db_badField;
    }

    pcanMbboDirect = (mbboDirectCanPrivate_t *) malloc(sizeof(mbboDirectCanPrivate_t));
    if (pcanMbboDirect == NULL) {
	return S_dev_noMemory;
    }
    prec->dpvt = pcanMbboDirect;
    pcanMbboDirect->prec = prec;
    pcanMbboDirect->ioscanpvt = NULL;
    pcanMbboDirect->status = NO_ALARM;

    /* Convert the parameter string into members of the canIo structure */
    status = canIoParse(prec->out.value.instio.string, &pcanMbboDirect->out);
    if (status ||
	pcanMbboDirect->out.parameter < 0 ||
	pcanMbboDirect->out.parameter > 7) {
	if (canSilenceErrors) {
	    pcanMbboDirect->out.canBusID = NULL;
	    prec->pact = TRUE;
	    return DO_NOT_CONVERT;
	} else {
	    recGblRecordError(S_can_badAddress, (void *) prec,
			      "devMbboDirectCan (init_record) bad CAN address");
	    return S_can_badAddress;
	}
    }

    #ifdef DEBUG
	printf("canMbboDirect %s: Init bus=%s, id=%#x, off=%d, parm=%ld\n",
		    prec->name, pcanMbboDirect->out.busName, pcanMbboDirect->out.identifier,
		    pcanMbboDirect->out.offset, pcanMbboDirect->out.parameter);
    #endif

    /* For mbboDirect records, the final parameter specifies the output bit shift,
       with the offset specifying the message byte number. */
    prec->shft = pcanMbboDirect->out.parameter;
    prec->mask <<= pcanMbboDirect->out.parameter;

    #ifdef DEBUG
	printf("  bit=%ld, mask=%#lx\n", pcanMbboDirect->out.parameter, prec->mask);
    #endif

    /* Find the bus matching this record */
    for (pbus = firstBus; pbus != NULL; pbus = pbus->nextBus) {
    	if (pbus->canBusID == pcanMbboDirect->out.canBusID) break;
    }
    
    /* If not found, create one */
    if (pbus == NULL) {
    	pbus = malloc(sizeof (mbboDirectCanBus_t));
    	if (pbus == NULL) return S_dev_noMemory;
    	
    	/* Fill it in */
    	pbus->firstPrivate = NULL;
    	pbus->canBusID = pcanMbboDirect->out.canBusID;
    	callbackSetCallback((VOIDFUNCPTR) busCallback, &pbus->callback);
    	callbackSetPriority(priorityMedium, &pbus->callback);
    	
    	/* and add it to the list of busses we know about */
    	pbus->nextBus = firstBus;
    	firstBus = pbus;
    	
    	/* Ask driver for error signals */
    	canSignal(pbus->canBusID, (canSigCallback_t *) busSignal, pbus);
    }
    
    /* Insert private record structure into linked list for this CANbus */
    pcanMbboDirect->nextPrivate = pbus->firstPrivate;
    pbus->firstPrivate = pcanMbboDirect;

    /* Register the message handler with the Canbus driver */
    canMessage(pcanMbboDirect->out.canBusID, pcanMbboDirect->out.identifier, 
	       (canMsgCallback_t *) mbboDirectMessage, pcanMbboDirect);

    return DO_NOT_CONVERT;
}

LOCAL long get_ioint_info (
    int cmd,
    struct mbboDirectRecord *prec, 
    IOSCANPVT *ppvt
) {
    mbboDirectCanPrivate_t *pcanMbboDirect = (mbboDirectCanPrivate_t *) prec->dpvt;

    if (pcanMbboDirect->ioscanpvt == NULL) {
	scanIoInit(&pcanMbboDirect->ioscanpvt);
    }

    #ifdef DEBUG
	printf("mbboDirectCan %s: get_ioint_info %d\n", prec->name, cmd);
    #endif

    *ppvt = pcanMbboDirect->ioscanpvt;
    return OK;
}

LOCAL long write_mbboDirect (
    struct mbboDirectRecord *prec
) {
    mbboDirectCanPrivate_t *pcanMbboDirect = (mbboDirectCanPrivate_t *) prec->dpvt;

    if (pcanMbboDirect->out.canBusID == NULL) {
	return ERROR;
    }

    #ifdef DEBUG
	printf("mbboDirectCan %s: write_mbboDirect status=%#x\n", prec->name, pcanMbboDirect->status);
    #endif

    switch (pcanMbboDirect->status) {
	case COMM_ALARM:
	    recGblSetSevr(prec, pcanMbboDirect->status, INVALID_ALARM);
	    pcanMbboDirect->status = NO_ALARM;
	    return ERROR;

	case NO_ALARM:
	    {
		canMessage_t message;
		int status;

		message.identifier = pcanMbboDirect->out.identifier;
		message.rtr = SEND;

		pcanMbboDirect->data = prec->rval & prec->mask;

		message.data[pcanMbboDirect->out.offset] = pcanMbboDirect->data;
		message.length  = pcanMbboDirect->out.offset + 1;

		#ifdef DEBUG
		    printf("canMbboDirect %s: SEND id=%#x, length=%d, data=%#lx\n", 
			    prec->name, message.identifier, message.length, 
			    pcanMbboDirect->data);
		#endif

		status = canWrite(pcanMbboDirect->out.canBusID, &message, 
				  pcanMbboDirect->out.timeout);
		if (status) {
		    #ifdef DEBUG
			printf("canMbboDirect %s: canWrite status=%#x\n",
				prec->name, status);
		    #endif

		    recGblSetSevr(prec, TIMEOUT_ALARM, INVALID_ALARM);
		    return ERROR;
		}
		return OK;
	    }
	default:
	    recGblSetSevr(prec, UDF_ALARM, INVALID_ALARM);
	    pcanMbboDirect->status = NO_ALARM;
	    return ERROR;
    }
}

LOCAL void mbboDirectMessage (
    mbboDirectCanPrivate_t *pcanMbboDirect,
    canMessage_t *pmessage
) {
    if (!interruptAccept) return;
    
    if (pcanMbboDirect->prec->scan == SCAN_IO_EVENT &&
	pmessage->rtr == RTR) {
	pcanMbboDirect->status = NO_ALARM;
	scanIoRequest(pcanMbboDirect->ioscanpvt);
    }
}

LOCAL void busSignal (
    mbboDirectCanBus_t *pbus,
    int status
) {
    if (!interruptAccept) return;
    
    switch(status) {
	case CAN_BUS_OK:
	    logMsg("devMbboDirectCan: Bus Ok event from %s\n",
	    	   (int) pbus->firstPrivate->out.busName, 0, 0, 0, 0, 0);
	    pbus->status = NO_ALARM;
	    break;
	case CAN_BUS_ERROR:
	    logMsg("devMbboDirectCan: Bus Error event from %s\n",
	    	   (int) pbus->firstPrivate->out.busName, 0, 0, 0, 0, 0);
	    pbus->status = COMM_ALARM;
	    callbackRequest(&pbus->callback);
	    break;
	case CAN_BUS_OFF:
	    logMsg("devMbboDirectCan: Bus Off event from %s\n",
	    	   (int) pbus->firstPrivate->out.busName, 0, 0, 0, 0, 0);
	    pbus->status = COMM_ALARM;
	    callbackRequest(&pbus->callback);
	    break;
    }
}

LOCAL void busCallback (
    mbboDirectCanBus_t *pbus
) {
    mbboDirectCanPrivate_t *pcanMbboDirect = pbus->firstPrivate;
    
    while (pcanMbboDirect != NULL) {
	pcanMbboDirect->status = pbus->status;
	dbScanLock((struct dbCommon *) pcanMbboDirect->prec);
	(*((struct rset *) pcanMbboDirect->prec->rset)->process)(pcanMbboDirect->prec);
	dbScanUnlock((struct dbCommon *) pcanMbboDirect->prec);
	pcanMbboDirect = pcanMbboDirect->nextPrivate;
    }
}
