/*******************************************************************************

Project:
    CAN Bus Driver for EPICS

File:
    drvTip810.c

Description:
    CAN Bus driver for TEWS TIP810 Industry-Pack Module.

Author:
    Andrew Johnson <anjohnson@iee.org>
Created:
    20 July 1995

Copyright (c) 1995-2007 Andrew Johnson

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
#include <ctype.h>
#include <errno.h>

#include <epicsTypes.h>
#include <iocsh.h>
#include <drvSup.h>
#include <devLib.h>
#include <epicsExit.h>
#include <epicsEvent.h>
#include <epicsMutex.h>
#include <epicsTimer.h>
#include <epicsThread.h>
#include <epicsInterrupt.h>
#include <epicsMessageQueue.h>
#include <epicsExport.h>

#include "canBus.h"
#include "drvTip810.h"
#include "drvIpac.h"
#include "pca82c200.h"


/* Some local magic numbers */
#define T810_MAGIC_NUMBER 81001
#define RECV_Q_SIZE 1000	/* Num messages to buffer */

/* These are the IPAC IDs for this module */
#define IP_MANUFACTURER_TEWS 0xb3 
#define IP_MODEL_TEWS_TIP810 0x01


/* EPICS Driver Support Entry Table */

struct drvet drvTip810 = {
    2,
    (DRVSUPFUN) t810Report,
    (DRVSUPFUN) t810Initialise
};
epicsExportAddress(drvet, drvTip810);

epicsTimerQueueId canTimerQ = NULL;


typedef void callback_t(void *pprivate, long parameter);

typedef struct callbackTable_s {
    struct callbackTable_s *pnext;	/* linked list ... */
    void *pprivate;			/* reference for callback routine */
    callback_t *pcallback;		/* registered routine */
} callbackTable_t;


typedef struct canBusID_s {
    struct canBusID_s *pnext;	/* To next device. Must be first member */
    int magicNumber;		/* device pointer confirmation */
    char *pbusName;		/* Bus identification */
    int card;			/* Industry Pack address */
    int slot;			/*     "     "      "    */
    int irqNum; 		/* interrupt vector number */
    int busRate;		/* bit rate of bus in Kbits/sec */
    pca82c200_t *pchip;		/* controller registers */
    epicsEventId txSem;		/* Transmit complete signal */
    int txCount;		/* messages transmitted */
    int rxCount;		/* messages received */
    int overCount;		/* overrun - lost messages */
    int unusedCount;		/* messages without callback */
    canID_t unusedId;		/* last ID received without a callback */
    int errorCount;		/* Times entered Error state */
    int busOffCount;		/* Times entered Bus Off state */
    epicsMutexId readSem;	/* canRead task Mutex */
    canMessage_t *preadBuffer;	/* canRead destination buffer */
    epicsEventId rxSem;		/* canRead message arrival signal */
    callbackTable_t *pmsgHandler[CAN_IDENTIFIERS];	/* message callbacks */
    callbackTable_t *psigHandler;	/* error signal callbacks */
} t810Dev_t;

typedef struct {
   t810Dev_t *pdevice;
   canMessage_t message;
} t810Receipt_t;


static t810Dev_t *pt810First = NULL;
static epicsMessageQueueId receiptQueue = NULL;

int canSilenceErrors = FALSE;	/* for EPICS device support use */
int t810maxQueued = 0;		/* not static so may be reset by operator */

/*******************************************************************************

Routine:
    t810Status

Purpose:
    Return status of given t810 device

Description:
    Returns the status of the t810 device identified by the input parameter, 
    or -1 if not a device ID.

Returns:
    Bit-pattern (0..255) or -1.

*/

int t810Status (
    canBusID_t canBusID
) {
    t810Dev_t *pdevice = canBusID;
    if (canBusID != 0 &&
	pdevice->magicNumber == T810_MAGIC_NUMBER) {
	return pdevice->pchip->status;
    } else {
	return -1;
    }
}


/*******************************************************************************

Routine:
    t810Report

Purpose:
    Report status of all t810 devices

Description:
    Prints a list of all the t810 devices created, their IP carrier &
    slot numbers and the bus name string. For interest > 0 it gives
    additional information about each device.

Returns:
    0, or
    S_t810_badDevice if device list corrupted.

*/

int t810Report (
    int interest
) {
    t810Dev_t *pdevice = pt810First;
    canID_t id;
    int printed;
    int status;

    if (interest > 0) {
	printf("  Receive queue holds %d messages, max %d = %d %% used.\n", 
		RECV_Q_SIZE, t810maxQueued, 
		(100 * t810maxQueued) / RECV_Q_SIZE);
    }

    while (pdevice != NULL) {
	if (pdevice->magicNumber != T810_MAGIC_NUMBER) {
	    printf("t810 device list is corrupt\n");
	    return S_t810_badDevice;
	}

	printf("  '%s' : IP Carrier %hd Slot %hd, Bus rate %d Kbits/sec\n", 
		pdevice->pbusName, pdevice->card, pdevice->slot, 
		pdevice->busRate);

	switch (interest) {
	    case 1:
		printf("\tMessages Sent       : %5d\n", pdevice->txCount);
		printf("\tMessages Received   : %5d\n", pdevice->rxCount);
		printf("\tMessage Overruns    : %5d\n", pdevice->overCount);
		printf("\tDiscarded Messages  : %5d\n", pdevice->unusedCount);
		if (pdevice->unusedCount > 0) {
		    printf("\tLast Discarded ID   : %#5x\n", pdevice->unusedId);
		}
		printf("\tError Interrupts    : %5d\n", pdevice->errorCount);
		printf("\tBus Off Events      : %5d\n", pdevice->busOffCount);
		break;

	    case 2:
		printed = 0;
		printf("\tCallbacks registered: ");
		for (id=0; id < CAN_IDENTIFIERS; id++) {
		    if (pdevice->pmsgHandler[id] != NULL) {
			if (printed % 10 == 0) {
			    printf("\n\t    ");
			}
			printf("0x%-3hx  ", id);
			printed++;
		    }
		}
		if (printed == 0) {
		    printf("None.");
		}
		printf("\n\tcanRead Status : %s\n", 
			pdevice->preadBuffer ? "Active" : "Idle");
		break;

	    case 3:
		printf("    pca82c200 Chip Status:\n");
		status = pdevice->pchip->status;

		printf("\tBus Status             : %s\n", 
			status & PCA_SR_BS ? "Bus-Off" : "Bus-On");
		printf("\tError Status           : %s\n",
			status & PCA_SR_ES ? "Error" : "Ok");
		printf("\tData Overrun           : %s\n",
			status & PCA_SR_DO ? "Overrun" : "Ok");
		printf("\tReceive Status         : %s\n",
			status & PCA_SR_RS ? "Receiving" : "Idle");
		printf("\tReceive Buffer Status  : %s\n",
			status & PCA_SR_RBS ? "Full" : "Empty");
		printf("\tTransmit Status        : %s\n",
			status & PCA_SR_TS ? "Transmitting" : "Idle");
		printf("\tTransmission Complete  : %s\n",
			status & PCA_SR_TCS ? "Complete" : "Incomplete");
		printf("\tTransmit Buffer Access : %s\n",
			status & PCA_SR_TBS ? "Released" : "Locked");
		break;
	}
	pdevice = pdevice->pnext;
    }
    return 0;
}


/*******************************************************************************

Routine:
    t810Create

Purpose:
    Register a new TIP810 device

Description:
    Checks that the given name and card/slot numbers are unique, then
    creates a new device table, initialises it and adds it to the end
    of the linked list.

Returns:
    0,
    ENOMEM if malloc() fails,
    S_t810_badBusRate for an unsupported bus rate,
    S_t810_duplicateDevice if card/slot already used,
    any result from ipmValidate().

Example:
    t810Create "CAN1", 0, 0, 0x60, 500

*/

int t810Create (
    char *pbusName,	/* Unique Identifier for this device */
    int card,		/* Ipac Driver card .. */
    int slot,		/* .. and slot number */
    int irqNum, 	/* interrupt vector number */
    int busRate 	/* in Kbits/sec */
) {
    static const struct {
	int rate;
	epicsUInt8 busTiming0;
	epicsUInt8 busTiming1;
    } rateTable[] = {
	{ 5,    PCA_BTR0_5K,    PCA_BTR1_5K	},
	{ 10,   PCA_BTR0_10K,   PCA_BTR1_10K	},
	{ 20,   PCA_BTR0_20K,   PCA_BTR1_20K	},
	{ 50,   PCA_BTR0_50K,   PCA_BTR1_50K	},
	{ 100,  PCA_BTR0_100K,  PCA_BTR1_100K	},
	{ 125,  PCA_BTR0_125K,  PCA_BTR1_125K	},
	{ 250,  PCA_BTR0_250K,  PCA_BTR1_250K	},
	{ 500,  PCA_BTR0_500K,  PCA_BTR1_500K	},
	{ 1000, PCA_BTR0_1M0,   PCA_BTR1_1M0	},
	{ 1600, PCA_BTR0_1M6,   PCA_BTR1_1M6	},
	{ -125, PCA_KVASER_125K, PCA_BTR1_KVASER},
	{ -250, PCA_KVASER_250K, PCA_BTR1_KVASER},
	{ -500, PCA_KVASER_500K, PCA_BTR1_KVASER},
	{ -1000,PCA_KVASER_1M0,  PCA_BTR1_KVASER},
	{ 0,	0,		0		}
    };
    t810Dev_t *pdevice, *plist = (t810Dev_t *) &pt810First;
    int status, rateIndex, id;

    status = ipmValidate(card, slot, IP_MANUFACTURER_TEWS, 
			 IP_MODEL_TEWS_TIP810);
    if (status) {
	return status;
    }
    /* Slot contains a real TIP810 module */

    if (busRate == 0) {
	return S_t810_badBusRate;
    }
    for (rateIndex = 0; rateTable[rateIndex].rate != busRate; rateIndex++) {
	if (rateTable[rateIndex].rate == 0) {
	    return S_t810_badBusRate;
	}
    }
    /* Bus rate is legal and we now know the right chip settings */

    while (plist->pnext != NULL) {
	plist = plist->pnext;
	if (strcmp(plist->pbusName, pbusName) == 0 || 
	    (plist->card == card && 
	     plist->slot == slot)) {
	    return S_t810_duplicateDevice;
	}
    }
    /* plist now points to the last item in the list */

    pdevice = malloc(sizeof (t810Dev_t));
    if (pdevice == NULL) {
	return ENOMEM;
    }
    /* pdevice is our new device table */

    pdevice->pnext       = NULL;
    pdevice->magicNumber = T810_MAGIC_NUMBER;
    pdevice->pbusName    = pbusName;
    pdevice->card        = card;
    pdevice->slot        = slot;
    pdevice->irqNum      = irqNum;
    pdevice->busRate     = busRate;
    pdevice->pchip       = (pca82c200_t *) ipmBaseAddr(card, slot, ipac_addrIO);
    pdevice->preadBuffer = NULL;
    pdevice->psigHandler = NULL;

    for (id=0; id<CAN_IDENTIFIERS; id++) {
	pdevice->pmsgHandler[id] = NULL;
    }

    pdevice->txSem   = epicsEventCreate(epicsEventFull);
    pdevice->rxSem   = epicsEventCreate(epicsEventEmpty);
    pdevice->readSem = epicsMutexCreate();
    if (pdevice->txSem == NULL ||
	pdevice->rxSem == NULL ||
	pdevice->readSem == NULL) {
	free(pdevice);		/* Ought to free those semaphores, but... */
	return ENOMEM;
    }

    plist->pnext = pdevice;
    /* device table interface stuff filled in and added to list */

    pdevice->pchip->control        = PCA_CR_RR;	/* Reset state */
    pdevice->pchip->acceptanceCode = 0;
    pdevice->pchip->acceptanceMask = 0xff;
    pdevice->pchip->busTiming0     = rateTable[rateIndex].busTiming0;
    pdevice->pchip->busTiming1     = rateTable[rateIndex].busTiming1;
    pdevice->pchip->outputControl  = PCA_OCR_OCM_NORMAL |
				     PCA_OCR_OCT0_PUSHPULL |
				     PCA_OCR_OCT1_PUSHPULL;
    /* chip now initialised, but held in the Reset state */

    ipmIrqCmd(card, slot, 0, ipac_statActive);
    return 0;
}


/*******************************************************************************

Routine:
    t810Shutdown

Purpose:
    Reboot hook routine

Description:
    Stops interrupts and resets the CAN controller chip.

Returns:
    void

*/

void t810Shutdown (
    void *dummy
) {
    t810Dev_t *pdevice = pt810First;

    while (pdevice != NULL) {
	if (pdevice->magicNumber != T810_MAGIC_NUMBER) {
	    return;
	}

	pdevice->pchip->control = PCA_CR_RR;	/* Reset, interrupts off */
	ipmIrqCmd(pdevice->card, pdevice->slot, 0, ipac_statUnused);

	pdevice = pdevice->pnext;
    }
    return;
}


/*******************************************************************************

Routine:
    getRxMessage

Purpose:
    Copy a received message from chip to memory

Description:
    Reads a message from the chip receive buffer into the message buffer 
    and flags to chip to release the buffer for further input.

Returns:
    void

*/

static void getRxMessage (
    pca82c200_t *pchip,
    canMessage_t *pmessage
) {
    epicsUInt8 desc0, desc1;
    int i;

    desc0 = pchip->rxBuffer.descriptor0;
    desc1 = pchip->rxBuffer.descriptor1;

    pmessage->identifier = (desc0 << PCA_MSG_ID0_RSHIFT) |
			   ((desc1 & PCA_MSG_ID1_MASK) >> PCA_MSG_ID1_LSHIFT);
    pmessage->length     = desc1 & PCA_MSG_DLC_MASK;

    if (desc1 & PCA_MSG_RTR) {
	pmessage->rtr = RTR;
    } else {
	pmessage->rtr = SEND;
	for (i=0; i<pmessage->length; i++) {
	    pmessage->data[i] = pchip->rxBuffer.data[i];
	}
    }

    pchip->command = PCA_CMR_RRB;	/* Finished with chip buffer */
}


/*******************************************************************************

Routine:
    putTxMessage

Purpose:
    Copy a message from memory to the chip

Description:
    Copies a message from the message buffer into the chip receive buffer 
    and flags to chip to transmit the message.

Returns:
    void

*/

static void putTxMessage (
    pca82c200_t *pchip,
    const canMessage_t *pmessage
) {
    epicsUInt8 desc0, desc1;
    int i;

    desc0  = pmessage->identifier >> PCA_MSG_ID0_RSHIFT;
    desc1  = (pmessage->identifier << PCA_MSG_ID1_LSHIFT) & PCA_MSG_ID1_MASK;
    desc1 |= pmessage->length & PCA_MSG_DLC_MASK;

    if (pmessage->rtr == SEND) {
	for (i=0; i<pmessage->length; i++) {
	    pchip->txBuffer.data[i] = pmessage->data[i];
	}
    } else {
	desc1 |= PCA_MSG_RTR;
    }

    pchip->txBuffer.descriptor0 = desc0;
    pchip->txBuffer.descriptor1 = desc1;

    pchip->command = PCA_CMR_TR;
}


/*******************************************************************************

Routine:
    doCallbacks

Purpose:
    calls all routines in the given list

Description:


Returns:
    void

*/

static void doCallbacks (
    callbackTable_t *phandler,
    long parameter
) {
    while (phandler != NULL) {
	(*phandler->pcallback)(phandler->pprivate, parameter);
	phandler = phandler->pnext;
    }
}


/*******************************************************************************

Routine:
    t810ISR

Purpose:
    Interrupt Service Routine

Description:


Returns:
    void

*/

static void t810ISR (
    int pdev
) {
    t810Dev_t *pdevice = (t810Dev_t *) pdev;
    int intSource = pdevice->pchip->interrupt;

    if (intSource & PCA_IR_OI) {		/* Overrun Interrupt */
        pdevice->overCount++;
        canBusStop(pdevice->pbusName);		/* Reset the chip but not */
        canBusRestart(pdevice->pbusName);	/* all the counters */

	intSource = pdevice->pchip->interrupt;	/* Rescan interrupts */
    }

    if (intSource & PCA_IR_RI) {		/* Receive Interrupt */
	t810Receipt_t qmsg;

	/* Take a local copy of the message */
	qmsg.pdevice = pdevice;
	getRxMessage(pdevice->pchip, &qmsg.message);

	/* Send it to the servicing task */
	if (epicsMessageQueueTrySend(receiptQueue, &qmsg,
				sizeof(t810Receipt_t)) && !canSilenceErrors)
	    epicsInterruptContextMessage("Warning: CANbus receive queue overflow");
    }

    if (intSource & PCA_IR_EI) {		/* Error Interrupt */
	callbackTable_t *phandler = pdevice->psigHandler;
	int status;

	switch (pdevice->pchip->status & (PCA_SR_ES | PCA_SR_BS)) {
	    case PCA_SR_ES:
		status = CAN_BUS_ERROR;
		pdevice->errorCount++;
		if (!canSilenceErrors)
		    epicsInterruptContextMessage("t810ISR: CANbus error event");
		break;
	    case PCA_SR_BS:
	    case PCA_SR_BS | PCA_SR_ES:
		status = CAN_BUS_OFF;
		pdevice->busOffCount++;
		epicsEventSignal(pdevice->txSem);	/* Signal transmit */
		pdevice->pchip->control &= ~PCA_CR_RR;	/* Clear Reset state */
		if (!canSilenceErrors)
		    epicsInterruptContextMessage("t810ISR: CANbus off event");
		break;
	    default:
		status = CAN_BUS_OK;
		if (!canSilenceErrors)
		    epicsInterruptContextMessage("t810ISR: CANbus OK");
		break;
	}

	doCallbacks(phandler, status);
    }

    if (intSource & PCA_IR_TI) {		/* Transmit Interrupt */
	pdevice->txCount++;
	epicsEventSignal(pdevice->txSem);
    }

    if (intSource & PCA_IR_WUI) {		/* Wake-up Interrupt */
	if (!canSilenceErrors)
	    epicsInterruptContextMessage("Wake-up Interrupt from CANbus");
    }
}


/*******************************************************************************

Routine:
    t810RecvTask

Purpose:
    Receive task

Description:
    This routine is a background task started by t810Initialise. It
    takes messages out of the receive queue one by one and runs the
    callbacks registered against the relevent message ID.

Returns:
    int

*/

static void t810RecvTask(void *dummy) {
    t810Receipt_t rmsg;
    callbackTable_t *phandler;
    int numQueued;

    if (receiptQueue == 0) {
	fprintf(stderr, "CANbus Receive queue does not exist, task exiting.\n");
	return;
    }
    printf("CANbus receive task started\n");

    while (TRUE) {
	numQueued = epicsMessageQueuePending(receiptQueue);
        if (numQueued > t810maxQueued) t810maxQueued = numQueued;

	epicsMessageQueueReceive(receiptQueue, &rmsg, sizeof(t810Receipt_t));
	rmsg.pdevice->rxCount++;

	/* Look up the message ID and do the message callbacks */
	phandler = rmsg.pdevice->pmsgHandler[rmsg.message.identifier];
	if (phandler == NULL) {
	    rmsg.pdevice->unusedId = rmsg.message.identifier;
	    rmsg.pdevice->unusedCount++;
	} else {
	    doCallbacks(phandler, (long) &rmsg.message);
	}

	/* If canRead is waiting for this ID, give it the message and kick it */
	if (rmsg.pdevice->preadBuffer != NULL &&
	    rmsg.pdevice->preadBuffer->identifier == rmsg.message.identifier) {
	    memcpy(rmsg.pdevice->preadBuffer, &rmsg.message, 
		   sizeof(canMessage_t));
	    rmsg.pdevice->preadBuffer = NULL;
	    epicsEventSignal(rmsg.pdevice->rxSem);
	}
   }
}

/*******************************************************************************

Routine:
    t810Initialise

Purpose:
    Initialise driver and all registered hardware

Description:
    Under EPICS this routine is called by iocInit, which must occur
    after all t810Create calls in the startup script.  It completes the
    initialisation of the CAN controller chip and interrupt vector
    registers for all known TIP810 devices and starts the chips
    running.  The receive queue is created and its processing task is
    started to handle incoming data.  An exit hook is used to make
    sure all interrupts are turned off when the IOC is shut down.

Returns:
    int

*/

int t810Initialise (
    void
) {
    t810Dev_t *pdevice = pt810First;
    int status = 0;

    epicsAtExit(t810Shutdown, NULL);

    receiptQueue = epicsMessageQueueCreate(RECV_Q_SIZE, sizeof(t810Receipt_t));
    canTimerQ = epicsTimerQueueAllocate(1, epicsThreadPriorityLow);
    if (receiptQueue == NULL ||
	canTimerQ == NULL) return ENOMEM;

    if (epicsThreadCreate("canRecvTask", epicsThreadPriorityHigh,
			  epicsThreadGetStackSize(epicsThreadStackMedium),
			  t810RecvTask, NULL) == 0) return -1;

    while (pdevice != NULL) {
	pdevice->txCount     = 0;
	pdevice->rxCount     = 0;
	pdevice->overCount   = 0;
	pdevice->unusedCount = 0;
	pdevice->errorCount  = 0;
	pdevice->busOffCount = 0;

	status = ipmIntConnect(pdevice->card, pdevice->slot, pdevice->irqNum,
			       t810ISR, (int)pdevice);

	/* The TIP810's intVec register is external to the PCA82C200 chip */
	*((epicsUInt8 *) pdevice->pchip + 0x41) = pdevice->irqNum;

	ipmIrqCmd(pdevice->card, pdevice->slot, 0, ipac_irqEnable);

	pdevice->pchip->control = PCA_CR_OIE |
				  PCA_CR_EIE |
				  PCA_CR_TIE |
				  PCA_CR_RIE;

	pdevice = pdevice->pnext;
    }
    return status;
}


/*******************************************************************************

Routine:
    canOpen

Purpose:
    Return device pointer for given CAN bus name

Description:
    Searches through the linked list of known t810 devices for one
    which matches the name given, and returns the device pointer
    associated with the relevant device table.

Returns:
    0, or S_can_noDevice if no match found.

Example:
    void *can1;
    status = canOpen("CAN1", &can1);

*/

int canOpen (
    const char *pbusName,
    canBusID_t *pbusID
) {
    t810Dev_t *pdevice = pt810First;

    while (pdevice != NULL) {
	if (strcmp(pdevice->pbusName, pbusName) == 0) {
	    *pbusID = pdevice;
	    return 0;
	}
	pdevice = pdevice->pnext;
    }
    return S_can_noDevice;
}


/*******************************************************************************

Routine:
    canBusReset

Purpose:
    Reset named CANbus

Description:
    Resets the chip and connected to the named bus and all counters

Returns:
    0, or S_can_noDevice if no match found.

Example:
    status = canBusReset("CAN1");

*/

int canBusReset (
    const char *pbusName
) {
    t810Dev_t *pdevice;
    int status = canOpen(pbusName, &pdevice);

    if (status) return status;

    pdevice->pchip->control |=  PCA_CR_RR;    /* Reset the chip */
    pdevice->txCount   = 0;
    pdevice->rxCount   = 0;
    pdevice->overCount   = 0;
    pdevice->unusedCount = 0;
    pdevice->errorCount  = 0;
    pdevice->busOffCount = 0;
    epicsEventSignal(pdevice->txSem);
    pdevice->pchip->control = PCA_CR_OIE |
			      PCA_CR_EIE |
			      PCA_CR_TIE |
			      PCA_CR_RIE;

    return 0;
}


/*******************************************************************************

Routine:
    canBusStop

Purpose:
    Stop I/O on named CANbus

Description:
    Holds the chip for the named bus in Reset state

Returns:
    0, or S_can_noDevice if no match found.

Example:
    status = canBusStop("CAN1");

*/

int canBusStop (
    const char *pbusName
) {
    t810Dev_t *pdevice;
    int status = canOpen(pbusName, &pdevice);

    if (status) return status;

    pdevice->pchip->control |=  PCA_CR_RR;    /* Reset the chip */
    return 0;
}


/*******************************************************************************

Routine:
    canBusRestart

Purpose: 
    Restart I/O on named CANbus

Description:
    Restarts the chip for the named bus after a canBusStop

Returns: 
    0, or S_can_noDevice if no match found.

Example: 
    status = canBusRestart("CAN1");

*/

int canBusRestart (
    const char *pbusName
) {
    t810Dev_t *pdevice;
    int status = canOpen(pbusName, &pdevice);

    if (status) return status;

    pdevice->pchip->control = PCA_CR_OIE |
			      PCA_CR_EIE |
			      PCA_CR_TIE |
			      PCA_CR_RIE;
    epicsEventSignal(pdevice->txSem);

    return 0;
}


/*******************************************************************************

Routine:
    strdupn

Purpose:
    duplicate n characters of a string and return pointer to new substring

Description:
    Copies n characters from the input string to a newly malloc'ed memory
    buffer, and adds a trailing '\0', then returns the new string pointer.

Returns:
    char *newString, or NULL if malloc failed.

*/

static char* strdupn (
    const char *ct,
    size_t n
) {
    char *duplicate;

    duplicate = malloc(n+1);
    if (duplicate == NULL) {
	return NULL;
    }

    memcpy(duplicate, ct, n);
    duplicate[n] = '\0';

    return duplicate;
}


/*******************************************************************************

Routine:
    canIoParse

Purpose:
    Parse a CAN address string into a canIo_t structure

Description:
    canString which must match the format below is converted by this routine
    into the relevent fields of the canIo_t structure pointed to by pcanIo:

    	busname{/timeout}:id{+n}{.offset} parameter

    where
    	busname is alphanumeric, all other fields are hex, decimal or octal
    	timeout is in milliseconds
	id and any number of +n components are summed to give the CAN Id
	offset is the byte offset into the message
	parameter is a string or integer for use by device support

Returns:
    0, or
    S_can_badAddress for illegal input strings,
    ENOMEM if malloc() fails,
    S_can_noDevice for an unregistered bus name.

Example:
    canIoParse("CAN1/20:0126+4+1.4 0xfff", &myIo);

*/

int canIoParse (
    char *canString, 
    canIo_t *pcanIo
) {
    char separator;
    char *name;

    pcanIo->canBusID = NULL;

    if (canString == NULL ||
	pcanIo == NULL) {
	return S_can_badAddress;
    }

    /* Get rid of leading whitespace and non-alphanumeric chars */
    while (!isalnum(0xff & *canString)) {
	if (*canString++ == '\0') {
	    return S_can_badAddress;
	}
    }

    /* First part of string is the bus name */
    name = canString;

    /* find the end of the busName */
    canString = strpbrk(canString, "/:");
    if (canString == NULL ||
	*canString == '\0') {
	return S_can_badAddress;
    }

    /* now we're at character after the end of the busName */
    pcanIo->busName = strdupn(name, canString - name);
    if (pcanIo->busName == NULL) {
	return ENOMEM;
    }
    separator = *canString++;

    /* Handle /<timeout> if present, convert from ms to seconds */
    if (separator == '/') {
	pcanIo->timeout = ((double)strtol(canString, &canString, 0))/1000.0;
	separator = *canString++;
    } else {
	pcanIo->timeout = -1.0;
    }

    /* String must contain :<canID> */
    if (separator != ':') {
	return S_can_badAddress;
    }
    pcanIo->identifier = strtoul(canString, &canString, 0);
    separator = *canString++;

    /* Handle any number of optional +<n> additions to the ID */
    while (separator == '+') {
	pcanIo->identifier += strtol(canString, &canString, 0);
	separator = *canString++;
    }

    /* Handle .<offset> if present */
    if (separator == '.') {
	pcanIo->offset = strtoul(canString, &canString, 0);
	if (pcanIo->offset >= CAN_DATA_SIZE) {
	    return S_can_badAddress;
	}
	separator = *canString++;
    } else {
	pcanIo->offset = 0;
    }

    /* Final parameter is separated by whitespace */
    if (separator != ' ' &&
	separator != '\t') {
	return S_can_badAddress;
    }
    pcanIo->parameter = strtol(canString, &pcanIo->paramStr, 0);

    /* Ok, finally look up the bus name */
    return canOpen(pcanIo->busName, &pcanIo->canBusID);
}


/*******************************************************************************

Routine:
    canWrite

Purpose:
    writes a CAN message to the bus

Description:
    Sends the message described by pmessage out through the bus identified by
    canBusID.  After some simple argument checks it obtains exclusive access to
    the transmit registers, then copies the message to the chip.  The timeout
    value allows task recovery in the event that exclusive access is not
    available within a the given number of seconds.

Returns:
    0, 
    S_can_badMessage for bad identifier, message length or rtr value,
    S_can_badDevice for bad device pointer,
    S_t810_timeout indicates timeout,
    S_t810_transmitterBusy indicates an internal error.

Example:


*/

int canWrite (
    canBusID_t busID,
    const canMessage_t *pmessage,
    double timeout
) {
    t810Dev_t *pdevice = busID;

    if (pdevice->magicNumber != T810_MAGIC_NUMBER) {
	return S_t810_badDevice;
    }

    if (pmessage->identifier >= CAN_IDENTIFIERS ||
	pmessage->length > CAN_DATA_SIZE ||
	(pmessage->rtr != SEND && pmessage->rtr != RTR)) {
	return S_can_badMessage;
    }

    if (epicsEventWaitWithTimeout(pdevice->txSem, timeout) != epicsEventWaitOK) {
	return S_t810_timeout;
    }

    if (pdevice->pchip->status & PCA_SR_TBS) {
	putTxMessage(pdevice->pchip, pmessage);
	return 0;
    }
    epicsEventSignal(pdevice->txSem);
    return S_t810_transmitterBusy;
}


/*******************************************************************************

Routine:
    canMessage

Purpose:
    Register CAN message callback

Description:
    Adds a new callback routine for the given CAN message ID on the
    given device.  There can be any number of callbacks for the same ID,
    and all are called in turn when a message with this ID is
    received.  As a result, the callback routine must not change the
    message at all - it is only permitted to examine it.  The callback
    is called from vxWorks Interrupt Context, thus there are several
    restrictions in what the routine can perform (see vxWorks User
    Guide for details of these).  The callback routine should be
    declared of type canMsgCallback_t
	void callback(void *pprivate, can_Message_t *pmessage); 
    The pprivate value supplied to canMessage is passed to the callback
    routine with each message to allow it to identify its context.

Returns:
    0, 
    S_can_badMessage for bad identifier or NULL callback routine,
    S_t810_badDevice for bad device pointer,
    ENOMEM if malloc() fails.

Example:


*/

int canMessage (
    canBusID_t busID,
    canID_t identifier,
    canMsgCallback_t *pcallback,
    void *pprivate
) {
    t810Dev_t *pdevice = busID;
    callbackTable_t *phandler, *plist;

    if (pdevice->magicNumber != T810_MAGIC_NUMBER) {
	return S_t810_badDevice;
    }

    if (identifier >= CAN_IDENTIFIERS ||
	pcallback == NULL) {
	return S_can_badMessage;
    }

    phandler = malloc(sizeof (callbackTable_t));
    if (phandler == NULL) {
	return ENOMEM;
    }

    phandler->pnext     = NULL;
    phandler->pprivate  = pprivate;
    phandler->pcallback = (callback_t *) pcallback;

    plist = (callbackTable_t *) (&pdevice->pmsgHandler[identifier]);
    while (plist->pnext != NULL) {
	plist = plist->pnext;
    }
    /* plist now points to the last handler in the list */

    plist->pnext = phandler;
    return 0;
}


/*******************************************************************************

Routine:
    canMsgDelete

Purpose:
    Delete registered CAN message callback

Description:
    Deletes an existing callback routine for the given CAN message ID
    on the given device.  The first matching callback found in the list
    is deleted.  To match, the parameters to canMsgDelete must be
    identical to those given to canMessage.

Returns:
    0, 
    S_can_badMessage for bad identifier or NULL callback routine,
    S_can_noMessage for no matching message callback,
    S_t810_badDevice for bad device pointer.

Example:


*/

int canMsgDelete (
    canBusID_t busID,
    canID_t identifier,
    canMsgCallback_t *pcallback,
    void *pprivate
) {
    t810Dev_t *pdevice = busID;
    callbackTable_t *phandler, *plist;

    if (pdevice->magicNumber != T810_MAGIC_NUMBER) {
	return S_t810_badDevice;
    }

    if (identifier >= CAN_IDENTIFIERS ||
	pcallback == NULL) {
	return S_can_badMessage;
    }

    plist = (callbackTable_t *) (&pdevice->pmsgHandler[identifier]);
    while (plist->pnext != NULL) {
	phandler = plist->pnext;
	if (((canMsgCallback_t *)phandler->pcallback == pcallback) &&
	    (phandler->pprivate  == pprivate)) {
	    plist->pnext = phandler->pnext;
	    phandler->pnext = NULL;		/* Just in case... */
	    free(phandler);
	    return 0;
	}
	plist = phandler;
    }

    return S_can_noMessage;
}


/*******************************************************************************

Routine:
    canSignal

Purpose:
    Register CAN error signal callback

Description:
    Adds a new callback routine for the CAN error reports.  There can be
    any number of error callbacks, and all are called in turn when the
    controller chip reports an error or bus Off The callback is called
    from vxWorks Interrupt Context, thus there are restrictions in what
    the routine can perform (see vxWorks User Guide for details of
    these).  The callback routine should be declared a canSigCallback_t
	void callback(void *pprivate, int status);
    The pprivate value supplied to canSignal is passed to the callback
    routine with the error status to allow it to identify its context.
    Status values will be one of
	CAN_BUS_OK,
	CAN_BUS_ERROR or
	CAN_BUS_OFF.
    If the chip goes to the Bus Off state, the driver will attempt to
    restart it.

Returns:
    0,
    S_t810_badDevice for bad device pointer,
    ENOMEM if malloc() fails.

Example:


*/

int canSignal (
    canBusID_t busID,
    canSigCallback_t *pcallback,
    void *pprivate
) {
    t810Dev_t *pdevice = busID;
    callbackTable_t *phandler, *plist;

    if (pdevice->magicNumber != T810_MAGIC_NUMBER) {
	return S_t810_badDevice;
    }

    phandler = malloc(sizeof (callbackTable_t));
    if (phandler == NULL) {
	return ENOMEM;
    }

    phandler->pnext     = NULL;
    phandler->pprivate  = pprivate;
    phandler->pcallback = (callback_t *) pcallback;

    plist = (callbackTable_t *) (&pdevice->psigHandler);
    while (plist->pnext != NULL) {
	plist = plist->pnext;
    }
    /* plist now points to the last handler in the list */

    plist->pnext = phandler;
    return 0;
}


/*******************************************************************************

Routine:
    canRead

Purpose:
    read incoming CAN message, any ID number

Description:
    The simplest way to implement this is have canRead take a message
    ID in the buffer, send an RTR and look for the returned value of
    this message.  This is in keeping with the CAN philosophy and makes
    it useful for simple software interfaces.  More complex ones ought
    to use the canMessage callback functions.

Returns:
    0, or
    S_t810_badDevice for bad bus ID, 
    S_can_badMessage for bad message Identifier or length,
    S_t810_timeout for timeout

Example:
    canMessage_t myBuffer = {
	139,	// Can ID
	0,	// RTR
	4	// Length
    };
    int status = canRead(canID, &myBuffer, WAIT_FOREVER);

*/

int canRead (
    canBusID_t busID,
    canMessage_t *pmessage,
    double timeout
) {
    t810Dev_t *pdevice = busID;
    int status;

    if (pdevice->magicNumber != T810_MAGIC_NUMBER) {
	return S_t810_badDevice;
    }

    if (pmessage->identifier >= CAN_IDENTIFIERS ||
	pmessage->length > CAN_DATA_SIZE) {
	return S_can_badMessage;
    }

    /* Ensure that only one task canRead at once */
    if (epicsMutexLock(pdevice->readSem) != epicsMutexLockOK) {
	return S_t810_badDevice;
    }

    pdevice->preadBuffer = pmessage;

    /* All set for the reply, now send the request */
    pmessage->rtr = RTR;

    status = canWrite(busID, pmessage, timeout);
    if (status == 0) {
	/* Wait for the message to be recieved */
	switch (epicsEventWaitWithTimeout(pdevice->rxSem, timeout)) {
	case epicsEventWaitTimeout:
	    status = S_t810_timeout;
	    break;
	case epicsEventWaitError:
	    status = S_t810_badDevice;
	    break;
	default:
	    break;
	}
    }
    if (status) {
	/* Problem (timeout) sending the RTR or receiving the reply */
	pdevice->preadBuffer = NULL;
	epicsEventTryWait(pdevice->rxSem);	/* Try to clean up */
    }
    epicsMutexUnlock(pdevice->readSem);
    return status;
}


/*******************************************************************************

Routine:
    canTest

Purpose:
    Test routine, sends a single message to the named bus.

Description:
    This routine is intended for use from the vxWorks shell.

Returns:
    0, or ERROR

Example:


*/

int canTest (
    char *pbusName,
    canID_t identifier,
    int rtr,
    int length,
    char *data
) {
    canBusID_t busID;
    canMessage_t message;
    int status;

    if (pbusName == NULL) {
	printf("Usage: canTest \"busname\", id, rtr, len, \"data\"\n");
	return -1;
    }

    status = canOpen(pbusName, &busID);
    if (status) {
	printf("Error %d opening CAN bus '%s'\n", status, pbusName);
	return -1;
    }

    message.identifier = identifier;
    message.rtr        = rtr ? RTR : SEND;
    message.length     = length;

    if (rtr == 0) {
	memcpy(&message.data[0], data, length);
    }

    status = canWrite(busID, &message, 0);
    if (status) {
	printf("Error %d writing message\n", status);
	return -1;
    }
    return 0;
}


/*******************************************************************************
 * EPICS iocsh Command registry
 */

/* t810Create(char *pbusName, int card, int slot, int irqNum, int busRate) */
static const iocshArg t810CreateArg0 = {"busName",iocshArgPersistentString};
static const iocshArg t810CreateArg1 = {"carrier", iocshArgInt};
static const iocshArg t810CreateArg2 = {"slot", iocshArgInt};
static const iocshArg t810CreateArg3 = {"intVector", iocshArgInt};
static const iocshArg t810CreateArg4 = {"busRate", iocshArgInt};
static const iocshArg * const t810CreateArgs[5] = {
    &t810CreateArg0, &t810CreateArg1, &t810CreateArg2, &t810CreateArg3,
    &t810CreateArg4};
static const iocshFuncDef t810CreateFuncDef =
    {"t810Create",5,t810CreateArgs};
static void t810CreateCallFunc(const iocshArgBuf *arg)
{
    t810Create(arg[0].sval, arg[1].ival, arg[2].ival, arg[3].ival, 
	       arg[4].ival);
}

/* t810Report(int interest) */
static const iocshArg t810ReportArg0 = {"interest", iocshArgInt};
static const iocshArg * const t810ReportArgs[1] = {&t810ReportArg0};
static const iocshFuncDef t810ReportFuncDef =
    {"t810Report",1,t810ReportArgs};
static void t810ReportCallFunc(const iocshArgBuf *args)
{
    t810Report(args[0].ival);
}

/* canBusReset(char *pbusName) */
static const iocshArg canBusResetArg0 = {"busName", iocshArgString};
static const iocshArg * const canBusResetArgs[1] = {&canBusResetArg0};
static const iocshFuncDef canBusResetFuncDef =
    {"canBusReset",1,canBusResetArgs};
static void canBusResetCallFunc(const iocshArgBuf *args)
{
    canBusReset(args[0].sval);
}

/* canBusStop(char *pbusName) */
static const iocshArg canBusStopArg0 = {"busName", iocshArgString};
static const iocshArg * const canBusStopArgs[1] = {&canBusStopArg0};
static const iocshFuncDef canBusStopFuncDef =
    {"canBusStop",1,canBusStopArgs};
static void canBusStopCallFunc(const iocshArgBuf *args)
{
    canBusStop(args[0].sval);
}

/* canBusRestart(char *pbusName) */
static const iocshArg canBusRestartArg0 = {"busName", iocshArgString};
static const iocshArg * const canBusRestartArgs[1] = {&canBusRestartArg0};
static const iocshFuncDef canBusRestartFuncDef =
    {"canBusRestart",1,canBusRestartArgs};
static void canBusRestartCallFunc(const iocshArgBuf *args)
{
    canBusRestart(args[0].sval);
}

static void drvTip810Registrar(void) {
    iocshRegister(&t810CreateFuncDef,t810CreateCallFunc);
    iocshRegister(&t810ReportFuncDef,t810ReportCallFunc);
    iocshRegister(&canBusResetFuncDef,canBusResetCallFunc);
    iocshRegister(&canBusStopFuncDef,canBusStopCallFunc);
    iocshRegister(&canBusRestartFuncDef,canBusRestartCallFunc);
}
epicsExportRegistrar(drvTip810Registrar);

