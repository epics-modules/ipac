/*******************************************************************************

Project:
    CAN Bus Driver for EPICS

File:
    canBus.h

Description:
    CANBUS specific constants

Author:
    Andrew Johnson <anjohnson@iee.org>
Created:
    25 July 1995

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


#ifndef INCcanBusH
#define INCcanBusH

#include "epicsTypes.h"
#include "epicsTimer.h"
#include "shareLib.h"


#define CAN_IDENTIFIERS 2048
#define CAN_DATA_SIZE 8

#define CAN_BUS_OK 0
#define CAN_BUS_ERROR 1
#define CAN_BUS_OFF 2


#ifndef M_can
#define M_can			(811<<16)
#endif

#define S_can_badMessage	(M_can| 1) /*illegal CAN message contents*/
#define S_can_badAddress	(M_can| 2) /*CAN address syntax error*/
#define S_can_noDevice		(M_can| 3) /*CAN bus name does not exist*/
#define S_can_noMessage 	(M_can| 4) /*no matching CAN message callback*/

typedef epicsUInt16 canID_t;
typedef struct canBusID_s *canBusID_t;

typedef struct {
    canID_t identifier;		/* 0 .. 2047 with holes! */
    enum {
	SEND = 0, RTR = 1
    } rtr;			/* Remote Transmission Request */
    epicsUInt8 length;		/* 0 .. 8 */
    epicsUInt8 data[CAN_DATA_SIZE];
} canMessage_t;

typedef struct {
    char *busName;
    double timeout;
    canID_t identifier;
    epicsUInt16 offset;
    epicsInt32 parameter;
    char *paramStr;
    canBusID_t canBusID;
} canIo_t;

typedef void canMsgCallback_t(void *pprivate, const canMessage_t *pmessage);
typedef void canSigCallback_t(void *pprivate, int status);


extern int canSilenceErrors;
extern epicsTimerQueueId canTimerQ;

epicsShareFunc int canOpen(const char *busName, canBusID_t *pbusID);
epicsShareFunc int canBusReset(const char *busName);
epicsShareFunc int canBusStop(const char *busName);
epicsShareFunc int canBusRestart(const char *busName);
epicsShareFunc int canRead(canBusID_t busID, canMessage_t *pmessage, double timeout);
epicsShareFunc int canWrite(canBusID_t busID, const canMessage_t *pmessage,
		    double timeout);
epicsShareFunc int canMessage(canBusID_t busID, canID_t identifier, 
		      canMsgCallback_t callback, void *pprivate);
epicsShareFunc int canMsgDelete(canBusID_t busID, canID_t identifier, 
			canMsgCallback_t callback, void *pprivate);
epicsShareFunc int canSignal(canBusID_t busID, canSigCallback_t callback,
		     void *pprivate);
epicsShareFunc int canIoParse(char *canString, canIo_t *pcanIo);


#endif /* INCcanBusH */

