/*******************************************************************************

Project:
    CAN Bus Driver for EPICS

File:
    drvTip810.h

Description:
    Header file for TEWS TIP810 CAN Bus driver.

Author:
    Andrew Johnson <anjohnson@iee.org>
Created:
    20 July 1995
Version:
    $Id: drvTip810.h,v 1.4 2001-02-05 17:19:59 anj Exp $

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


#ifndef INCdrvTip810H
#define INCdrvTip810H

#include <types.h>
#include "canBus.h"


/* Error Numbers */

#ifndef M_t810
#define M_t810			(810<<16)
#endif

#define S_t810_duplicateDevice	(M_t810| 1) /*duplicate t810 device definition*/
#define S_t810_badBusRate 	(M_t810| 2) /*CANbus bit rate not supported*/
#define S_t810_badDevice	(M_t810| 3) /*device pointer is not for t810*/
#define S_t810_transmitterBusy	(M_t810| 4) /*transmit buffer unexpectedly busy*/


extern int t810Status(void *canBusID);
extern int t810Report(int page);
extern int t810Create(char *busName, ushort_t card, ushort_t slot, 
		      ushort_t irqNum, uint_t busRate);
extern int t810Shutdown(int starttype);
extern int t810Initialise(void);

#endif /* INCdrvTip810H */

