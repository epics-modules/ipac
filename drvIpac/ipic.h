/*******************************************************************************

Project:
    CAN Bus Driver for EPICS

File:
    ipic.h

Description:
    IndustryPack Interface Controller ASIC header file, giving the register 
    layout and programming model for the IPIC chip used on the MVME162.

Author:
    Andrew Johnson <anjohnson@iee.org>
Created:
    6 July 1995

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


#ifndef INCipicH
#define INCipicH

#include <types.h>

#ifdef __cplusplus
extern "C" {
#endif


/* Chip Registers */

#define IPIC_CHIP_ID 0x23
#define IPIC_CHIP_REVISION 0x00


/* Interrupt Control Register bits */

#define IPIC_INT_LEVEL 0x07
#define IPIC_INT_ICLR 0x08
#define IPIC_INT_IEN 0x10
#define IPIC_INT_INT 0x20
#define IPIC_INT_EDGE 0x40
#define IPIC_INT_PLTY 0x80


/* General Control Registers bits */

#define IPIC_GEN_MEN 0x01
#define IPIC_GEN_WIDTH 0x0c
#define IPIC_GEN_WIDTH_8 0x04
#define IPIC_GEN_WIDTH_16 0x08
#define IPIC_GEN_WIDTH_32 0x00
#define IPIC_GEN_RT 0x30
#define IPIC_GEN_RT_0 0x00
#define IPIC_GEN_RT_2 0x10
#define IPIC_GEN_RT_4 0x20
#define IPIC_GEN_RT_8 0x30
#define IPIC_GEN_ERR 0x80


/* IP Reset register bits */

#define IPIC_IP_RESET 0x01


/* Chip Structure */

typedef volatile struct {
    uchar_t chipId;
    uchar_t chipRevision;
    uchar_t reserved1[2];
    ushort_t memBase[4];
    uchar_t memSize[4];
    uchar_t intCtrl[4][2];
    uchar_t genCtrl[4];
    uchar_t reserved2[3];
    uchar_t ipReset;
} ipic_t;


#ifdef __cplusplus
}
#endif

#endif /* INCipicH */

