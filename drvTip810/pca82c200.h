/*******************************************************************************

Project:
    CAN Bus Driver for EPICS

File:
    pca82c200.h

Description:
    Philips Stand-alone CAN-controller chip header file, giving the register
    layout and programming model for the chip used on the TIP810 IP module.

Author:
    Andrew Johnson <anjohnson@iee.org>
Created:
    19 July 1995

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


#ifndef INCpca82c200H
#define INCpca82c200H

#include <epicsTypes.h>

#ifdef __cplusplus
extern "C" {
#endif


/***** Control Segment Bit Patterns *****/

/* Control Register */

#define PCA_CR_TM	0x80	/* Test Mode */
#define PCA_CR_S 	0x40	/* Synch */
#define PCA_CR_OIE	0x10	/* Overrun Interrupt Enable */
#define PCA_CR_EIE	0x08	/* Error Interrupt Enable */
#define PCA_CR_TIE	0x04	/* Transmit Interrupt Enable */
#define PCA_CR_RIE	0x02	/* Receive Interrupt Enable */
#define PCA_CR_RR	0x01	/* Reset Request */


/* Command Register */

#define PCA_CMR_GTS	0x10	/* Goto Sleep */
#define PCA_CMR_COS	0x08	/* Clear Overrun Status */
#define PCA_CMR_RRB	0x04	/* Release Receive Buffer */
#define PCA_CMR_AT	0x02	/* Abort Transmission */
#define PCA_CMR_TR	0x01	/* Transmission Request */


/* Status Register */

#define PCA_SR_BS	0x80	/* Bus Status */
#define PCA_SR_ES	0x40	/* Error Status */
#define PCA_SR_TS	0x20	/* Transmit Status */
#define PCA_SR_RS	0x10	/* Receive Status */
#define PCA_SR_TCS	0x08	/* Transmission Complete Status */
#define PCA_SR_TBS	0x04	/* Transmit Buffer Status */
#define PCA_SR_DO	0x02	/* Data Overrun */
#define PCA_SR_RBS	0x01	/* Receive Buffer Status */


/* Interrupt Register */

#define PCA_IR_WUI	0x10	/* Wake-Up Interrupt */
#define PCA_IR_OI	0x08	/* Overrun Interrupt */
#define PCA_IR_EI	0x04	/* Error Interrupt */
#define PCA_IR_TI	0x02	/* Transmit Interrupt */
#define PCA_IR_RI	0x01	/* Receive Interrupt */


/* Bus Timing Register 0 */

#define PCA_BTR0_1M6	0x00	/* 1.6 Mbits/sec,  20 m */
#define PCA_BTR0_1M0	0x00	/* 1.0 Mbits/sec,  40 m */
#define PCA_BTR0_500K	0x00	/* 500 Kbits/sec, 130 m */
#define PCA_BTR0_250K	0x01	/* 250 Kbits/sec, 270 m */
#define PCA_BTR0_125K	0x03	/* 125 Kbits/sec, 530 m */
#define PCA_BTR0_100K	0x43	/* 100 Kbits/sec, 620 m */
#define PCA_BTR0_50K	0x47	/*  50 Kbits/sec, 1.3 km */
#define PCA_BTR0_20K	0x53	/*  20 Kbits/sec, 3.3 km */
#define PCA_BTR0_10K	0x67	/*  10 Kbits/sec, 6.7 km */
#define PCA_BTR0_5K	0x7f	/*   5 Kbits/sec,  10 km */

#define PCA_KVASER_1M0	0x00	/* 1.0 Mbits/sec,  40 m -- Kvaser standard */
#define PCA_KVASER_500K	0x01	/* 500 Kbits/sec, 130 m -- Kvaser standard */
#define PCA_KVASER_250K	0x03	/* 250 Kbits/sec, 270 m -- Kvaser standard */
#define PCA_KVASER_125K	0x07	/* 125 Kbits/sec, 530 m -- Kvaser standard */


/* Bus Timing Register 1 */

#define PCA_BTR1_1M6	0x11	/* 1.6 Mbits/sec,  20 m */
#define PCA_BTR1_1M0	0x14	/* 1.0 Mbits/sec,  40 m */
#define PCA_BTR1_500K	0x1c	/* 500 Kbits/sec, 130 m */
#define PCA_BTR1_250K	0x1c	/* 250 Kbits/sec, 270 m */
#define PCA_BTR1_125K	0x1c	/* 125 Kbits/sec, 530 m */
#define PCA_BTR1_100K	0x2f	/* 100 Kbits/sec, 620 m */
#define PCA_BTR1_50K	0x2f	/*  50 Kbits/sec, 1.3 km */
#define PCA_BTR1_20K	0x2f	/*  20 Kbits/sec, 3.3 km */
#define PCA_BTR1_10K	0x2f	/*  10 Kbits/sec, 6.7 km */
#define PCA_BTR1_5K	0x7f	/*   5 Kbits/sec,  10 km */

#define PCA_BTR1_KVASER	0x23	/* All speeds -- Kvaser standard */

/* Output Control Register */

#define PCA_OCR_OCM_NORMAL	0x02
#define PCA_OCR_OCM_CLOCK	0x03
#define PCA_OCR_OCM_BIPHASE	0x00
#define PCA_OCR_OCM_TEST	0x01

#define PCA_OCR_OCT1_FLOAT	0x00
#define PCA_OCR_OCT1_PULLDOWN	0x40
#define PCA_OCR_OCT1_PULLUP	0x80
#define PCA_OCR_OCT1_PUSHPULL	0xc0

#define PCA_OCR_OCT0_FLOAT	0x00
#define PCA_OCR_OCT0_PULLDOWN	0x08
#define PCA_OCR_OCT0_PULLUP	0x10
#define PCA_OCR_OCT0_PUSHPULL	0x18

#define PCA_OCR_OCP1_INVERT	0x20
#define PCA_OCR_OCP0_INVERT	0x04


/* Message Buffers */

#define PCA_MSG_ID0_RSHIFT	3
#define PCA_MSG_ID1_LSHIFT	5
#define PCA_MSG_ID1_MASK	0xe0
#define PCA_MSG_RTR		0x10
#define PCA_MSG_DLC_MASK	0x0f


/***** Chip Structure *****/

/* Message Buffers */

typedef struct {
    epicsUInt8 pad0;
    epicsUInt8 descriptor0;
    epicsUInt8 pad1;
    epicsUInt8 descriptor1;
    epicsUInt16 data[8];
} msgBuffer_t;


/* Chip Registers */

typedef volatile struct {
    epicsUInt8 pad00;
    epicsUInt8 control;
    epicsUInt8 pad01;
    epicsUInt8 command;
    epicsUInt8 pad02;
    epicsUInt8 status;
    epicsUInt8 pad03;
    epicsUInt8 interrupt;
    epicsUInt8 pad04;
    epicsUInt8 acceptanceCode;
    epicsUInt8 pad05;
    epicsUInt8 acceptanceMask;
    epicsUInt8 pad06;
    epicsUInt8 busTiming0;
    epicsUInt8 pad07;
    epicsUInt8 busTiming1;
    epicsUInt8 pad08;
    epicsUInt8 outputControl;
    epicsUInt8 pad09;
    epicsUInt8 test;
    msgBuffer_t txBuffer;
    msgBuffer_t rxBuffer;
    epicsUInt8 pad30;
    epicsUInt8 notImpl;
    epicsUInt8 pad31;
    epicsUInt8 clockDivider;
} pca82c200_t;


#ifdef __cplusplus
}
#endif

#endif /* INCpca82c200H */

